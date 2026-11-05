// MPEG-TS -> audio elementary stream. See ts_audio.h.

#include "ts_audio.h"

#include <string.h>

#define TS_PACKET    188
#define MAX_JUNK     65536  // give up skipping (and just play) if no frame header turns up in this much

void ts_audio_init(ts_audio_t *ts, double skip_seconds)
{
    memset(ts, 0, sizeof(*ts));
    ts->pmt_pid = -1;
    ts->audio_pid = -1;
    ts->skip_seconds = skip_seconds;
    ts->skipping = skip_seconds > 0;
}

/* ---------- audio frame headers (for skipping) ---------- */

// ADTS: 7-byte header with the frame length; 1-4 raw blocks of 1024 samples.
static bool parse_adts(const uint8_t *h, uint32_t *len, double *seconds)
{
    static const int rates[13] = {96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000,
                                  7350};
    if (h[0] != 0xFF || (h[1] & 0xF6) != 0xF0) return false;
    const int sfi = (h[2] >> 2) & 0x0F;
    if (sfi >= 13) return false;
    *len = ((uint32_t)(h[3] & 0x03) << 11) | ((uint32_t)h[4] << 3) | (h[5] >> 5);
    if (*len < 7) return false;
    *seconds = (double)(((h[6] & 0x03) + 1) * 1024) / rates[sfi];
    return true;
}

// MPEG audio Layer III (what ABS stream-copies for MP3 sources).
static bool parse_mp3(const uint8_t *h, uint32_t *len, double *seconds)
{
    static const int kbps_v1[15] = {0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320};
    static const int kbps_v2[15] = {0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160};
    static const int rates[3] = {44100, 48000, 32000};
    if (h[0] != 0xFF || (h[1] & 0xE0) != 0xE0) return false;
    const int version = (h[1] >> 3) & 3;  // 3: MPEG-1, 2: MPEG-2, 0: MPEG-2.5
    const int layer = (h[1] >> 1) & 3;    // 1: Layer III
    const int bri = h[2] >> 4, sri = (h[2] >> 2) & 3, pad = (h[2] >> 1) & 1;
    if (version == 1 || layer != 1 || bri == 0 || bri == 15 || sri == 3) return false;
    const bool v1 = version == 3;
    const int rate = rates[sri] >> (v1 ? 0 : version == 2 ? 1 : 2);
    const int kbps = v1 ? kbps_v1[bri] : kbps_v2[bri];
    *len = (uint32_t)((v1 ? 144000 : 72000) * kbps / rate + pad);
    *seconds = (double)(v1 ? 1152 : 576) / rate;
    return true;
}

static bool emit(ts_audio_t *ts, const uint8_t *data, size_t len, ts_audio_out_t out, void *ctx)
{
    while (len > 0 && ts->skipping) {
        if (ts->frame_left) {
            const uint32_t n = len < ts->frame_left ? (uint32_t)len : ts->frame_left;
            data += n;
            len -= n;
            ts->frame_left -= n;
            continue;
        }
        const int need = ts->codec == TS_AUDIO_AAC ? 7 : 4;
        while (ts->hdr_fill < need && len > 0) {
            ts->hdr[ts->hdr_fill++] = *data++;
            len--;
        }
        if (ts->hdr_fill < need) return true;
        uint32_t frame_len;
        double seconds;
        const bool ok = ts->codec == TS_AUDIO_AAC ? parse_adts(ts->hdr, &frame_len, &seconds)
                                                  : parse_mp3(ts->hdr, &frame_len, &seconds);
        if (!ok) {
            // Not on a frame boundary: slide one byte along.
            memmove(ts->hdr, ts->hdr + 1, --ts->hdr_fill);
            if (++ts->junk > MAX_JUNK) ts->skipping = false;
            continue;
        }
        if (ts->skipped + seconds / 2 <= ts->skip_seconds && frame_len >= (uint32_t)ts->hdr_fill) {
            ts->skipped += seconds;
            ts->frame_left = frame_len - ts->hdr_fill;
            ts->hdr_fill = 0;
            continue;
        }
        ts->skipping = false;  // this frame plays
    }
    if (ts->hdr_fill) {
        if (!out(ctx, ts->hdr, ts->hdr_fill)) return false;
        ts->hdr_fill = 0;
    }
    return len == 0 || out(ctx, data, len);
}

/* ---------- TS ---------- */

// Start of a PSI section in a packet payload (after the pointer field), and where it ends.
static const uint8_t *psi_section(const uint8_t *payload, int plen, const uint8_t **end)
{
    if (plen < 1 || 1 + payload[0] + 3 > plen) return NULL;
    const uint8_t *sec = payload + 1 + payload[0];
    const int sec_len = ((sec[1] & 0x0F) << 8) | sec[2];
    const uint8_t *e = sec + 3 + sec_len - 4;  // before the CRC
    if (e > payload + plen) e = payload + plen;
    *end = e;
    return sec;
}

static void parse_pat(ts_audio_t *ts, const uint8_t *payload, int plen)
{
    const uint8_t *end, *sec = psi_section(payload, plen, &end);
    if (!sec || sec[0] != 0x00) return;
    for (const uint8_t *e = sec + 8; e + 4 <= end; e += 4) {
        const int program = (e[0] << 8) | e[1];
        if (program != 0) {  // 0 is the network PID
            ts->pmt_pid = ((e[2] & 0x1F) << 8) | e[3];
            return;
        }
    }
}

static void parse_pmt(ts_audio_t *ts, const uint8_t *payload, int plen)
{
    const uint8_t *end, *sec = psi_section(payload, plen, &end);
    if (!sec || sec[0] != 0x02 || sec + 12 > end) return;
    const int info_len = ((sec[10] & 0x0F) << 8) | sec[11];
    for (const uint8_t *es = sec + 12 + info_len; es + 5 <= end;) {
        const int type = es[0];
        const int pid = ((es[1] & 0x1F) << 8) | es[2];
        const int es_info = ((es[3] & 0x0F) << 8) | es[4];
        if (type == 0x0F || type == 0x03 || type == 0x04) {
            ts->audio_pid = pid;
            ts->codec = type == 0x0F ? TS_AUDIO_AAC : TS_AUDIO_MP3;
            return;
        }
        es += 5 + es_info;
    }
}

static bool packet(ts_audio_t *ts, ts_audio_out_t out, void *ctx)
{
    const uint8_t *p = ts->pkt;
    const bool start = p[1] & 0x40;
    const int pid = ((p[1] & 0x1F) << 8) | p[2];
    const int afc = (p[3] >> 4) & 3;
    if (!(afc & 1)) return true;  // no payload
    int i = 4;
    if (afc == 3) i += 1 + p[4];
    if (i >= TS_PACKET) return true;
    const uint8_t *payload = p + i;
    int plen = TS_PACKET - i;

    if (pid == 0) {
        if (start) parse_pat(ts, payload, plen);
    } else if (pid == ts->pmt_pid) {
        if (start) parse_pmt(ts, payload, plen);
    } else if (pid == ts->audio_pid) {
        if (start) {
            // PES header: 00 00 01, stream id, length, flags, header data length, header data.
            if (plen < 9 || payload[0] || payload[1] || payload[2] != 1) return true;
            const int header = 9 + payload[8];
            if (header > plen) return true;
            payload += header;
            plen -= header;
        }
        if (plen > 0) return emit(ts, payload, plen, out, ctx);
    }
    return true;
}

bool ts_audio_feed(ts_audio_t *ts, const uint8_t *data, size_t len, ts_audio_out_t out, void *ctx)
{
    while (len > 0) {
        if (ts->fill == 0) {
            // Find the sync byte (resynchronises after damaged data).
            const uint8_t *sync = memchr(data, 0x47, len);
            if (!sync) return true;
            len -= sync - data;
            data = sync;
        }
        int n = TS_PACKET - ts->fill;
        if ((size_t)n > len) n = (int)len;
        memcpy(ts->pkt + ts->fill, data, n);
        ts->fill += n;
        data += n;
        len -= n;
        if (ts->fill == TS_PACKET) {
            ts->fill = 0;
            if (!packet(ts, out, ctx)) return false;
        }
    }
    return true;
}
