#pragma once

// MPEG-TS -> audio elementary stream, for sending ABS's HLS segments to the audio module
// undecoded. AAC in TS is already ADTS-framed, so the payload bytes go out as they are.
//
// ts_audio_feed() takes any slice of a TS byte stream (segments back to back), follows the
// PAT/PMT to the first AAC (stream type 0x0F) or MP3 (0x03/0x04) stream and hands its PES
// payload to `out`. Skipping drops whole audio frames from the start until `skip_seconds` of
// audio is gone (to the nearest frame), which is how playback starts mid-segment.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum { TS_AUDIO_UNKNOWN, TS_AUDIO_AAC, TS_AUDIO_MP3 } ts_audio_codec_t;

// Returns false to stop feeding (e.g. playback was aborted).
typedef bool (*ts_audio_out_t)(void *ctx, const uint8_t *data, size_t len);

typedef struct {
    // TS packet assembly
    uint8_t pkt[188];
    int fill;
    int pmt_pid, audio_pid;
    ts_audio_codec_t codec;
    // frame skipping
    double skip_seconds;   // still to skip
    double skipped;        // skipped so far
    bool skipping;
    uint8_t hdr[8];
    int hdr_fill;
    uint32_t frame_left;   // bytes of the current skipped frame still to drop
    uint32_t junk;         // bytes dropped while looking for a frame header
} ts_audio_t;

void ts_audio_init(ts_audio_t *ts, double skip_seconds);
// False when `out` asked to stop.
bool ts_audio_feed(ts_audio_t *ts, const uint8_t *data, size_t len, ts_audio_out_t out, void *ctx);
