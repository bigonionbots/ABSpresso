#pragma once

/**
 * @file AudioLinkProtocol.h
 * @brief Wire protocol between a host board and the ATOM Lite audio module.
 *
 * This repository is its home. Hosts include a verbatim copy, so keep it dependency-free C++ and
 * keep changes backwards compatible: new fields go at the end of a payload, and receivers accept
 * payloads at least as long as the struct they know.
 *
 * The same frames run over either transport the module offers (USB through the ATOM's bridge, or
 * a plain UART on its Grove port), 8N1 at BAUD unless both ends are built otherwise.
 *
 * Frame (little endian):
 *
 *   0xA5 0x5A | type u8 | seq u8 | length u16 | payload[length] | crc16 u16
 *
 * crc16 is CRC-16/CCITT-FALSE over type, seq, length and payload. A receiver hunts for the
 * magic, drops frames with a bad CRC or an oversized length, and resynchronises on the next magic.
 *
 * Host -> module ("host" frames) are reliable, go-back-N: every frame carries the next sequence
 * number; the module accepts only the expected one and answers with ACK(nextSeq, rxFree). An
 * out-of-sequence frame (one before it was lost or damaged) gets an ACK flagged OUT_OF_SEQUENCE,
 * on which the host resends its window from nextSeq at once; RESEND_TIMEOUT_MS without progress
 * is only the fallback. The host keeps at most MAX_INFLIGHT_BYTES unacknowledged, so draining
 * them never takes long on a slow bridge (resending a window that was merely slow collapses the
 * link). Audio flow control rides on the same ACK: the host keeps the audio bytes in
 * unacknowledged frames at or below the last rxFree.
 *
 * Module -> host frames are small and idempotent (ACK, periodic STATUS, scan results), so they
 * carry seq 0 and are never resent. Keep them short: some bridges (the ATOM Lite's FTDI
 * emulation, see the README) have no flow control in that direction.
 *
 * Positions: STREAM_START names a stream id and the book position the data starts at. The
 * module counts playedMs from that point in source samples actually sent to the output, so
 * host position = startMs + STATUS.playedMs for the matching stream id.
 *
 * Live streams (Codec::MuLaw, game sound): AUDIO_DATA carries MuLaw.h bytes, one per sample,
 * and STREAM_START gives their sample rate and channel count. The module keeps such a stream close to
 * real time: it drops blocks rather than let more than LIVE_MAX_BUFFERED_MS wait for the output.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace audiolink {

constexpr uint8_t MAGIC0 = 0xA5;
constexpr uint8_t MAGIC1 = 0x5A;
constexpr size_t HEADER_BYTES = 6;  // magic(2) type seq length(2)
constexpr size_t CRC_BYTES = 2;
constexpr size_t MAX_PAYLOAD = 1024;
constexpr size_t MAX_FRAME = HEADER_BYTES + MAX_PAYLOAD + CRC_BYTES;
constexpr uint32_t BAUD = 500000;
constexpr uint8_t PROTOCOL_VERSION = 3;

/** Without an acknowledged step forward for this long the host resends its window (fallback). */
constexpr uint32_t RESEND_TIMEOUT_MS = 1000;
/** Fast resends (OUT_OF_SEQUENCE ACKs) at most this often, so a burst of them costs one resend. */
constexpr uint32_t FAST_RESEND_GAP_MS = 100;
/** Unacknowledged host bytes, all frames counted. */
constexpr uint32_t MAX_INFLIGHT_BYTES = 8192;
/** Maximum unacknowledged host frames (must stay below 128 for the 8-bit sequence space). */
constexpr uint8_t MAX_WINDOW_FRAMES = 48;
/** Audio bytes per AUDIO_DATA frame the host should aim for. */
constexpr size_t AUDIO_CHUNK = 512;
/** A live stream (Codec::MuLaw) waiting for the output beyond this is dropped as it arrives. */
constexpr uint32_t LIVE_MAX_BUFFERED_MS = 150;
/** Module STATUS period. */
constexpr uint32_t STATUS_PERIOD_MS = 250;

enum class Type : uint8_t {
  // Host -> module (reliable)
  Ping = 0x01,
  StreamStart = 0x10,  ///< StreamStartPayload
  AudioData = 0x11,    ///< streamId u8 + codec bytes
  StreamEnd = 0x12,    ///< streamId u8: no more data; play out what is buffered
  StreamStop = 0x13,   ///< streamId u8: drop everything now
  Pause = 0x14,
  Resume = 0x15,
  SetVolume = 0x16,  ///< u8 0..100
  SetOutput = 0x17,  ///< u8 Output
  BtScan = 0x20,     ///< u8 1 = start, 0 = stop
  BtConnect = 0x21,  ///< 6-byte address
  BtDisconnect = 0x22,
  BtForget = 0x23,  ///< forget the remembered device

  // Module -> host (unreliable, seq 0)
  Hello = 0x80,     ///< HelloPayload, sent at boot and in answer to Ping
  Ack = 0x81,       ///< AckPayload
  Status = 0x82,    ///< StatusPayload
  BtDevice = 0x83,  ///< BtDevicePayload, one per discovered device
  Log = 0x84,       ///< UTF-8 text, for the host's log
};

enum class Codec : uint8_t {
  Mp3 = 1,
  AacAdts = 2,
  MuLaw = 3,  ///< Live PCM, MuLaw.h bytes (mono), rate in StreamStartPayload
};

enum class Output : uint8_t {
  Bluetooth = 1,
  I2s = 2,
  Null = 3,  ///< Consumes PCM in real time and discards it (link and decoder testing)
};

/** The link the module heard the host on (HELLO, from module firmware 0.2). */
enum class Transport : uint8_t {
  Unknown = 0,  ///< Not yet known (an older module sends a shorter HELLO: treat a missing field as this)
  Usb = 1,      ///< Through the ATOM's USB-to-UART bridge
  Uart = 2,     ///< The Grove port UART
};

enum class PlayState : uint8_t {
  Idle = 0,       ///< No stream
  Buffering = 1,  ///< Stream started, waiting for enough data or for the output
  Playing = 2,
  Paused = 3,
  Draining = 4,  ///< StreamEnd received, playing out the buffer
  Finished = 5,  ///< Stream played out completely
};

enum class BtState : uint8_t {
  Off = 0,
  Idle = 1,  ///< Stack up, not connected, not scanning
  Scanning = 2,
  Connecting = 3,
  Connected = 4,
};

#pragma pack(push, 1)
struct HelloPayload {
  uint8_t protocolVersion;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint8_t outputs;  ///< bit (1 << Output) for each output this build supports
  uint32_t rxBufferBytes;
  /**
   * The sequence number the module expects next; the host continues from it. HELLO goes out at
   * boot as well as in answer to PING, and a host that pinged a module still booting takes the
   * boot HELLO as the answer, so it must not assume its own PING got through.
   */
  uint8_t nextSeq;
  uint8_t transport;  ///< Transport, from module firmware 0.2 (absent before: Unknown)
};

constexpr uint8_t ACK_OUT_OF_SEQUENCE = 0x01;  ///< A frame arrived out of sequence: resend from nextSeq

struct AckPayload {
  uint8_t nextSeq;  ///< First sequence number not yet accepted
  uint32_t rxFree;  ///< Free bytes in the module's compressed-audio buffer
  uint8_t flags;    ///< ACK_* bits
};

struct StreamStartPayload {
  uint8_t streamId;
  uint8_t codec;  ///< Codec
  uint32_t startMs;
  // Protocol 3: for Codec::MuLaw (compressed codecs carry their own; send 0 for them).
  uint32_t sampleRate;
  uint8_t channels;
};

struct StatusPayload {
  uint8_t streamId;
  uint8_t playState;  ///< PlayState
  uint8_t output;     ///< Output
  uint8_t btState;    ///< BtState
  uint8_t volume;
  uint8_t channels;     ///< Source channels, 0 until the first frame decodes
  uint32_t sampleRate;  ///< Source sample rate, 0 until the first frame decodes
  uint32_t playedMs;    ///< Source time sent to the output since StreamStart
  uint32_t rxBuffered;  ///< Compressed bytes waiting
  uint16_t pcmBufferedMs;
  uint16_t underruns;
  uint16_t decodeErrors;  ///< For a live stream: drops (audio that arrived while too much was waiting)
  uint32_t freeHeap;
  uint8_t btAddress[6];  ///< Connected or remembered device, zeros if none
};

struct BtDevicePayload {
  uint8_t address[6];
  int8_t rssi;
  char name[33];  ///< NUL-terminated
};
#pragma pack(pop)

/** CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). */
inline uint16_t crc16(const uint8_t* data, size_t length, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

/** Writes a complete frame into out (at least HEADER_BYTES + length + CRC_BYTES). Returns its size. */
inline size_t encode(uint8_t* out, const Type type, const uint8_t seq, const void* payload, const size_t length) {
  out[0] = MAGIC0;
  out[1] = MAGIC1;
  out[2] = static_cast<uint8_t>(type);
  out[3] = seq;
  out[4] = static_cast<uint8_t>(length);
  out[5] = static_cast<uint8_t>(length >> 8);
  if (length > 0) std::memcpy(out + HEADER_BYTES, payload, length);
  const uint16_t crc = crc16(out + 2, HEADER_BYTES - 2 + length);
  out[HEADER_BYTES + length] = static_cast<uint8_t>(crc);
  out[HEADER_BYTES + length + 1] = static_cast<uint8_t>(crc >> 8);
  return HEADER_BYTES + length + CRC_BYTES;
}

/**
 * Incremental frame parser. Feed bytes; when push() returns true a complete, CRC-checked frame
 * is available through type()/seq()/payload()/length() until the next push().
 */
class Parser {
 public:
  bool push(const uint8_t byte) {
    switch (state_) {
      case State::Magic0:
        if (byte == MAGIC0) state_ = State::Magic1;
        return false;
      case State::Magic1:
        state_ = byte == MAGIC1 ? State::Header : (byte == MAGIC0 ? State::Magic1 : State::Magic0);
        filled_ = 2;
        buffer_[0] = MAGIC0;
        buffer_[1] = MAGIC1;
        return false;
      case State::Header:
        buffer_[filled_++] = byte;
        if (filled_ < HEADER_BYTES) return false;
        length_ = static_cast<uint16_t>(buffer_[4] | (buffer_[5] << 8));
        if (length_ > MAX_PAYLOAD) {
          ++badFrames_;
          state_ = State::Magic0;
          return false;
        }
        state_ = State::Body;
        return false;
      case State::Body: {
        buffer_[filled_++] = byte;
        if (filled_ < HEADER_BYTES + length_ + CRC_BYTES) return false;
        state_ = State::Magic0;
        const uint16_t expected = crc16(buffer_ + 2, HEADER_BYTES - 2 + length_);
        const uint16_t got =
            static_cast<uint16_t>(buffer_[HEADER_BYTES + length_] | (buffer_[HEADER_BYTES + length_ + 1] << 8));
        if (expected != got) {
          ++badFrames_;
          return false;
        }
        return true;
      }
    }
    return false;
  }

  Type type() const { return static_cast<Type>(buffer_[2]); }
  uint8_t seq() const { return buffer_[3]; }
  uint16_t length() const { return length_; }
  const uint8_t* payload() const { return buffer_ + HEADER_BYTES; }
  uint32_t badFrames() const { return badFrames_; }

 private:
  enum class State : uint8_t { Magic0, Magic1, Header, Body };
  State state_ = State::Magic0;
  uint8_t buffer_[MAX_FRAME] = {};
  size_t filled_ = 0;
  uint16_t length_ = 0;
  uint32_t badFrames_ = 0;
};

/** Sequence a is before b in the 8-bit space (window < 128). */
inline bool seqBefore(const uint8_t a, const uint8_t b) { return static_cast<int8_t>(a - b) < 0; }

}  // namespace audiolink
