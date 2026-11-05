#pragma once

// Host end of the ATOM Lite audio module link, for Bluetooth headphones (and the module's wired
// DAC). The module is github.com/bigonionbots/atom-lite-audio-module; AudioLinkProtocol.h here is a
// verbatim copy of its protocol/AudioLinkProtocol.h.
//
// The link runs over UART0 on the board's UART header (TXD GPIO43 -> module G32, RXD GPIO44 <-
// module G26, GND), 8N1 at 500000 baud. A link task pings until the module answers with HELLO,
// keeps the go-back-N send window (in PSRAM) and caches the module's STATUS and Bluetooth scan
// results. Every call here is thread-safe.
//
// The link only starts once the module is chosen as the audio output, so the UART pins stay free
// on devices without one.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Values match audiolink::Codec / Output / PlayState / BtState (checked in audio_module.cpp).
typedef enum { AM_CODEC_MP3 = 1, AM_CODEC_AAC_ADTS = 2 } am_codec_t;
typedef enum { AM_OUT_BLUETOOTH = 1, AM_OUT_I2S = 2, AM_OUT_NULL = 3 } am_output_t;
typedef enum {
    AM_PLAY_IDLE,
    AM_PLAY_BUFFERING,
    AM_PLAY_PLAYING,
    AM_PLAY_PAUSED,
    AM_PLAY_DRAINING,
    AM_PLAY_FINISHED,
} am_play_state_t;
typedef enum { AM_BT_OFF, AM_BT_IDLE, AM_BT_SCANNING, AM_BT_CONNECTING, AM_BT_CONNECTED } am_bt_state_t;

// Audio bytes per audio_module_send_audio() call, at most.
#define AM_AUDIO_CHUNK 512

typedef struct {
    bool ready;   // link up: the module answered and commands can flow
    bool valid;   // ready, and a STATUS arrived recently (the fields below are current)
    uint8_t fw_major, fw_minor;
    uint8_t stream_id;   // 0 when no stream
    uint8_t play_state;  // am_play_state_t
    uint8_t output;      // am_output_t
    uint8_t bt_state;    // am_bt_state_t
    uint8_t volume;
    uint32_t played_ms;  // source time played since STREAM_START of stream_id
    uint32_t rx_buffered;
    uint16_t pcm_ms;
    uint16_t underruns;
    uint8_t bt_address[6];  // connected or remembered headset, zeros if none
} am_status_t;

typedef struct {
    uint8_t address[6];
    int8_t rssi;
    char name[33];
} am_bt_device_t;

// Reads the saved output choice and starts the link if the module is the output.
void audio_module_init(void);
// Playback goes to the module (true) or the board's speaker (false). Saved.
bool audio_module_enabled(void);
void audio_module_set_enabled(bool enabled);
// Where the module plays: Bluetooth or its wired DAC. Saved here and sent to the module now and
// whenever the link reconnects.
am_output_t audio_module_output(void);
void audio_module_set_output(am_output_t output);

// Changes every time the link (re)connects, e.g. after the module rebooted: a stream in flight
// is gone and has to be restarted.
uint32_t audio_module_session(void);
void audio_module_get_status(am_status_t *out);

// Reliable frames. Audio waits up to timeout_ms for window room and buffer credit from the
// module. Commands only wait if the window is full (they never wait for credit). All return
// false when the link is down or the wait timed out.
bool audio_module_stream_start(uint8_t stream, am_codec_t codec, uint32_t start_ms);
bool audio_module_send_audio(uint8_t stream, const uint8_t *data, size_t len, uint32_t timeout_ms);
bool audio_module_stream_end(uint8_t stream);
bool audio_module_stream_stop(uint8_t stream);
bool audio_module_pause(bool paused);
bool audio_module_set_volume(int volume);

// Bluetooth. Scan results collect until cleared; connecting stops the scan. The module remembers
// the headset and reconnects to it on its own.
bool audio_module_bt_scan(bool on);
bool audio_module_bt_connect(const uint8_t address[6]);
bool audio_module_bt_forget(void);
int audio_module_bt_devices(am_bt_device_t *out, int max);
void audio_module_bt_clear(void);
// The name a scan reported for this address, if any.
bool audio_module_bt_name(const uint8_t address[6], char *out, size_t len);

#ifdef __cplusplus
}
#endif
