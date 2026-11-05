// Host end of the ATOM Lite audio module link over UART0. See audio_module.h.
//
// Ported from the T5 host (inx-pro AudioLink.cpp), with UART in place of USB: there is no
// disconnect event, so a link that has heard nothing for LINK_SILENCE_MS (the module sends STATUS
// every 250 ms) is taken as gone and pinged again.

#include "audio_module.h"

#include <new>
#include <string.h>
#include "AudioLinkProtocol.h"
extern "C" {
#include "config.h"  // C header without C++ guards
}
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

static const char *TAG = "amod";

#define LINK_UART      UART_NUM_0
#define PIN_LINK_TX    43  // UART header TXD -> module G32
#define PIN_LINK_RX    44  // UART header RXD <- module G26
#define LINK_RX_BUFFER 1024  // module -> host is small: ACKs, STATUS, scan results

static_assert(AM_CODEC_MP3 == (int)audiolink::Codec::Mp3 && AM_CODEC_AAC_ADTS == (int)audiolink::Codec::AacAdts, "");
static_assert(AM_OUT_BLUETOOTH == (int)audiolink::Output::Bluetooth && AM_OUT_I2S == (int)audiolink::Output::I2s &&
                  AM_OUT_NULL == (int)audiolink::Output::Null,
              "");
static_assert(AM_PLAY_FINISHED == (int)audiolink::PlayState::Finished, "");
static_assert(AM_BT_CONNECTED == (int)audiolink::BtState::Connected, "");
static_assert(AM_AUDIO_CHUNK + 1 <= audiolink::MAX_PAYLOAD, "");

namespace {

using audiolink::Type;

constexpr uint32_t PING_INTERVAL_MS = 500;
constexpr uint32_t LINK_SILENCE_MS = 2000;
constexpr uint32_t STATUS_STALE_MS = 1500;
constexpr uint32_t COMMAND_TIMEOUT_MS = 100;
constexpr int MAX_BT_DEVICES = 16;

struct Slot {
    uint8_t seq;
    uint16_t size;
    uint16_t audioBytes;
    uint8_t frame[audiolink::MAX_FRAME];
};

SemaphoreHandle_t lock;      // everything below
SemaphoreHandle_t progress;  // given on every ACK, so senders wake up
TaskHandle_t task;
Slot *window;  // PSRAM
size_t windowHead, windowCount;
uint8_t nextSeq;
uint32_t rxFree, inflightAudio, inflightBytes;
uint32_t lastFastResendMs, lastProgressMs, lastPingMs, lastRxMs;
bool linkReady, pingOutstanding;
uint32_t session;
audiolink::HelloPayload hello;
audiolink::StatusPayload lastStatus;
uint32_t lastStatusMs;
bool statusValid;
am_bt_device_t devices[MAX_BT_DEVICES];
int deviceCount;
audiolink::Parser *parser;

bool enabled;
uint8_t output = AM_OUT_BLUETOOTH;
bool outputPending;  // send SET_OUTPUT once the link is up
// The headset last connected from here, so Settings can name it after a reboot (no scan needed).
uint8_t savedAddress[6];
char savedName[33];

uint32_t nowMs() { return (uint32_t)(esp_timer_get_time() / 1000); }

struct Guard {
    Guard() { xSemaphoreTake(lock, portMAX_DELAY); }
    ~Guard() { xSemaphoreGive(lock); }
};

void writeFrame(const uint8_t *frame, size_t size) { uart_write_bytes(LINK_UART, frame, size); }

void clearWindowLocked()
{
    windowHead = windowCount = 0;
    inflightAudio = inflightBytes = 0;
    rxFree = 0;
}

void resetSessionLocked()
{
    linkReady = false;
    pingOutstanding = false;
    statusValid = false;
    clearWindowLocked();
}

void sendPingLocked()
{
    uint8_t frame[audiolink::HEADER_BYTES + audiolink::CRC_BYTES];
    const size_t size = audiolink::encode(frame, Type::Ping, nextSeq, nullptr, 0);
    nextSeq = (uint8_t)(nextSeq + 1);
    clearWindowLocked();
    pingOutstanding = true;
    lastPingMs = nowMs();
    writeFrame(frame, size);
}

void resendWindowLocked()
{
    for (size_t i = 0; i < windowCount; ++i) {
        const Slot &slot = window[(windowHead + i) % audiolink::MAX_WINDOW_FRAMES];
        writeFrame(slot.frame, slot.size);
    }
    lastProgressMs = nowMs();
}

void handleAck(const uint8_t *payload, uint16_t length)
{
    if (length < sizeof(audiolink::AckPayload)) return;
    audiolink::AckPayload ack;
    memcpy(&ack, payload, sizeof(ack));
    Guard guard;
    if (!linkReady) return;
    bool advanced = false;
    while (windowCount > 0 && audiolink::seqBefore(window[windowHead].seq, ack.nextSeq)) {
        inflightAudio -= window[windowHead].audioBytes;
        inflightBytes -= window[windowHead].size;
        windowHead = (windowHead + 1) % audiolink::MAX_WINDOW_FRAMES;
        --windowCount;
        advanced = true;
    }
    rxFree = ack.rxFree;
    if (advanced || windowCount == 0) lastProgressMs = nowMs();
    const bool inWindow = windowCount == 0 || (uint8_t)(ack.nextSeq - window[windowHead].seq) <= windowCount;
    if ((ack.flags & audiolink::ACK_OUT_OF_SEQUENCE) && !inWindow) {
        // The module expects a sequence number we never sent: the ends are out of step, and
        // resending can't fix that. Start a new session.
        ESP_LOGW(TAG, "sequence out of step (module wants %u), reconnecting", ack.nextSeq);
        resetSessionLocked();
    } else if ((ack.flags & audiolink::ACK_OUT_OF_SEQUENCE) && windowCount > 0 &&
               nowMs() - lastFastResendMs >= audiolink::FAST_RESEND_GAP_MS) {
        // The module saw a gap: resend now rather than at the timeout.
        lastFastResendMs = nowMs();
        resendWindowLocked();
    }
    xSemaphoreGive(progress);
}

void handleHello(const uint8_t *payload, uint16_t length)
{
    // Older modules send a shorter HELLO (no transport byte); take what's there.
    const size_t required = offsetof(audiolink::HelloPayload, nextSeq) + 1;
    if (length < required) return;
    Guard guard;
    memset(&hello, 0, sizeof(hello));
    memcpy(&hello, payload, length < sizeof(hello) ? length : sizeof(hello));
    if (pingOutstanding) {
        pingOutstanding = false;
        // Continue from where the module is, not necessarily after our PING (see HelloPayload).
        nextSeq = hello.nextSeq;
        clearWindowLocked();
        linkReady = true;
        outputPending = true;
        ++session;
        ESP_LOGI(TAG, "module %u.%u ready (protocol %u)", hello.firmwareMajor, hello.firmwareMinor,
                 hello.protocolVersion);
    } else {
        // Nobody asked: the module rebooted. Start a new session; the player restarts its stream.
        ESP_LOGW(TAG, "module restarted");
        resetSessionLocked();
    }
}

void handleBtDevice(const uint8_t *payload, uint16_t length)
{
    if (length < sizeof(audiolink::BtDevicePayload)) return;
    audiolink::BtDevicePayload found;
    memcpy(&found, payload, sizeof(found));
    found.name[sizeof(found.name) - 1] = 0;
    Guard guard;
    for (int i = 0; i < deviceCount; i++) {
        if (memcmp(devices[i].address, found.address, 6) == 0) {
            devices[i].rssi = found.rssi;
            if (found.name[0]) strlcpy(devices[i].name, found.name, sizeof(devices[i].name));
            return;
        }
    }
    if (deviceCount >= MAX_BT_DEVICES) return;
    am_bt_device_t &d = devices[deviceCount++];
    memcpy(d.address, found.address, 6);
    d.rssi = found.rssi;
    strlcpy(d.name, found.name, sizeof(d.name));
}

void handleFrame()
{
    const uint8_t *payload = parser->payload();
    const uint16_t length = parser->length();
    switch (parser->type()) {
    case Type::Ack:
        handleAck(payload, length);
        break;
    case Type::Hello:
        handleHello(payload, length);
        break;
    case Type::Status:
        if (length >= sizeof(audiolink::StatusPayload)) {
            Guard guard;
            memcpy(&lastStatus, payload, sizeof(lastStatus));
            lastStatusMs = nowMs();
            statusValid = true;
        }
        break;
    case Type::BtDevice:
        handleBtDevice(payload, length);
        break;
    case Type::Log:
        ESP_LOGI(TAG, "module: %.*s", (int)length, (const char *)payload);
        break;
    default:
        break;
    }
}

bool command(Type type, const void *payload, size_t length);

void linkTask(void *)
{
    uint8_t chunk[256];
    for (;;) {
        {
            Guard guard;
            const uint32_t now = nowMs();
            if (linkReady && now - lastRxMs > LINK_SILENCE_MS) {
                ESP_LOGW(TAG, "module silent, reconnecting");
                resetSessionLocked();
            }
            if (!linkReady && now - lastPingMs >= PING_INTERVAL_MS) sendPingLocked();
            if (windowCount > 0 && now - lastProgressMs >= audiolink::RESEND_TIMEOUT_MS) resendWindowLocked();
        }
        const int got = uart_read_bytes(LINK_UART, chunk, sizeof(chunk), pdMS_TO_TICKS(10));
        for (int i = 0; i < got; ++i) {
            if (parser->push(chunk[i])) {
                lastRxMs = nowMs();
                handleFrame();
            }
        }
        if (outputPending) {
            const uint8_t v = output;
            if (command(Type::SetOutput, &v, 1)) outputPending = false;
        }
    }
}

bool trySendLocked(Type type, const uint8_t *prefix, size_t prefixLength, const uint8_t *payload, size_t length,
                   uint16_t audioBytes)
{
    if (!linkReady || windowCount >= audiolink::MAX_WINDOW_FRAMES) return false;
    const size_t frameSize = audiolink::HEADER_BYTES + prefixLength + length + audiolink::CRC_BYTES;
    if (audioBytes > 0) {
        // Only audio is held back by the in-flight limit and the module's credit; a command is a
        // few bytes and must get through while audio is waiting for room.
        if (windowCount > 0 && inflightBytes + frameSize > audiolink::MAX_INFLIGHT_BYTES) return false;
        if (inflightAudio + audioBytes > rxFree) return false;
    }
    Slot &slot = window[(windowHead + windowCount) % audiolink::MAX_WINDOW_FRAMES];
    uint8_t body[audiolink::MAX_PAYLOAD];
    if (prefixLength) memcpy(body, prefix, prefixLength);
    if (length) memcpy(body + prefixLength, payload, length);
    slot.seq = nextSeq;
    slot.size = (uint16_t)audiolink::encode(slot.frame, type, nextSeq, body, prefixLength + length);
    slot.audioBytes = audioBytes;
    nextSeq = (uint8_t)(nextSeq + 1);
    if (windowCount == 0) lastProgressMs = nowMs();
    ++windowCount;
    inflightAudio += audioBytes;
    inflightBytes += slot.size;
    writeFrame(slot.frame, slot.size);
    return true;
}

bool sendWithPrefix(Type type, const uint8_t *prefix, size_t prefixLength, const uint8_t *payload, size_t length,
                    uint16_t audioBytes, uint32_t timeoutMs)
{
    if (!task || prefixLength + length > audiolink::MAX_PAYLOAD) return false;
    const uint32_t start = nowMs();
    for (;;) {
        {
            Guard guard;
            if (!linkReady) return false;
            if (trySendLocked(type, prefix, prefixLength, payload, length, audioBytes)) return true;
        }
        const uint32_t waited = nowMs() - start;
        if (waited >= timeoutMs) return false;
        const uint32_t left = timeoutMs - waited;
        xSemaphoreTake(progress, pdMS_TO_TICKS(left < 20 ? left : 20));
    }
}

bool command(Type type, const void *payload, size_t length)
{
    return sendWithPrefix(type, nullptr, 0, (const uint8_t *)payload, length, 0, COMMAND_TIMEOUT_MS);
}

void startLink()
{
    if (task) return;
    window = (Slot *)heap_caps_malloc(sizeof(Slot) * audiolink::MAX_WINDOW_FRAMES, MALLOC_CAP_SPIRAM);
    // The parser holds a 1 KB frame buffer: PSRAM, explicitly (malloc keeps blocks < 4 KB internal).
    void *parserMem = heap_caps_malloc(sizeof(audiolink::Parser), MALLOC_CAP_SPIRAM);
    if (parserMem) parser = new (parserMem) audiolink::Parser();
    const uart_config_t cfg = {
        .baud_rate = (int)audiolink::BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {},
    };
    if (!window || !parser || uart_driver_install(LINK_UART, LINK_RX_BUFFER, 0, 0, nullptr, 0) != ESP_OK ||
        uart_param_config(LINK_UART, &cfg) != ESP_OK ||
        uart_set_pin(LINK_UART, PIN_LINK_TX, PIN_LINK_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "can't start the module link");
        return;
    }
    gpio_pullup_en((gpio_num_t)PIN_LINK_RX);  // idle high when no module is wired
    // Stack in PSRAM: UART and memory work only, never the flash.
    xTaskCreatePinnedToCoreWithCaps(linkTask, "amod_link", 4096, nullptr, 5, &task, 0, MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG, "link on UART0 (TX %d, RX %d) at %lu baud", PIN_LINK_TX, PIN_LINK_RX,
             (unsigned long)audiolink::BAUD);
}

void saveSettings(void *)
{
    nvs_handle_t h;
    if (nvs_open("amod", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, "on", enabled ? 1 : 0);
        nvs_set_u8(h, "out", output);
        nvs_commit(h);
        nvs_close(h);
    }
}

void saveHeadset(void *)
{
    nvs_handle_t h;
    if (nvs_open("amod", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, "btaddr", savedAddress, sizeof(savedAddress));
        nvs_set_str(h, "btname", savedName);
        nvs_commit(h);
        nvs_close(h);
    }
}

}  // namespace

extern "C" {

void audio_module_init(void)
{
    lock = xSemaphoreCreateMutex();
    progress = xSemaphoreCreateBinary();
    uint8_t on = 0;
    nvs_handle_t h;
    if (nvs_open("amod", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, "on", &on);
        nvs_get_u8(h, "out", &output);
        size_t n = sizeof(savedAddress);
        nvs_get_blob(h, "btaddr", savedAddress, &n);
        n = sizeof(savedName);
        if (nvs_get_str(h, "btname", savedName, &n) != ESP_OK) savedName[0] = 0;
        nvs_close(h);
    }
    enabled = on;
    if (enabled) startLink();
}

bool audio_module_enabled(void) { return enabled; }

void audio_module_set_enabled(bool on)
{
    if (on == enabled) return;
    enabled = on;
    if (on) startLink();  // stays up once started; it costs nothing when the speaker is used
    flash_safe(saveSettings, nullptr);
}

uint32_t audio_module_session(void)
{
    if (!lock) return 0;
    Guard guard;
    return session;
}

void audio_module_get_status(am_status_t *out)
{
    memset(out, 0, sizeof(*out));
    if (!lock) return;
    Guard guard;
    out->ready = linkReady;
    out->valid = linkReady && statusValid && nowMs() - lastStatusMs < STATUS_STALE_MS;
    out->fw_major = hello.firmwareMajor;
    out->fw_minor = hello.firmwareMinor;
    const audiolink::StatusPayload &s = lastStatus;
    out->stream_id = s.streamId;
    out->play_state = s.playState;
    out->output = s.output;
    out->bt_state = s.btState;
    out->volume = s.volume;
    out->played_ms = s.playedMs;
    out->rx_buffered = s.rxBuffered;
    out->pcm_ms = s.pcmBufferedMs;
    out->underruns = s.underruns;
    memcpy(out->bt_address, s.btAddress, 6);
}

bool audio_module_stream_start(uint8_t stream, am_codec_t codec, uint32_t start_ms)
{
    audiolink::StreamStartPayload p{};
    p.streamId = stream;
    p.codec = (uint8_t)codec;
    p.startMs = start_ms;
    return command(Type::StreamStart, &p, sizeof(p));
}

bool audio_module_send_audio(uint8_t stream, const uint8_t *data, size_t len, uint32_t timeout_ms)
{
    if (len > AM_AUDIO_CHUNK) return false;
    return sendWithPrefix(Type::AudioData, &stream, 1, data, len, (uint16_t)(len + 1), timeout_ms);
}

bool audio_module_stream_end(uint8_t stream) { return command(Type::StreamEnd, &stream, 1); }
bool audio_module_stream_stop(uint8_t stream) { return command(Type::StreamStop, &stream, 1); }
bool audio_module_pause(bool paused) { return command(paused ? Type::Pause : Type::Resume, nullptr, 0); }

bool audio_module_set_volume(int volume)
{
    const uint8_t v = (uint8_t)(volume < 0 ? 0 : volume > 100 ? 100 : volume);
    return command(Type::SetVolume, &v, 1);
}

am_output_t audio_module_output(void) { return (am_output_t)output; }

void audio_module_set_output(am_output_t out)
{
    if ((uint8_t)out == output) return;
    output = (uint8_t)out;
    outputPending = true;  // the link task sends it
    flash_safe(saveSettings, nullptr);
}

bool audio_module_bt_scan(bool on)
{
    const uint8_t v = on ? 1 : 0;
    return command(Type::BtScan, &v, 1);
}

bool audio_module_bt_connect(const uint8_t address[6])
{
    if (!command(Type::BtConnect, address, 6)) return false;
    char name[33] = "";
    audio_module_bt_name(address, name, sizeof(name));
    {
        Guard guard;
        memcpy(savedAddress, address, 6);
        strlcpy(savedName, name, sizeof(savedName));
    }
    flash_safe(saveHeadset, nullptr);
    return true;
}
bool audio_module_bt_forget(void) { return command(Type::BtForget, nullptr, 0); }

int audio_module_bt_devices(am_bt_device_t *out, int max)
{
    if (!lock) return 0;
    Guard guard;
    const int n = deviceCount < max ? deviceCount : max;
    memcpy(out, devices, n * sizeof(*out));
    return n;
}

void audio_module_bt_clear(void)
{
    if (!lock) return;
    Guard guard;
    deviceCount = 0;
}

bool audio_module_bt_name(const uint8_t address[6], char *out, size_t len)
{
    if (!lock) return false;
    Guard guard;
    for (int i = 0; i < deviceCount; i++) {
        if (memcmp(devices[i].address, address, 6) == 0 && devices[i].name[0]) {
            strlcpy(out, devices[i].name, len);
            return true;
        }
    }
    if (savedName[0] && memcmp(savedAddress, address, 6) == 0) {
        strlcpy(out, savedName, len);
        return true;
    }
    return false;
}

}  // extern "C"
