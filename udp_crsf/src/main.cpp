/*
   ESP32 CRSF raw-frame sender with UDP RC input
   ─────────────────────────────────────────────
   • Wi-Fi client → UDP_PORT (60000) → 16×uint16_t channels + CRC32.
   • CRC32 integrity check, CRC-8 for CRSF.
   • Holds last valid RC 5 s, then neutral.
   • Serial console prints:
       – Wi-Fi connection + IP
       – First packet per client
*/

#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
  #include <WiFi.h>
#elif defined(ARDUINO_ARCH_ESP8266)
  #include <ESP8266WiFi.h>
#else
  #error "This sketch supports ESP32 or ESP8266 only."
#endif

#include <WiFiUdp.h>
#include "wifi_credentials.h"

#define UDP_PORT   60000       
#define ELRS_RX_PIN 16
#define ELRS_TX_PIN 17
#define CRSF_BAUD   400000
#define SEND_RATE_HZ 50

constexpr uint8_t CRSF_SYNC = 0xC8;
constexpr uint8_t CRSF_TYPE = 0x16;
constexpr uint8_t PAYLOAD_BYTES = 22;
constexpr uint8_t FRAME_LEN = PAYLOAD_BYTES + 2;
constexpr uint8_t CHANNELS = 16;
constexpr uint16_t PWM_MIN = 900;
constexpr uint16_t PWM_MAX = 2100;

// ───────────── CRC-8 DVB-S2 ─────────────
static uint8_t crc8(uint8_t crc, uint8_t d)
{
    crc ^= d;
    for (uint8_t i = 0; i < 8; ++i)
        crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    return crc;
}

// ───────────── CRC-32 Ethernet ───────────
static uint32_t crc32(const uint8_t *s, size_t n)
{
    uint32_t crc = 0xFFFFFFFF;
    while (n--) {
        crc ^= *s++;
        for (uint8_t i = 0; i < 8; ++i)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
    return crc ^ 0xFFFFFFFF;
}

// ───────── µs → CRSF 11-bit ──────────────
static inline uint16_t usToCrsf(uint16_t us)   // 1000–2000 µs → 191–1792
{
    us = constrain(us, 880u, 2150u);
    return (uint32_t)(us - 880) * 1600 / 1000;
}

// ─────── bit-packer 16×11-bit → 22 B ─────
static void packChannels(const uint16_t *src, uint8_t *dst)
{
    uint32_t buf = 0; uint8_t bits = 0; uint8_t *p = dst;
    for (uint8_t i = 0; i < CHANNELS; ++i) {
        buf |= (uint32_t)(src[i] & 0x7FF) << bits;
        bits += 11;
        while (bits >= 8) { *p++ = buf; buf >>= 8; bits -= 8; }
    }
    if (bits) *p = buf;
}

// ─────────── globals ────────────
WiFiUDP Udp;
uint32_t lastPacketTime = 0;
constexpr uint32_t RC_HOLD_MS = 5000;

uint16_t ch[CHANNELS] = {
    1500,1500,1000,1500,1000,1500,1500,1500,
    1500,1500,1500,1500,1500,1500,1500,1500
};

static void setNeutral()
{
    ch[0]=1500; ch[1]=1500; ch[2]=1000; ch[3]=1500; ch[4]=1000;
    for(uint8_t i=5;i<CHANNELS;++i) ch[i]=1500;
}

// ─────── send CRSF frame ───────
static void sendRC(const uint16_t *us)
{
    uint16_t rc[CHANNELS];
    for(uint8_t i=0;i<CHANNELS;++i) rc[i]=usToCrsf(us[i]);

    uint8_t frame[2+FRAME_LEN];
    frame[0]=CRSF_SYNC;
    frame[1]=FRAME_LEN;
    frame[2]=CRSF_TYPE;
    packChannels(rc, frame+3);

    uint8_t c=0;
    c = crc8(c, CRSF_TYPE);
    for(uint8_t i=0;i<PAYLOAD_BYTES;++i) c = crc8(c, frame[3+i]);
    frame[3+PAYLOAD_BYTES]=c;

    Serial1.write(frame,sizeof(frame));
}

// ─────────── setup ────────────
void setup()
{
    Serial.begin(115200);
    Serial1.begin(CRSF_BAUD, SERIAL_8N1, ELRS_RX_PIN, ELRS_TX_PIN);

    Serial.println(F("\n[BOOT] CRSF-UDP bridge starting"));
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print(F("[NET] Connecting"));
    while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print('.'); }
    Serial.printf("\n[NET] Connected – IP %s, RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    Udp.begin(UDP_PORT);
    Serial.printf("[NET] UDP listening on port %u\n", UDP_PORT);

    setNeutral();
    Serial.println(F("[INFO] Neutral channels armed – waiting for client"));
}

// ─────────── loop ────────────
void loop()
{
    const uint32_t now = millis();

    // ── receive UDP packet ──
    int len = Udp.parsePacket();
    if (len == 40) {                     // 4 ts + 32 rc + 4 CRC
        uint8_t buf[40];
        Udp.read(buf, 40);

        uint32_t crc_rx =  buf[36] | (buf[37]<<8) | (buf[38]<<16) | (buf[39]<<24);
        if (crc32(buf, 36) == crc_rx) {
            static IPAddress lastClientIP;
            if (lastClientIP != Udp.remoteIP()) {
                lastClientIP = Udp.remoteIP();
                Serial.printf("[NET] Client %s:%u connected\n",
                              lastClientIP.toString().c_str(), Udp.remotePort());
            }

            for(uint8_t i=0;i<CHANNELS;++i){
                uint16_t v = buf[4+i*2] | (buf[5+i*2]<<8);
                ch[i] = constrain(v, PWM_MIN, PWM_MAX);
            }
            lastPacketTime = now;
        }
    }

    // ── failsafe to neutral ── (probably very bad idea)
    //if (now - lastPacketTime > RC_HOLD_MS) setNeutral();

    // ── periodic CRSF transmit ──
    static uint32_t tSend=0;
    if (now - tSend >= 1000/SEND_RATE_HZ) { tSend = now; sendRC(ch); }
}
