/*
 * can_id_stats.ino
 * -----------------------------------------------------------------------------
 * The capability sweep found 0x103, 0x108, 0x10B and 0x10E silent after 5 retries
 * each, while neighbouring ids answered. At the measured ~70 % per-attempt hit
 * rate, five straight misses is a 0.24 % event - so four of them is not bad luck.
 * This measures each id independently: N single attempts, no retries, so the hit
 * rate per id is directly comparable against known-good controls.
 *
 * Read-only: remote frames only, all inside 0x100-0x110.
 */

#include <Arduino.h>
#include <string.h>
#include "driver/twai.h"

#define BMS_RX_PIN  18
#define BMS_TX_PIN  17
#define TWAI_TX_PIN 5
#define TWAI_RX_PIN 4

static constexpr int N = 25;

HardwareSerial &BmsSerial = Serial1;

struct Target { uint32_t id; uint8_t dlc; const char *note; };
static const Target TARGETS[] = {
  { 0x100, 8, "control - pack"          },
  { 0x103, 8, "SUSPECT - MOS/date/sw"   },
  { 0x104, 4, "control - config"        },
  { 0x107, 8, "control - cells 1-3"     },
  { 0x108, 8, "SUSPECT - cells 4-6"     },
  { 0x109, 8, "control - cells 7-9"     },
  { 0x10B, 8, "SUSPECT - cells 13-15"   },
  { 0x10C, 8, "control - cells 16-17"   },
  { 0x10E, 8, "SUSPECT - cells 22-24"   },
};

static inline uint16_t be16(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }
static uint16_t crc16Modbus(const uint8_t *p, uint8_t n) {
  uint16_t c = 0xFFFF;
  while (n--) { c ^= *p++; for (uint8_t i = 0; i < 8; i++) c = (c&1) ? uint16_t((c>>1)^0xA001) : uint16_t(c>>1); }
  return c;
}
static uint16_t jbdChecksum(uint8_t f, uint8_t l, const uint8_t *d) {
  uint16_t c = 0; c -= f; c -= l;
  for (uint16_t i = 0; i < l; i++) c -= d[i];
  return c;
}
static bool rdB(uint8_t &b, uint32_t dl) {
  while ((int32_t)(dl-millis()) > 0) { const int c = BmsSerial.read(); if (c>=0) { b=c; return true; } delay(1); }
  return false;
}
static int uartRead(uint8_t reg, uint8_t *out) {
  const uint16_t c = jbdChecksum(reg,0,nullptr);
  const uint8_t rq[7] = {0xDD,0xA5,reg,0x00,uint8_t(c>>8),uint8_t(c&0xFF),0x77};
  while (BmsSerial.available()) BmsSerial.read();
  BmsSerial.write(rq,sizeof(rq));
  const uint32_t dl = millis()+500; uint8_t buf[300],t; uint16_t n=0;
  for(;;){ if(!rdB(t,dl)) return -1; if(t==0xDD) break; }
  buf[n++]=t;
  while(n<4){ if(!rdB(t,dl)) return -1; buf[n++]=t; }
  const uint16_t tot = 4+buf[3]+3;
  while(n<tot){ if(!rdB(t,dl)) return -1; buf[n++]=t; }
  if(buf[1]!=reg||buf[2]!=0x00) return -1;
  memcpy(out,buf+4,buf[3]);
  return buf[3];
}

// single attempt, no retry
static int once(uint32_t id, uint8_t dlc, twai_message_t &m) {
  uint32_t a=0; twai_read_alerts(&a,0);
  twai_message_t q={};
  q.identifier=id; q.rtr=1; q.ss=1; q.data_length_code=dlc;
  if (twai_transmit(&q,pdMS_TO_TICKS(50))!=ESP_OK) return -1;
  a=0; twai_read_alerts(&a,pdMS_TO_TICKS(15));
  const uint32_t dl = millis()+110;
  while ((int32_t)(dl-millis())>0) {
    twai_message_t r;
    if (twai_receive(&r,pdMS_TO_TICKS(5))!=ESP_OK) continue;
    if (r.rtr || r.identifier!=id) continue;
    m=r; return 1;
  }
  twai_status_info_t st; twai_get_status_info(&st);
  if (st.state==TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); delay(60); twai_start(); }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis()<3000) delay(10);
  delay(200);
  Serial.printf("\n# can_id_stats - %d single attempts per id, no retries\n\n", N);

  BmsSerial.begin(9600, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN,(gpio_num_t)TWAI_RX_PIN,TWAI_MODE_NORMAL);
  g.rx_queue_len=32; g.alerts_enabled=TWAI_ALERT_TX_SUCCESS|TWAI_ALERT_TX_FAILED;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  if (twai_driver_install(&g,&t,&f)==ESP_OK) twai_start();

  uint8_t d[260];
  Serial.printf("UART wake: %s\n\n", uartRead(0x03,d)>=23 ? "OK" : "FAILED");

  Serial.printf("%-6s %-22s %6s %7s  %s\n","id","note","hits","rate","last payload");
  for (size_t i=0;i<sizeof(TARGETS)/sizeof(TARGETS[0]);i++) {
    const Target &T = TARGETS[i];
    int hit=0; twai_message_t last={}; bool have=false;
    for (int k=0;k<N;k++) {
      twai_message_t m;
      if (once(T.id,T.dlc,m)==1) { hit++; last=m; have=true; }
      delay(25);
    }
    Serial.printf("0x%03lX  %-22s %3d/%-2d %6d%%  ",(unsigned long)T.id,T.note,hit,N,hit*100/N);
    if (have) {
      // print only real payload: DLC minus the 2 CRC bytes
      const int payload = int(last.data_length_code) - 2;
      for (int b=0;b<payload;b++) Serial.printf("%02X ", last.data[b]);
      if (payload<=0) Serial.printf("(empty frame, DLC %u)", last.data_length_code);
      else {
        const uint8_t n=last.data_length_code-2;
        Serial.printf(" crc %s", crc16Modbus(last.data,n)==((uint16_t(last.data[n])<<8)|last.data[n+1]) ? "ok":"BAD");
      }
    } else Serial.printf("-- never answered --");
    Serial.printf("\n");
  }
  Serial.printf("\n# done\n");
}

void loop(){ delay(1000); }
