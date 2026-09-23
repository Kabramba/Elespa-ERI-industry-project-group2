/*
 * rx_pin_check.ino
 * -----------------------------------------------------------------------------
 * Is the CAN transceiver's receiver output actually DRIVING the ESP32 RX pin?
 *
 * With no TWAI driver installed, GPIO4 is sampled as a plain input in three
 * configurations. An idle CAN bus is recessive, so a powered, connected
 * transceiver must hold its R output solidly HIGH - strongly enough to beat an
 * internal pulldown (~45 kOhm).
 *
 *   FLOATING  : reads whatever noise is around
 *   PULLUP    : reads HIGH whether or not anything is connected  (not diagnostic alone)
 *   PULLDOWN  : reads HIGH  -> something is actively driving it  = transceiver alive
 *               reads LOW   -> nothing is driving it             = open / unpowered
 *
 * No CAN traffic is generated; this transmits nothing at all.
 */

#include <Arduino.h>

#define RX_PIN 4          // ESP32 CAN RX  <- transceiver R / RXD / CRX
#define TX_PIN 5          // ESP32 CAN TX  -> transceiver D / TXD / CTX

static void sample(const char *label, int pin, int mode) {
  pinMode(pin, mode);
  delay(50);
  uint32_t high = 0, edges = 0;
  int prev = digitalRead(pin);
  const uint32_t N = 200000;
  for (uint32_t i = 0; i < N; i++) {
    const int v = digitalRead(pin);
    if (v) high++;
    if (v != prev) { edges++; prev = v; }
  }
  Serial.printf("  GPIO%-2d %-9s : high %6.2f%%   edges %lu\n",
                pin, label, high * 100.0 / N, (unsigned long)edges);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) delay(10);
  delay(200);
  Serial.printf("\n# rx_pin_check - is the transceiver driving the RX pin?\n");
  Serial.printf("# no TWAI driver installed, nothing transmitted\n\n");

  Serial.printf("RX pin (should be driven HIGH by a live transceiver on an idle bus):\n");
  sample("FLOATING", RX_PIN, INPUT);
  sample("PULLUP",   RX_PIN, INPUT_PULLUP);
  sample("PULLDOWN", RX_PIN, INPUT_PULLDOWN);

  Serial.printf("\nTX pin (ESP32 output when idle; here sampled as input for reference):\n");
  sample("FLOATING", TX_PIN, INPUT);
  sample("PULLDOWN", TX_PIN, INPUT_PULLDOWN);

  pinMode(RX_PIN, INPUT_PULLDOWN);
  delay(50);
  const bool driven = digitalRead(RX_PIN) == HIGH;
  Serial.printf("\n===================== VERDICT =====================\n");
  if (driven)
    Serial.printf("GPIO%d is HELD HIGH against an internal pulldown.\n"
                  "  -> The transceiver IS powered and IS driving the RX line.\n"
                  "     The fault is on the bus/differential side: termination,\n"
                  "     CANH/CANL polarity, or stub length.\n", RX_PIN);
  else
    Serial.printf("GPIO%d is NOT being driven - it follows the internal pulldown.\n"
                  "  -> The transceiver's receiver output is not reaching this pin.\n"
                  "     Check: module VCC = 3.3 V, GND shared, the jumper into GPIO%d,\n"
                  "     and that GPIO%d lands on the module's R/RXD/CRX pin.\n",
                  RX_PIN, RX_PIN, RX_PIN);
  Serial.printf("===================================================\n");
}

void loop() { delay(1000); }
