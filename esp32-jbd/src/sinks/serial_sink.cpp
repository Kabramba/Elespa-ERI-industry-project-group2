#include "serial_sink.h"

#include <Arduino.h>

namespace app {
namespace {

struct FieldLabel {
  uint16_t    bit;
  const char *name;
};
const FieldLabel kFieldLabels[] = {
    {jbd::kFieldPack, "pack"},     {jbd::kFieldCapacity, "capacity"},
    {jbd::kFieldProtect, "prot"},  {jbd::kFieldMos, "mos"},
    {jbd::kFieldConfig, "config"}, {jbd::kFieldTemps, "temps"},
    {jbd::kFieldCells, "cells"},
};

}  // namespace

void SerialSink::onSnapshot(const jbd::BmsSnapshot &s) {
  using namespace jbd;

  // A field group that did not arrive prints as "--", never as 0. On a lossy
  // link the difference between "the MOSFET is off" and "that frame never came
  // back" matters.
  Serial.printf("[%s] ", sourceName(s.source));
  if (hasFields(s, kFieldPack))
    Serial.printf("%.2f V  %+.2f A  %+.1f W", s.packVolts, s.currentAmps,
                  s.packVolts * s.currentAmps);
  else
    Serial.printf("pack --");

  if (hasFields(s, kFieldCapacity))
    Serial.printf("   SoC %u %%   %.2f / %.2f Ah   cycles %u", s.socPercent, s.remainingAh,
                  s.fullAh, s.cycles);
  else
    Serial.printf("   SoC --");
  Serial.printf("\n");

  Serial.printf("       ");
  if (hasFields(s, kFieldMos))
    Serial.printf("MOS chg %s dsg %s", s.chargeMosOn ? "ON" : "off",
                  s.dischargeMosOn ? "ON" : "off");
  else
    Serial.printf("MOS --");

  if (hasFields(s, kFieldProtect)) {
    Serial.printf("   prot 0x%04X", s.protection);
    for (uint8_t i = 0; i < 13; ++i)
      if (s.protection & (1u << i)) Serial.printf(" %s", protectionName(i));
  } else {
    Serial.printf("   prot --");
  }
  Serial.printf("\n");

  Serial.printf("       temps ");
  if (hasFields(s, kFieldTemps))
    for (uint8_t i = 0; i < s.ntcCount && i < kMaxNtc; ++i)
      Serial.printf("T%u=%.1fC ", i + 1, s.ntcCelsius[i]);
  else
    Serial.printf("-- ");
  if (hasFields(s, kFieldMos))
    Serial.printf("  sw 0x%04X  mfg %04u-%02u-%02u", s.softwareVersion, s.mfgYear, s.mfgMonth,
                  s.mfgDay);
  Serial.printf("\n");

  if (hasFields(s, kFieldCells)) {
    const bool complete = (s.cellsRead == s.cellCount);
    Serial.printf("       cells %u/%u read%s, %u live, min %u max %u spread %u mV, sum %.2f V%s\n",
                  s.cellsRead, s.cellCount, complete ? "" : " (INCOMPLETE)", s.liveCellCount,
                  s.cellMinMv, s.cellMaxMv, s.cellSpreadMv, s.cellSumVolts,
                  complete ? "" : " of the read cells only");
    Serial.printf("      ");
    for (uint8_t i = 0; i < s.cellCount && i < kMaxCells; ++i) {
      if (i && (i % 9) == 0) Serial.printf("\n      ");
      if (s.cellsReadMask & (1u << i))
        Serial.printf(" %5u", s.cellMilliVolts[i]);
      else
        Serial.printf("     -");  // not read this poll
    }
    Serial.printf("\n");
  } else {
    Serial.printf("       cells --\n");
  }

  // Name what went missing, so a lossy link is visible rather than implied.
  uint16_t missing = 0;
  for (const FieldLabel &f : kFieldLabels)
    if (!hasFields(s, f.bit)) missing |= f.bit;
  if (missing) {
    Serial.printf("       MISSING:");
    for (const FieldLabel &f : kFieldLabels)
      if (missing & f.bit) Serial.printf(" %s", f.name);
    Serial.printf("\n");
  }
}

}  // namespace app
