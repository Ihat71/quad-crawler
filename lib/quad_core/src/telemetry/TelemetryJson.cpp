#include "telemetry/TelemetryJson.h"

#include "motion/Animation.h"
#include "telemetry/JsonWriter.h"

namespace qspider {

namespace {

void faults(JsonWriter& w, const char* key, uint32_t mask) {
  w.beginArray(key);
  for (int i = 0; i < kFaultCount; ++i)
    if (mask & (1u << i)) w.str(nullptr, toString(static_cast<Fault>(i)));
  w.endArray();
}

void vec(JsonWriter& w, const char* key, const Vec3& v) {
  w.beginArray(key).real(nullptr, v.x, 1).real(nullptr, v.y, 1).real(nullptr, v.z, 1).endArray();
}

template <typename Entry, typename Ring, typename Fn>
void forEachSince(const Ring& ring, uint32_t after, size_t max, Fn fn) {
  Entry chunk[8];
  size_t total = 0;
  while (total < max) {
    const size_t want = max - total < 8 ? max - total : 8;
    const size_t n = ring.since(after, chunk, want);
    if (n == 0) break;
    for (size_t i = 0; i < n; ++i) fn(chunk[i]);
    after = chunk[n - 1].seq;
    total += n;
  }
}

}  // namespace

void writeTelemetryJson(std::string& out, const RobotConfig& cfg, const RobotSnapshot& s, const EventLog& log,
                        const TelemetryCursor& since, const PlatformInfo& platform, size_t maxEntries) {
  JsonWriter w(out);
  w.beginObject();
  w.str("name", cfg.robotName);
  w.str("firmware", platform.firmware);
  w.num("t", s.timeMs);
  w.num("uptime", platform.uptimeMs);
  w.num("pollMs", cfg.timing.telemetryPeriodMs);

  w.beginObject("status")
      .str("lifecycle", toString(s.lifecycle))
      .str("mode", toString(s.mode))
      .str("posture", toString(s.posture))
      .str("motion", toString(s.motion))
      .str("gait", s.gaitName)
      .num("gaitIndex", s.gaitIndex)
      .str("dance", s.danceName)
      .str("animation", s.animationName)
      .boolean("loop", s.animationLoop)
      .boolean("outputs", s.outputsEnabled)
      .str("estopReason", s.estopReason)
      .endObject();

  w.beginObject("walk")
      .real("vx", s.walk.vx)
      .real("vy", s.walk.vy)
      .real("turn", s.walk.turn)
      .real("speed", s.walk.speed)
      .endObject();

  w.beginObject("health");
  faults(w, "faults", s.activeFaults);
  faults(w, "seenFaults", s.seenFaults);
  w.beginObject("battery")
      .boolean("enabled", s.batteryEnabled)
      .str("status", toString(s.batteryStatus))
      .real("v", s.batteryVoltage)
      .real("ripple", s.batteryRipple)
      .num("pct", s.batteryPercent)
      .endObject();
  w.boolean("controller", s.controllerConnected)
      .num("ticks", s.ticks)
      .num("overruns", s.overruns)
      .real("periodMs", s.periodAvgMs)
      .real("execUs", s.execAvgUs, 0)
      .num("execMaxUs", s.execMaxUs)
      .num("i2cErrors", s.i2cErrors)
      .num("clamps", s.clampEvents)
      .num("ikFailures", s.ikFailures)
      .num("warnings", s.warnings)
      .num("errors", s.errors)
      .num("heap", platform.freeHeap)
      .num("minHeap", platform.minFreeHeap)
      .num("rssi", platform.wifiRssi)
      .str("ip", platform.ip);
  w.endObject();

  w.beginArray("servos");
  for (int i = 0; i < kServoCount; ++i) {
    const int leg = legOfServo(i), joint = jointOfServo(i);
    w.beginObject()
        .str("leg", legName(leg))
        .str("joint", jointName(joint))
        .num("ch", cfg.legs[leg].servo[joint].channel)
        .real("deg", s.servoDeg[i], 1)
        .real("target", s.servoTarget[i], 1)
        .real("trim", s.trim[i], 1)
        .endObject();
  }
  w.endArray();

  w.beginArray("feet");
  for (const Vec3& f : s.feet) vec(w, nullptr, f);
  w.endArray();
  w.beginArray("gaits");
  for (int i = 0; i < cfg.motion.gaitCount; ++i) w.str(nullptr, cfg.motion.gaits[i].name);
  w.endArray();
  w.beginArray("dances");
  for (int i = 0; i < animations::danceCount(); ++i) w.str(nullptr, animations::dance(i).name);
  w.endArray();
  w.beginArray("mounts");
  for (const LegConfig& l : cfg.legs) vec(w, nullptr, l.mountPosition);
  w.endArray();

  w.beginObject("cal").num("selected", s.calSelected).boolean("dirty", s.calDirty).endObject();
  w.beginObject("ik").num("leg", s.ikLeg);
  vec(w, "offset", s.ikOffset);
  w.endObject();

  // Incremental histories, copied in small chunks to keep stack usage low.
  w.beginArray("logs");
  forEachSince<LogEntry>(log.events(), since.logSeq, maxEntries, [&](const LogEntry& e) {
    w.beginObject()
        .num("seq", e.seq)
        .num("t", e.timeMs)
        .str("lvl", toString(e.level))
        .str("cat", toString(e.category))
        .str("msg", e.text)
        .endObject();
  });
  w.endArray();

  w.beginArray("moves");
  forEachSince<MovementEntry>(log.movements(), since.moveSeq, maxEntries, [&](const MovementEntry& e) {
    w.beginObject().num("seq", e.seq).num("t", e.timeMs).str("mode", toString(e.mode)).str("msg", e.text).endObject();
  });
  w.endArray();

  w.beginArray("buttons");
  forEachSince<ButtonEntry>(log.buttons(), since.buttonSeq, maxEntries, [&](const ButtonEntry& e) {
    w.beginObject()
        .num("seq", e.seq)
        .num("t", e.timeMs)
        .str("btn", toString(e.button))
        .str("act", toString(e.action))
        .str("mode", toString(e.mode))
        .endObject();
  });
  w.endArray();

  w.endObject();
}

}  // namespace qspider
