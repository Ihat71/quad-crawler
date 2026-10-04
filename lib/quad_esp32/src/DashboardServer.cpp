#include "DashboardServer.h"

#include <ESPmDNS.h>
#include <WiFi.h>

#include <string>

#include "robot/CommandParser.h"
#include "telemetry/JsonWriter.h"
#include "telemetry/TelemetryJson.h"

// dashboard/index.html, embedded at build time (board_build.embed_txtfiles).
extern const char kDashboardHtml[] asm("_binary_dashboard_index_html_start");

namespace qspider {

DashboardServer::DashboardServer(Robot& robot, EventLog& log, const RobotConfig& cfg, const char* firmware)
    : robot_(robot), log_(log), cfg_(cfg), firmware_(firmware), server_(cfg.network.httpPort) {}

void DashboardServer::begin() {
  if (!cfg_.network.enabled) {
    log_.log(LogLevel::Info, LogCategory::Network, "Dashboard disabled in config");
    return;
  }
  // Note: WiFi modem sleep must stay enabled while Bluetooth is active (ESP32 coexistence).
  WiFi.setHostname(cfg_.network.hostname);
  if (cfg_.network.staSsid && cfg_.network.staSsid[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg_.network.staSsid, cfg_.network.staPassword);
    state_ = NetState::Connecting;
    connectStartMs_ = millis();
    log_.log(LogLevel::Info, LogCategory::Network, "Joining WiFi '%s'", cfg_.network.staSsid);
  } else {
    startAccessPoint();
  }
}

void DashboardServer::startAccessPoint() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(cfg_.network.apSsid, cfg_.network.apPassword)) {
    log_.log(LogLevel::Error, LogCategory::Network, "Access point failed (password must be >= 8 chars)");
    state_ = NetState::Off;
    return;
  }
  snprintf(ip_, sizeof(ip_), "%s", WiFi.softAPIP().toString().c_str());
  state_ = NetState::AccessPoint;
  log_.log(LogLevel::Info, LogCategory::Network, "Access point '%s' up, dashboard at http://%s/", cfg_.network.apSsid,
           ip_);
  startHttp();
}

void DashboardServer::startHttp() {
  if (!httpStarted_) {
    server_.on("/", HTTP_GET, [this] { handleIndex(); });
    server_.on("/api/state", HTTP_GET, [this] { handleState(); });
    server_.on("/api/cmd", HTTP_POST, [this] { handleCommand(); });
    server_.onNotFound([this] { server_.send(404, "text/plain", "not found"); });
    server_.begin();
    httpStarted_ = true;
  }
  if (MDNS.begin(cfg_.network.hostname)) {
    MDNS.addService("http", "tcp", cfg_.network.httpPort);
    log_.log(LogLevel::Info, LogCategory::Network, "mDNS: http://%s.local/", cfg_.network.hostname);
  }
}

void DashboardServer::poll() {
  switch (state_) {
    case NetState::Connecting:
      if (WiFi.status() == WL_CONNECTED) {
        snprintf(ip_, sizeof(ip_), "%s", WiFi.localIP().toString().c_str());
        state_ = NetState::Station;
        log_.log(LogLevel::Info, LogCategory::Network, "WiFi connected, dashboard at http://%s/", ip_);
        startHttp();
      } else if (millis() - connectStartMs_ > cfg_.network.staConnectTimeoutMs) {
        log_.log(LogLevel::Warn, LogCategory::Network, "WiFi '%s' not reachable, starting access point",
                 cfg_.network.staSsid);
        startAccessPoint();
      }
      break;
    case NetState::Station:
      if (WiFi.status() != WL_CONNECTED) {
        log_.log(LogLevel::Warn, LogCategory::Network, "WiFi connection lost, reconnecting");
        WiFi.reconnect();
        state_ = NetState::Connecting;
        connectStartMs_ = millis();
      }
      break;
    default: break;
  }
  if (httpStarted_) server_.handleClient();
}

void DashboardServer::handleIndex() {
  server_.sendHeader("Cache-Control", "no-cache");
  server_.send_P(200, "text/html", kDashboardHtml);
}

void DashboardServer::handleState() {
  TelemetryCursor since;
  since.logSeq = static_cast<uint32_t>(server_.arg("log").toInt());
  since.moveSeq = static_cast<uint32_t>(server_.arg("move").toInt());
  since.buttonSeq = static_cast<uint32_t>(server_.arg("btn").toInt());

  PlatformInfo info;
  info.uptimeMs = millis();
  info.freeHeap = ESP.getFreeHeap();
  info.minFreeHeap = ESP.getMinFreeHeap();
  info.wifiRssi = state_ == NetState::Station ? WiFi.RSSI() : 0;
  info.ip = ip_;
  info.firmware = firmware_;

  std::string json;
  json.reserve(8192);
  writeTelemetryJson(json, cfg_, robot_.snapshot(), log_, since, info);
  server_.sendHeader("Cache-Control", "no-store");
  server_.send(200, "application/json", json.c_str());
}

void DashboardServer::handleCommand() {
  std::string out;
  JsonWriter w(out);
  Command c;
  char error[64] = {};
  const String line = server_.arg("line");
  w.beginObject();
  switch (parseCommand(line.c_str(), c, error, sizeof(error))) {
    case ParseResult::Ok:
      c.source = CommandSource::Dashboard;
      if (robot_.submit(c))
        w.boolean("ok", true);
      else
        w.boolean("ok", false).str("error", "command queue full");
      break;
    case ParseResult::Help: w.boolean("ok", true).str("text", commandHelp()); break;
    case ParseResult::Status: w.boolean("ok", true).str("text", "see the Status card"); break;
    case ParseResult::Empty: w.boolean("ok", false).str("error", "empty command"); break;
    case ParseResult::Error: w.boolean("ok", false).str("error", error); break;
  }
  w.endObject();
  server_.send(200, "application/json", out.c_str());
}

}  // namespace qspider
