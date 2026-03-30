#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>

/* ── User config ─────────────────────────────────────────────────────────── */
#define WIFI_SSID     "team5291"
#define WIFI_PASSWORD "12345678"
#define STM32_RX_PIN  D5                /* GPIO14 — wire to STM32 PA9 (TX)   */
#define STM32_TX_PIN  D6                /* GPIO12 — not connected, required   */
/* ────────────────────────────────────────────────────────────────────────── */

SoftwareSerial stm32Serial(STM32_RX_PIN, STM32_TX_PIN);
ESP8266WebServer server(80);

struct {
    String mode         = "---";
    int    path         = 0;
    int    intersection = 0;
    int    obstacle     = 9999;
    int    left         = 0;
    int    right        = 0;
} telem;

/* Parse one CSV line: MODE:AUTO,PATH:1,INTERSECTION:3,OBSTACLE:87,LEFT:412,RIGHT:389 */
void parseLine(const String &line) {
    int start = 0;
    while (start < (int)line.length()) {
        int comma = line.indexOf(',', start);
        if (comma == -1) comma = line.length();
        String field = line.substring(start, comma);
        int colon = field.indexOf(':');
        if (colon != -1) {
            String key = field.substring(0, colon);
            String val = field.substring(colon + 1);
            if      (key == "MODE")         telem.mode         = val;
            else if (key == "PATH")         telem.path         = val.toInt();
            else if (key == "INTERSECTION") telem.intersection = val.toInt();
            else if (key == "OBSTACLE")     telem.obstacle     = val.toInt();
            else if (key == "LEFT")         telem.left         = val.toInt();
            else if (key == "RIGHT")        telem.right        = val.toInt();
        }
        start = comma + 1;
    }
}

/* /data — returns latest telemetry as JSON */
void handleData() {
    String json = "{";
    json += "\"mode\":\""       + telem.mode                    + "\",";
    json += "\"path\":"         + String(telem.path)            + ",";
    json += "\"intersection\":" + String(telem.intersection)    + ",";
    json += "\"obstacle\":"     + String(telem.obstacle)        + ",";
    json += "\"left\":"         + String(telem.left)            + ",";
    json += "\"right\":"        + String(telem.right);
    json += "}";
    server.send(200, "application/json", json);
}

/* / — dashboard page */
void handleRoot() {
    String html =
        "<!DOCTYPE html><html><head>"
        "<meta charset='utf-8'><title>Robot Telemetry</title>"
        "<style>"
        "body{background:#111;color:#fff;font-family:monospace;padding:24px;margin:0}"
        "h1{color:#0f0;margin:0 0 20px}"
        ".grid{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;max-width:660px}"
        ".card{background:#1a1a1a;border:1px solid #2a2a2a;border-radius:8px;padding:16px}"
        ".lbl{color:#666;font-size:11px;text-transform:uppercase;letter-spacing:1px;margin-bottom:6px}"
        ".val{font-size:30px;font-weight:bold;color:#0f0}"
        ".val.stop{color:#f44336}.val.lost{color:#ff9800}.val.intr{color:#2196f3}"
        ".bar{height:6px;background:#222;border-radius:3px;margin-top:8px;overflow:hidden}"
        ".fill{height:100%;background:#0f0;border-radius:3px;transition:width .3s}"
        "</style></head><body>"
        "<h1>&#9679; Robot Telemetry</h1>"
        "<div class='grid'>"
        "<div class='card'><div class='lbl'>Mode</div><div class='val' id='mode'>---</div></div>"
        "<div class='card'><div class='lbl'>Path</div><div class='val' id='path'>-</div></div>"
        "<div class='card'><div class='lbl'>Intersections</div><div class='val' id='ix'>-</div></div>"
        "<div class='card'><div class='lbl'>Obstacle (mm)</div><div class='val' id='obs'>-</div></div>"
        "<div class='card'><div class='lbl'>Left Sensor</div><div class='val' id='lft'>-</div>"
        "<div class='bar'><div class='fill' id='lbar' style='width:0%'></div></div></div>"
        "<div class='card'><div class='lbl'>Right Sensor</div><div class='val' id='rgt'>-</div>"
        "<div class='bar'><div class='fill' id='rbar' style='width:0%'></div></div></div>"
        "</div>"
        "<script>"
        "function applyMode(el,v){"
        "el.className='val';"
        "if(v==='STOP')el.classList.add('stop');"
        "else if(v==='LOST')el.classList.add('lost');"
        "else if(v==='INTR')el.classList.add('intr');}"
        "async function tick(){"
        "try{"
        "const d=await(await fetch('/data')).json();"
        "const m=document.getElementById('mode');"
        "m.textContent=d.mode;applyMode(m,d.mode);"
        "document.getElementById('path').textContent=d.path;"
        "document.getElementById('ix').textContent=d.intersection;"
        "const obs=document.getElementById('obs');"
        "obs.textContent=d.obstacle;"
        "obs.className='val'+(d.obstacle<200?' stop':'');"
        "document.getElementById('lft').textContent=d.left;"
        "document.getElementById('rgt').textContent=d.right;"
        "document.getElementById('lbar').style.width=Math.min(d.left/4095*100,100)+'%';"
        "document.getElementById('rbar').style.width=Math.min(d.right/4095*100,100)+'%';"
        "}catch(e){}"
        "}"
        "setInterval(tick,1000);tick();"
        "</script></body></html>";

    server.send(200, "text/html", html);
}

static String rxLine;

void setup() {
    
    Serial.begin(115200);
    stm32Serial.begin(9600);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) delay(250);
    Serial.println(WiFi.localIP());
    server.on("/",     handleRoot);
    server.on("/data", handleData);
    server.begin();
}

void loop() {
    server.handleClient();
    while (stm32Serial.available()) {
        char c = (char)stm32Serial.read();
        if (c == '\n') {
            if (rxLine.length() > 0) {
                parseLine(rxLine);
                rxLine = "";
            }
        } else if (c != '\r') {
            rxLine += c;
        }
    }
}
