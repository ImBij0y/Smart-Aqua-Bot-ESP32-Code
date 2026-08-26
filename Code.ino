/******************
   POND CLEANING ROBOT - WEB ONLY CONTROL
   ESP32 Dev Module
   
   Fixes:
   - HTML served from PROGMEM (no heap fragmentation)
   - Motor watchdog (auto-stop if no command received)
   - Non-blocking camera IP detection
   - Lightweight JSON responses
   - WiFi stability improvements
******************/

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include <LittleFS.h>

// ================== PINS ==================
#define IN1           13
#define IN2           14
#define IN3           16
#define IN4           17
#define RELAY_PIN     23
#define PAN_SERVO     26
#define TILT_SERVO    27
#define PH_PIN        34
#define TURB_PIN      35

// ================== MOTOR WATCHDOG ==================
// If no movement command received within this time, auto stop
// This is the KEY fix for "motor keeps running"
#define MOTOR_WATCHDOG_MS 300

// ================== OBJECTS ==================
WebServer server(80);
Servo panServo;
Servo tiltServo;

// ================== STATE ==================
int    panAngle        = 90;
int    tiltAngle       = 90;
float  phValue         = 0;
float  turbidityValue  = 0;
bool   feederState     = false;
String cameraIP        = "";

// Motor watchdog timer
unsigned long lastMotorCmdMs  = 0;
bool          motorRunning    = false;

unsigned long lastSensor      = 0;
unsigned long lastIPScan      = 0;

// ================== HTML in PROGMEM ==================
// Stored in flash, not RAM - fixes heap exhaustion/freezing
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport"
      content="width=device-width,initial-scale=1,
               user-scalable=no,maximum-scale=1">
<title>Pond Robot</title>
<script src="/chart.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0;
  -webkit-tap-highlight-color:transparent;
  -webkit-touch-callout:none;
  user-select:none;}
body{
  font-family:'Segoe UI',sans-serif;
  background:linear-gradient(135deg,#0f2027,#203a43,#2c5364);
  color:#fff;min-height:100vh;
}
header{
  padding:18px;text-align:center;font-size:24px;
  font-weight:700;background:rgba(0,0,0,0.3);
  border-bottom:1px solid rgba(255,255,255,0.08);
}
.grid{
  display:flex;flex-wrap:wrap;gap:16px;
  padding:16px;justify-content:center;
}
.card{
  background:rgba(255,255,255,0.07);
  backdrop-filter:blur(14px);
  border:1px solid rgba(255,255,255,0.1);
  border-radius:16px;padding:16px;
  box-shadow:0 8px 24px rgba(0,0,0,0.35);
}
.card h2{
  font-size:12px;font-weight:600;
  text-transform:uppercase;letter-spacing:1.2px;
  color:rgba(255,255,255,0.5);margin-bottom:10px;
}
.c-cam{width:100%;max-width:720px;}
.c-cam img{width:100%;border-radius:10px;
  background:#000;min-height:180px;display:block;}
.no-cam{color:rgba(255,255,255,0.3);font-size:13px;
  padding:50px 0;text-align:center;}
.c-sensor{width:340px;flex:1 1 300px;max-width:420px;}
.sv{font-size:38px;font-weight:700;margin-bottom:8px;}
.ph-g{color:#00ff88;}.ph-w{color:#ffcc00;}.ph-b{color:#ff4d4d;}
.tu-g{color:#00cfff;}.tu-w{color:#ffaa00;}.tu-b{color:#ff4d4d;}
canvas{width:100%!important;height:130px!important;
  border-radius:8px;background:rgba(255,255,255,0.05);}
.c-feed{width:220px;text-align:center;}
.fs{font-size:30px;font-weight:700;margin:8px 0 14px;}
.on{color:#00ff88;}.off{color:#ff4d4d;}
.bf{
  padding:12px 30px;border-radius:30px;border:none;
  font-size:15px;font-weight:700;cursor:pointer;
  transition:all .2s;touch-action:manipulation;
}
.bf-on{background:linear-gradient(135deg,#00c853,#69f0ae);color:#003300;}
.bf-off{background:linear-gradient(135deg,#d50000,#ff5252);color:#fff;}
.bf:active{transform:scale(.94);}
.c-move{width:250px;text-align:center;}
.dpad{
  display:grid;
  grid-template-columns:repeat(3,68px);
  grid-template-rows:repeat(3,68px);
  gap:6px;justify-content:center;margin-top:6px;
}
.dpad button{
  width:68px;height:68px;border-radius:14px;border:none;
  background:rgba(255,255,255,0.12);color:#fff;
  font-size:24px;cursor:pointer;
  transition:background .12s,transform .1s;
  touch-action:manipulation;
}
.dpad button:active,
.dpad button.pressed{
  background:rgba(0,200,255,0.4);
  transform:scale(.92);
}
.dpad .e{background:transparent;pointer-events:none;}
.dpad .stp{
  background:rgba(255,80,80,0.25);
  font-size:13px;font-weight:700;
}
.c-srv{width:280px;}
.sr{margin:10px 0;}
.sr label{
  font-size:12px;color:rgba(255,255,255,0.6);
  display:flex;justify-content:space-between;
  margin-bottom:4px;
}
input[type=range]{width:100%;accent-color:#00cfff;cursor:pointer;}
.bc{
  margin-top:12px;padding:9px 22px;border-radius:20px;
  border:1px solid rgba(255,255,255,0.2);
  background:rgba(255,255,255,0.1);color:#fff;
  font-size:13px;cursor:pointer;touch-action:manipulation;
}
.bc:active{background:rgba(255,255,255,0.25);}
.status-bar{
  text-align:center;padding:8px;
  font-size:11px;color:rgba(255,255,255,0.35);
}
.conn-dot{
  display:inline-block;width:8px;height:8px;
  border-radius:50%;background:#ff4d4d;
  margin-right:6px;vertical-align:middle;
}
.conn-dot.ok{background:#00ff88;}
@media(max-width:600px){
  .c-sensor,.c-move,.c-srv,.c-feed{width:100%;}
  .dpad{
    grid-template-columns:repeat(3,72px);
    grid-template-rows:repeat(3,72px);
  }
}
</style>
</head>
<body>

<header>&#x1F6A4; Pond Cleaning Robot</header>

<div class="grid">

  <!-- Camera -->
  <div class="card c-cam">
    <h2>&#x1F4F7; Live Camera</h2>
    <div id="camBox">
      <div class="no-cam" id="camMsg">
        Waiting for camera connection...
      </div>
    </div>
  </div>

  <!-- pH -->
  <div class="card c-sensor">
    <h2>&#x2697;&#xFE0F; pH Level</h2>
    <div class="sv ph-g" id="phV">--</div>
    <canvas id="phC"></canvas>
  </div>

  <!-- Turbidity -->
  <div class="card c-sensor">
    <h2>&#x1F4A7; Turbidity (NTU)</h2>
    <div class="sv tu-g" id="tuV">--</div>
    <canvas id="tuC"></canvas>
  </div>

  <!-- Feeder -->
  <div class="card c-feed">
    <h2>&#x1F41F; Fish Feeder</h2>
    <div class="fs off" id="fSt">OFF</div>
    <button class="bf bf-on" id="fBtn"
            onclick="toggleFeed()">TURN ON</button>
  </div>

  <!-- Movement D-Pad -->
  <div class="card c-move">
    <h2>&#x1F579;&#xFE0F; Movement Control</h2>
    <p style="font-size:11px;color:rgba(255,255,255,0.35);
       margin-bottom:6px;">Hold to move &middot; Release to stop</p>
    <div class="dpad">
      <div class="e"></div>
      <button id="bF" data-cmd="F">&#x25B2;</button>
      <div class="e"></div>

      <button id="bL" data-cmd="L">&#x25C4;</button>
      <button class="stp" id="bS" data-cmd="S">STOP</button>
      <button id="bR" data-cmd="R">&#x25BA;</button>

      <div class="e"></div>
      <button id="bB" data-cmd="B">&#x25BC;</button>
      <div class="e"></div>
    </div>
  </div>

  <!-- Servo -->
  <div class="card c-srv">
    <h2>&#x1F3A5; Camera Servo</h2>
    <div class="sr">
      <label><span>Pan (Left / Right)</span>
        <span id="pL">90&deg;</span></label>
      <input type="range" min="0" max="180" value="90"
             id="pS" oninput="sv('PAN',this.value)">
    </div>
    <div class="sr">
      <label><span>Tilt (Up / Down)</span>
        <span id="tL">90&deg;</span></label>
      <input type="range" min="0" max="180" value="90"
             id="tS" oninput="sv('TILT',this.value)">
    </div>
    <button class="bc" onclick="ctr()">&#x2299; Center</button>
  </div>

</div>

<div class="status-bar">
  <span class="conn-dot" id="cDot"></span>
  <span id="sBar">Connecting...</span>
</div>

<script>
// ── Charts ──────────────────────────────────────
let phCh, tuCh;

function mkC(id,lbl,col){
  return new Chart(document.getElementById(id),{
    type:'line',
    data:{labels:[],datasets:[{
      label:lbl,data:[],
      borderColor:col,backgroundColor:col+'22',
      borderWidth:2,pointRadius:0,
      tension:0.4,fill:true
    }]},
    options:{
      animation:false,responsive:true,
      maintainAspectRatio:false,
      plugins:{legend:{display:false}},
      scales:{
        x:{ticks:{color:'rgba(255,255,255,0.4)',
           maxTicksLimit:5},
           grid:{color:'rgba(255,255,255,0.06)'}},
        y:{ticks:{color:'rgba(255,255,255,0.4)'},
           grid:{color:'rgba(255,255,255,0.06)'}}
      }
    }
  });
}

window.addEventListener('load',()=>{
  if(typeof Chart!=='undefined'){
    phCh=mkC('phC','pH','#00ff88');
    tuCh=mkC('tuC','Turbidity','#00cfff');
  }
  poll();
  setInterval(poll,1500);
  setupDpad();
});

// ── Polling ─────────────────────────────────────
// Increased to 1500ms to reduce ESP load
let pollFails = 0;

function poll(){
  fetch('/data',{signal:AbortSignal.timeout(2000)})
    .then(r=>r.json())
    .then(d=>{
      pollFails = 0;
      setConn(true);

      let pe=document.getElementById('phV');
      pe.textContent=d.ph.toFixed(2);
      pe.className='sv '+(d.ph>=6.5&&d.ph<=8.5?'ph-g':
                  d.ph>=5.5&&d.ph<=9.5?'ph-w':'ph-b');
      push(phCh,d.ph);

      let te=document.getElementById('tuV');
      te.textContent=d.turbidity.toFixed(1);
      te.className='sv '+(d.turbidity<30?'tu-g':
                  d.turbidity<70?'tu-w':'tu-b');
      push(tuCh,d.turbidity);

      updFeed(d.feeder);

      document.getElementById('pS').value=d.pan;
      document.getElementById('tS').value=d.tilt;
      document.getElementById('pL').textContent=d.pan+'°';
      document.getElementById('tL').textContent=d.tilt+'°';

      if(d.cam&&d.cam.length>6){
        let url='http://'+d.cam+':81/stream';
        let img=document.querySelector('.c-cam img');
        if(!img){
          document.getElementById('camBox').innerHTML=
            '<img src="'+url+'" alt="Feed">';
          document.getElementById('camMsg').style.display='none';
        }
      }
    })
    .catch(()=>{
      pollFails++;
      if(pollFails>=3) setConn(false);
    });
}

function setConn(ok){
  document.getElementById('cDot').className='conn-dot'+(ok?' ok':'');
  document.getElementById('sBar').textContent=
    ok?'Connected to PondRobot AP':'Connection lost - retrying...';
}

// ── Chart Push ──────────────────────────────────
function push(ch,v){
  if(!ch)return;
  let t=new Date().toLocaleTimeString();
  if(ch.data.labels.length>25){
    ch.data.labels.shift();
    ch.data.datasets[0].data.shift();
  }
  ch.data.labels.push(t);
  ch.data.datasets[0].data.push(v);
  ch.update('none');
}

// ── Feeder ──────────────────────────────────────
let feedOn=false;
function toggleFeed(){
  feedOn=!feedOn;
  send(feedOn?'FEED_ON':'FEED_OFF');
}
function updFeed(s){
  let e=document.getElementById('fSt');
  let b=document.getElementById('fBtn');
  if(s==='ON'){
    e.textContent='ON';e.className='fs on';
    b.textContent='TURN OFF';b.className='bf bf-off';
    feedOn=true;
  }else{
    e.textContent='OFF';e.className='fs off';
    b.textContent='TURN ON';b.className='bf bf-on';
    feedOn=false;
  }
}

// ── Servo ───────────────────────────────────────
let svT=null;
function sv(ax,v){
  document.getElementById(ax==='PAN'?'pL':'tL')
    .textContent=v+'°';
  clearTimeout(svT);
  svT=setTimeout(()=>send(ax+':'+v),80);
}
function ctr(){
  document.getElementById('pS').value=90;
  document.getElementById('tS').value=90;
  document.getElementById('pL').textContent='90°';
  document.getElementById('tL').textContent='90°';
  send('CENTER');
}

// ══════════════════════════════════════════════════
//  D-PAD - HOLD = MOVE, RELEASE = STOP
//  Key fix: repeated sends while held + immediate stop
// ══════════════════════════════════════════════════
let holdInterval = null;
let activeCmd    = null;

function startMove(cmd){
  if(activeCmd === cmd) return;   // already running this command
  stopMove();                      // stop previous if any
  activeCmd = cmd;
  send(cmd);                       // send immediately

  // Re-send every 200ms while held (watchdog keepalive)
  // ESP auto-stops if it doesn't receive within 300ms
  holdInterval = setInterval(()=> send(cmd), 200);
}

function stopMove(){
  if(holdInterval){
    clearInterval(holdInterval);
    holdInterval = null;
  }
  if(activeCmd && activeCmd !== 'S'){
    send('S');                     // explicit stop
  }
  activeCmd = null;
}

function setupDpad(){
  const btns = document.querySelectorAll('.dpad button');

  btns.forEach(btn=>{
    const cmd = btn.dataset.cmd;
    if(!cmd) return;

    if(cmd==='S'){
      btn.addEventListener('click',()=>{
        stopMove();
        send('S');
      });
      return;
    }

    // Touch
    btn.addEventListener('touchstart',function(e){
      e.preventDefault();
      this.classList.add('pressed');
      startMove(cmd);
    },{passive:false});

    btn.addEventListener('touchend',function(e){
      e.preventDefault();
      this.classList.remove('pressed');
      stopMove();
    },{passive:false});

    btn.addEventListener('touchcancel',function(){
      this.classList.remove('pressed');
      stopMove();
    });

    // Mouse
    btn.addEventListener('mousedown',function(e){
      e.preventDefault();
      this.classList.add('pressed');
      startMove(cmd);
    });
    btn.addEventListener('mouseup',function(){
      this.classList.remove('pressed');
      stopMove();
    });
    btn.addEventListener('mouseleave',function(){
      if(this.classList.contains('pressed')){
        this.classList.remove('pressed');
        stopMove();
      }
    });
  });

  // Global safety nets
  document.addEventListener('touchend',()=>{
    let anyPressed=document.querySelectorAll('.dpad .pressed');
    if(anyPressed.length>0){
      anyPressed.forEach(b=>b.classList.remove('pressed'));
      stopMove();
    }
  });

  document.addEventListener('mouseup',()=>{
    let anyPressed=document.querySelectorAll('.dpad .pressed');
    if(anyPressed.length>0){
      anyPressed.forEach(b=>b.classList.remove('pressed'));
      stopMove();
    }
  });

  // If page loses focus (tab switch, phone call etc), stop motors
  document.addEventListener('visibilitychange',()=>{
    if(document.hidden) stopMove();
  });

  window.addEventListener('blur',()=> stopMove());
}

// ── Send Command ────────────────────────────────
// Uses AbortSignal timeout so stuck requests dont pile up
function send(cmd){
  fetch('/control',{
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'cmd='+encodeURIComponent(cmd),
    signal:AbortSignal.timeout(1000)   // drop if ESP takes > 1s
  }).catch(()=>{});
}
</script>
</body>
</html>
)rawliteral";

// ================== MOTORS ==================
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  motorRunning = false;
}
void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  motorRunning     = true;
  lastMotorCmdMs   = millis();
}
void moveBackward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  motorRunning     = true;
  lastMotorCmdMs   = millis();
}
void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  motorRunning     = true;
  lastMotorCmdMs   = millis();
}
void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  motorRunning     = true;
  lastMotorCmdMs   = millis();
}

// ================== MOTOR WATCHDOG ==================
// Auto-stops motors if no command received within MOTOR_WATCHDOG_MS
// This is the core fix for "motor keeps running after finger lift"
void checkMotorWatchdog() {
  if (motorRunning &&
      (millis() - lastMotorCmdMs > MOTOR_WATCHDOG_MS)) {
    Serial.println("[WATCHDOG] Motor timeout - auto stop");
    stopMotors();
  }
}

// ================== SENSORS ==================
float readPH() {
  int   raw     = analogRead(PH_PIN);
  float voltage = (raw / 4095.0) * 3.3;
  float ph      = 7.0 + ((2.5 - voltage) * 3.5);
  if (ph < 0)  ph = 0;
  if (ph > 14) ph = 14;
  return ph;
}

float readTurbidity() {
  int   raw     = analogRead(TURB_PIN);
  float voltage = (raw / 4095.0) * 3.3;
  float ntu     = (3.3 - voltage) * 1000.0 / 3.3;
  if (ntu < 0)  ntu = 0;
  return ntu;
}

void readSensors() {
  phValue        = readPH();
  turbidityValue = readTurbidity();
}

// ================== CAMERA DETECT ==================
void detectCameraIP() {
  wifi_sta_list_t          wl;
  tcpip_adapter_sta_list_t al;
  memset(&wl, 0, sizeof(wl));
  memset(&al, 0, sizeof(al));
  esp_wifi_ap_get_sta_list(&wl);
  tcpip_adapter_get_sta_list(&wl, &al);
  for (int i = 0; i < al.num; i++) {
    char ip[16];
    sprintf(ip, IPSTR, IP2STR(&al.sta[i].ip));
    cameraIP = String(ip);
  }
}

// ================== /data ==================
void handleData() {
  String j = "{\"ph\":"        + String(phValue, 2)        +
             ",\"turbidity\":" + String(turbidityValue, 2) +
             ",\"feeder\":\""  + (feederState ? "ON":"OFF") + "\"" +
             ",\"pan\":"       + String(panAngle)           +
             ",\"tilt\":"      + String(tiltAngle)          +
             ",\"cam\":\""     + cameraIP                   + "\"}";
  server.send(200, "application/json", j);
}

// ================== /control ==================
void handleControl() {
  if (!server.hasArg("cmd")) {
    server.send(400, "text/plain", "no cmd");
    return;
  }
  String cmd = server.arg("cmd");

  if      (cmd == "F") moveForward();
  else if (cmd == "B") moveBackward();
  else if (cmd == "L") turnLeft();
  else if (cmd == "R") turnRight();
  else if (cmd == "S") stopMotors();
  else if (cmd == "FEED_ON")  { digitalWrite(RELAY_PIN, HIGH); feederState = true;  }
  else if (cmd == "FEED_OFF") { digitalWrite(RELAY_PIN, LOW);  feederState = false; }
  else if (cmd == "CENTER") {
    panAngle  = 90; tiltAngle = 90;
    panServo.write(90); tiltServo.write(90);
  }
  else if (cmd.startsWith("PAN:")) {
    panAngle = constrain(cmd.substring(4).toInt(), 0, 180);
    panServo.write(panAngle);
  }
  else if (cmd.startsWith("TILT:")) {
    tiltAngle = constrain(cmd.substring(5).toInt(), 0, 180);
    tiltServo.write(tiltAngle);
  }

  server.send(200, "text/plain", "OK");
}

// ================== /chart.min.js ==================
void handleChartJS() {
  File f = LittleFS.open("/chart.min.js", "r");
  if (!f) {
    server.send(404, "text/plain", "chart.min.js missing");
    return;
  }
  server.sendHeader("Cache-Control", "public, max-age=31536000");
  server.streamFile(f, "application/javascript");
  f.close();
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  stopMotors();
  digitalWrite(RELAY_PIN, LOW);

  analogReadResolution(12);

  panServo.attach(PAN_SERVO);
  tiltServo.attach(TILT_SERVO);
  panServo.write(90);
  tiltServo.write(90);

  // Increase TX power for better WiFi signal
  WiFi.softAP("PondRobot", "12345678");
  esp_wifi_set_max_tx_power(78);   // 78 = ~19.5 dBm (max)

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  if (!LittleFS.begin(true))
    Serial.println("LittleFS FAIL");
  else
    Serial.println("LittleFS OK");

  // Serve HTML from PROGMEM - no heap string building
  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html", INDEX_HTML);
  });
  server.on("/data",         HTTP_GET,  handleData);
  server.on("/control",      HTTP_POST, handleControl);
  server.on("/chart.min.js", HTTP_GET,  handleChartJS);

  server.begin();
  Serial.println("Server ready");

  readSensors();
}

// ================== LOOP ==================
void loop() {
  // Motor watchdog - checked every loop iteration (critical)
  checkMotorWatchdog();

  server.handleClient();

  if (millis() - lastSensor > 1000) {
    lastSensor = millis();
    readSensors();
  }

  // Camera IP scan reduced to every 10s (was 5s)
  // Reduces WiFi stack interruption
  if (millis() - lastIPScan > 10000) {
    lastIPScan = millis();
    detectCameraIP();
  }
}
