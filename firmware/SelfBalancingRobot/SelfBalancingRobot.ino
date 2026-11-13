#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define USE_LEDC_V3
#endif

#define I2C_SDA    21
#define I2C_SCL    22
#define TOF_SDA    16
#define TOF_SCL    17
#define PIN_PWMA   32
#define PIN_AIN1   33
#define PIN_AIN2   25
#define PIN_PWMB   14
#define PIN_BIN1   27
#define PIN_BIN2   26
#define PIN_IR_FRONT 19
#define PIN_IR_BACK  18

// ALL FLOAT - 2 decimal
volatile float Kp = 9.74;          
volatile float Kd = 1.32;           
volatile float Ki = 164.28;
volatile float setPoint = -4.71;
volatile float trimAngle = 0.00;    
volatile float maxSpeed = 200.00;
volatile float motorDeadband = 44.00;   
volatile float accelStep = 20.00;

#define COMP_ALPHA        0.96
#define FALL_ANGLE        70.0
#define MAX_SPEED_LIMIT   255
#define LOOP_FREQ_HZ      200
#define LOOP_TIME_MS      (1000 / LOOP_FREQ_HZ)

#define ATTACK_DISTANCE   300
#define ATTACK_ANGLE      3.5
#define SEARCH_SPEED      30

Adafruit_MPU6050 mpu;
Adafruit_VL53L0X tof;
volatile int currentDistance = 0;
volatile bool isEscaping = false;
volatile bool isAttacking = false;
volatile unsigned long escapeStartTime = 0;
volatile float escapeDirection = 0.0;
volatile float currentPitch = 0.0;
volatile float gyroRate = 0.0;
volatile int currentSpeedLeft = 0;
volatile int currentSpeedRight = 0;
volatile int targetSpeedLeft = 0;
volatile int targetSpeedRight = 0;
SemaphoreHandle_t i2cMutex;
WebServer server(80);
Preferences prefs;

// ── HTML Dashboard - BUTTON UPDATE, NO AUTO, ALL %.2f ──
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SUMO Balance - Manual Update %.2f</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Outfit:wght@400;600;700&display=swap');
*{box-sizing:border-box;font-family:'Outfit',system-ui,sans-serif;-webkit-tap-highlight-color:transparent}
body{margin:0;background:linear-gradient(135deg,#05070a 0%,#0f172a 100%);color:#f8fafc;display:flex;justify-content:center;min-height:100vh}
.wrap{width:100%;max-width:760px;padding:8px;display:flex;flex-direction:column;gap:10px}
.card{background:rgba(30,41,59,0.4);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);border:1px solid rgba(255,255,255,0.08);border-radius:16px;padding:12px;box-shadow:0 8px 32px rgba(0,0,0,0.3)}
h1{font-size:18px;font-weight:700;margin:0 0 4px;background:linear-gradient(90deg,#60a5fa,#a78bfa);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.sub{color:#94a3b8;font-size:12px;margin-bottom:10px;display:flex;align-items:center;flex-wrap:wrap;gap:6px}
.telemetry{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.metric{background:rgba(15,23,42,0.6);border:1px solid rgba(255,255,255,0.05);border-radius:12px;padding:10px}
.metric .label{font-size:11px;color:#94a3b8;font-weight:600;letter-spacing:0.5px;text-transform:uppercase}
.metric .value{font-size:24px;font-weight:700;margin-top:2px}
.metric .unit{font-size:12px;color:#64748b;margin-left:4px}
.pitch-bar{height:12px;background:rgba(0,0,0,0.3);border-radius:8px;overflow:hidden;margin-top:8px;border:1px inset rgba(255,255,255,0.05);box-shadow:inset 0 2px 4px rgba(0,0,0,0.2)}
.pitch-fill{height:100%;width:50%;background:linear-gradient(90deg,#34d399,#3b82f6);transition:width 0.15s ease-out,background 0.3s;box-shadow:0 0 10px rgba(59,130,246,0.5)}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
@media(max-width:640px){.grid{grid-template-columns:1fr}}
.ctrl{background:rgba(15,23,42,0.4);border:1px solid rgba(255,255,255,0.05);border-radius:12px;padding:10px;transition:0.2s}
.ctrl:focus-within{border-color:rgba(96,165,250,0.5);background:rgba(30,41,59,0.6)}
.ctrl label{display:flex;justify-content:space-between;align-items:center;font-size:12px;font-weight:600;margin-bottom:6px;color:#cbd5e1}
.ctrl label b{color:#38bdf8;font-variant-numeric:tabular-nums;background:rgba(14,165,233,0.1);padding:2px 6px;border-radius:6px;font-size:13px}
.ctrl input[type=range]{width:100%;height:6px;background:rgba(255,255,255,0.1);border-radius:4px;outline:none;appearance:none;margin:8px 0}
.ctrl input[type=range]::-webkit-slider-thumb{appearance:none;width:20px;height:20px;border-radius:50%;background:#60a5fa;cursor:pointer;box-shadow:0 0 10px rgba(96,165,250,0.4);transition:0.2s}
.ctrl input[type=range]::-webkit-slider-thumb:active{transform:scale(1.15)}
.ctrl input[type=number]{width:100%;background:rgba(0,0,0,0.3);border:1px solid rgba(255,255,255,0.1);color:#f8fafc;border-radius:8px;padding:8px 10px;font-size:14px;font-weight:600;text-align:right;margin-top:4px;transition:0.2s;outline:none}
.ctrl input[type=number]:focus{border-color:#60a5fa;box-shadow:0 0 0 3px rgba(96,165,250,0.2)}
.row{display:flex;gap:8px;margin-top:10px;flex-wrap:wrap}
button{flex:1;min-width:100px;min-height:38px;background:rgba(51,65,85,0.6);color:#f8fafc;border:1px solid rgba(255,255,255,0.1);border-radius:10px;padding:0 10px;font-size:14px;font-weight:600;cursor:pointer;transition:all 0.2s cubic-bezier(0.4,0,0.2,1);display:flex;align-items:center;justify-content:center;gap:6px}
button:active{transform:scale(0.96)}
button:hover{background:rgba(71,85,105,0.8)}
button.primary{background:linear-gradient(135deg,#2563eb,#4f46e5);border:none;box-shadow:0 4px 15px rgba(37,99,235,0.3)}
button.primary:hover{box-shadow:0 6px 20px rgba(37,99,235,0.4)}
button.success{background:linear-gradient(135deg,#059669,#10b981);border:none;box-shadow:0 4px 15px rgba(16,185,129,0.2)}
button.danger{background:linear-gradient(135deg,#dc2626,#ef4444);border:none;box-shadow:0 4px 15px rgba(220,38,38,0.2)}
button.small{min-height:30px;padding:0 8px;font-size:12px;flex:0 0 auto;border-radius:8px}
.log{font-family:'Consolas',monospace;font-size:11px;background:rgba(0,0,0,0.4);border-radius:10px;padding:10px;height:100px;overflow:auto;color:#a3a8b4;border:1px inset rgba(255,255,255,0.05);line-height:1.4}
.badge{font-size:11px;font-weight:600;padding:2px 8px;border-radius:99px;background:rgba(255,255,255,0.1);border:1px solid rgba(255,255,255,0.05)}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;background:#22c55e;box-shadow:0 0 10px #22c55e}
.dirty{border-color:#f59e0b !important;box-shadow:0 0 0 2px rgba(245,158,11,0.3)}
.pending{color:#fcd34d !important}
::-webkit-scrollbar{width:6px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:rgba(255,255,255,0.1);border-radius:3px}
::-webkit-scrollbar-thumb:hover{background:rgba(255,255,255,0.2)}
</style>
</head>
<body>
<div class="wrap">
  <div class="card">
    <h1>🤖 SUMO Balancing - Manual Update %.2f</h1>
    <div class="sub"><span class="dot"></span><span id="conn">Connecting...</span> • AP: <b>SUMO_BALANCE</b> OPEN • <span class="badge">192.168.4.1</span> • <span id="statusTxt" class="badge">IDLE</span></div>
    <div class="telemetry">
      <div class="metric"><div class="label">Pitch Angle</div><div class="value"><span id="pitch">0.00</span><span class="unit">°</span></div><div class="pitch-bar"><div id="pitchFill" class="pitch-fill"></div></div></div>
      <div class="metric"><div class="label">IR Sensor</div><div class="value" style="font-size:18px"><span id="irStateF" class="badge">F: SAFE</span> <span id="irStateB" class="badge">B: SAFE</span></div><div class="label" style="margin-top:6px">Raw: <span id="irRaw">0 / 0</span></div></div>
      <div class="metric"><div class="label">Motor Power</div><div class="value"><span id="motorL">0</span>/<span id="motorR">0</span></div><div class="label" style="margin-top:6px">T: <span id="tL">0</span>/<span id="tR">0</span> | G: <span id="gyro">0</span>°/s</div></div>
      <div class="metric"><div class="label">ToF Distance</div><div class="value"><span id="tofDist">0</span><span class="unit">mm</span></div><div class="label" style="margin-top:6px">Status: <span id="tofStatus">OK</span></div></div>
    </div>
    <div class="row">
      <button class="danger" onclick="sendCmd('stop=1')">⛔ STOP</button>
      <button onclick="resetIntegral()">Reset Integral</button>
      <button onclick="fetchStatus()">↻ Refresh Status</button>
    </div>
  </div>

  <div class="card">
    <h1>PID Tuning - All %.2f <span id="pidDirty" style="font-size:11px;color:#fbbf24;display:none">● UNSAVED</span></h1>
    <div class="grid">
      <div class="ctrl" id="c_kp"><label>Kp 0-50 <b id="vKp">20.00</b></label><input type="range" id="kp_r" min="0" max="50" step="0.01" oninput="onInput('kp',this.value)"><input type="number" id="kp" step="0.01" min="0" max="50" oninput="onInput('kp',this.value)"></div>
      <div class="ctrl" id="c_ki"><label>Ki 0-255 <b id="vKi">30.00</b></label><input type="range" id="ki_r" min="0" max="255" step="0.01" oninput="onInput('ki',this.value)"><input type="number" id="ki" step="0.01" min="0" max="255" oninput="onInput('ki',this.value)"></div>
      <div class="ctrl" id="c_kd"><label>Kd 0-10 <b id="vKd">1.00</b></label><input type="range" id="kd_r" min="0" max="10" step="0.01" oninput="onInput('kd',this.value)"><input type="number" id="kd" step="0.01" min="0" max="10" oninput="onInput('kd',this.value)"></div>
      <div class="ctrl" id="c_sp"><label>SetPoint -30.60~30.60 <b id="vSp">-16.50</b></label><input type="range" id="sp_r" min="-30.60" max="30.60" step="0.01" oninput="onInput('sp',this.value)"><input type="number" id="sp" step="0.01" min="-30.60" max="30.60" oninput="onInput('sp',this.value)"></div>
    </div>
    <div class="row">
      <button class="success" onclick="updatePID()">💾 UPDATE PID NOW</button>
      <button onclick="applyPreset('slow')">🐢 Slow</button>
      <button onclick="applyPreset('agile')">⚡ Agile</button>
    </div>
  </div>

  <div class="card">
    <h1>Balance & Motor - All %.2f <span id="motDirty" style="font-size:11px;color:#fbbf24;display:none">● UNSAVED</span></h1>
    <div class="grid">
      <div class="ctrl" id="c_trim"><label>Trim Angle 0-10 <b id="vTrim">0.00</b></label><input type="range" id="trim_r" min="0" max="10" step="0.01" oninput="onInput('trim',this.value)"><input type="number" id="trim" step="0.01" min="0" max="10" oninput="onInput('trim',this.value)"></div>
      <div class="ctrl" id="c_max"><label>Max Speed PWM 0-255 <b id="vMax">190.00</b></label><input type="range" id="max_r" min="0" max="255" step="0.01" oninput="onInput('max',this.value)"><input type="number" id="max" step="0.01" min="0" max="255" oninput="onInput('max',this.value)"></div>
      <div class="ctrl" id="c_db"><label>Deadband 0-80 <b id="vDb">25.00</b></label><input type="range" id="db_r" min="0" max="80" step="0.01" oninput="onInput('db',this.value)"><input type="number" id="db" step="0.01" min="0" max="80" oninput="onInput('db',this.value)"></div>
      <div class="ctrl" id="c_acc"><label>Accel Step 0-30 <b id="vAcc">5.00</b></label><input type="range" id="acc_r" min="0" max="30" step="0.01" oninput="onInput('acc',this.value)"><input type="number" id="acc" step="0.01" min="0" max="30" oninput="onInput('acc',this.value)"></div>
    </div>
    <div class="row">
      <button class="success" onclick="updateMotor()">💾 UPDATE MOTOR NOW</button>
      <button onclick="applyPreset('original')">↩ Original 20.00/30.00</button>
    </div>
  </div>

  <div class="card">
    <div class="row">
      <button class="primary" style="font-size:16px;padding:14px" onclick="updateAll()">🚀 UPDATE ALL (%.2f) TO ROBOT</button>
    </div>
    <div style="margin-top:12px"><div style="font-size:11px;color:#7a93a9">LOG - No auto update, manual button only</div><div id="log" class="log"></div></div>
  </div>
</div>

<script>
function fmt2(x){ let n=parseFloat(x); if(isNaN(n)) return "0.00"; return n.toFixed(2); }
function logMsg(m){ const el=document.getElementById('log'); const t=new Date().toLocaleTimeString(); el.innerHTML+=`[${t}] ${m}<br>`; el.scrollTop=el.scrollHeight; }

let dirtyPID=false, dirtyMOT=false;

function onInput(k,v){
  let num=parseFloat(v); if(isNaN(num)) return; let s=num.toFixed(2);
  // only UI, no auto send
  if(k==='kp'){document.getElementById('kp').value=s;document.getElementById('kp_r').value=s;document.getElementById('vKp').innerText=s; dirtyPID=true; document.getElementById('c_kp').classList.add('dirty');}
  if(k==='ki'){document.getElementById('ki').value=s;document.getElementById('ki_r').value=s;document.getElementById('vKi').innerText=s; dirtyPID=true; document.getElementById('c_ki').classList.add('dirty');}
  if(k==='kd'){document.getElementById('kd').value=s;document.getElementById('kd_r').value=s;document.getElementById('vKd').innerText=s; dirtyPID=true; document.getElementById('c_kd').classList.add('dirty');}
  if(k==='sp'){document.getElementById('sp').value=s;document.getElementById('sp_r').value=s;document.getElementById('vSp').innerText=s; dirtyPID=true; document.getElementById('c_sp').classList.add('dirty');}
  if(k==='trim'){document.getElementById('trim').value=s;document.getElementById('trim_r').value=s;document.getElementById('vTrim').innerText=s; dirtyMOT=true; document.getElementById('c_trim').classList.add('dirty');}
  if(k==='max'){document.getElementById('max').value=s;document.getElementById('max_r').value=s;document.getElementById('vMax').innerText=s; dirtyMOT=true; document.getElementById('c_max').classList.add('dirty');}
  if(k==='db'){document.getElementById('db').value=s;document.getElementById('db_r').value=s;document.getElementById('vDb').innerText=s; dirtyMOT=true; document.getElementById('c_db').classList.add('dirty');}
  if(k==='acc'){document.getElementById('acc').value=s;document.getElementById('acc_r').value=s;document.getElementById('vAcc').innerText=s; dirtyMOT=true; document.getElementById('c_acc').classList.add('dirty');}
  updateDirtyBadge();
}

function setUIFromRobot(k,s){
  // used by fetchStatus, does NOT mark dirty
  if(k==='kp'){document.getElementById('kp').value=s;document.getElementById('kp_r').value=s;document.getElementById('vKp').innerText=s;}
  if(k==='ki'){document.getElementById('ki').value=s;document.getElementById('ki_r').value=s;document.getElementById('vKi').innerText=s;}
  if(k==='kd'){document.getElementById('kd').value=s;document.getElementById('kd_r').value=s;document.getElementById('vKd').innerText=s;}
  if(k==='sp'){document.getElementById('sp').value=s;document.getElementById('sp_r').value=s;document.getElementById('vSp').innerText=s;}
  if(k==='trim'){document.getElementById('trim').value=s;document.getElementById('trim_r').value=s;document.getElementById('vTrim').innerText=s;}
  if(k==='max'){document.getElementById('max').value=s;document.getElementById('max_r').value=s;document.getElementById('vMax').innerText=s;}
  if(k==='db'){document.getElementById('db').value=s;document.getElementById('db_r').value=s;document.getElementById('vDb').innerText=s;}
  if(k==='acc'){document.getElementById('acc').value=s;document.getElementById('acc_r').value=s;document.getElementById('vAcc').innerText=s;}
}

function updateDirtyBadge(){
  document.getElementById('pidDirty').style.display = dirtyPID ? 'inline' : 'none';
  document.getElementById('motDirty').style.display = dirtyMOT ? 'inline' : 'none';
  document.getElementById('statusTxt').innerText = (dirtyPID||dirtyMOT) ? 'UNSAVED CHANGES' : 'SYNCED';
  document.getElementById('statusTxt').className = (dirtyPID||dirtyMOT) ? 'badge pending' : 'badge';
}

function clearDirty(group){
  if(group==='pid'||group==='all'){
    dirtyPID=false;
    ['c_kp','c_ki','c_kd','c_sp'].forEach(id=>document.getElementById(id).classList.remove('dirty'));
  }
  if(group==='mot'||group==='all'){
    dirtyMOT=false;
    ['c_trim','c_max','c_db','c_acc'].forEach(id=>document.getElementById(id).classList.remove('dirty'));
  }
  updateDirtyBadge();
}

function getVal(id){ return fmt2(document.getElementById(id).value); }

function sendCmd(q){
  document.getElementById('statusTxt').innerText='SENDING...';
  logMsg('Sending '+q);
  fetch('/set?'+q).then(r=>r.json()).then(j=>{
    if(j.ok){ logMsg('✅ Updated '+q); document.getElementById('statusTxt').innerText='UPDATED OK'; setTimeout(()=>{document.getElementById('statusTxt').innerText='SYNCED';},1500); }
    else logMsg('ERR');
  }).catch(e=>logMsg('Fetch err '+e));
}

function updatePID(){
  let q=`kp=${getVal('kp')}&ki=${getVal('ki')}&kd=${getVal('kd')}&sp=${getVal('sp')}`;
  sendCmd(q); clearDirty('pid'); logMsg('PID Update button pressed - '+q);
}
function updateMotor(){
  let q=`trim=${getVal('trim')}&max=${getVal('max')}&db=${getVal('db')}&acc=${getVal('acc')}`;
  sendCmd(q); clearDirty('mot'); logMsg('Motor Update button pressed - '+q);
}
function updateAll(){
  let q=`kp=${getVal('kp')}&ki=${getVal('ki')}&kd=${getVal('kd')}&sp=${getVal('sp')}&trim=${getVal('trim')}&max=${getVal('max')}&db=${getVal('db')}&acc=${getVal('acc')}`;
  sendCmd(q); clearDirty('all'); logMsg('UPDATE ALL - '+q);
}
function resetIntegral(){ sendCmd('reset=1'); logMsg('Integral reset'); }
function applyPreset(p){
  // preset only changes UI, user must press update
  if(p==='slow'){ onInput('kp','15.00'); onInput('ki','18.00'); onInput('kd','0.80'); onInput('max','120.00'); onInput('acc','3.00'); }
  if(p==='agile'){ onInput('kp','28.00'); onInput('ki','35.00'); onInput('kd','1.50'); onInput('max','210.00'); onInput('acc','8.00'); }
  if(p==='original'){ onInput('kp','20.00'); onInput('ki','30.00'); onInput('kd','1.00'); onInput('max','190.00'); onInput('acc','5.00'); onInput('db','25.00'); onInput('trim','0.00'); onInput('sp','-16.50'); }
  logMsg('Preset '+p+' loaded - PRESS UPDATE BUTTON to send');
}

function fetchStatus(){
  fetch('/status').then(r=>r.json()).then(j=>{
    document.getElementById('conn').innerText='LIVE • '+j.fall;
    document.getElementById('pitch').innerText=fmt2(j.pitch);
    document.getElementById('gyro').innerText=fmt2(j.gyro);
    document.getElementById('motorL').innerText=j.mL;
    document.getElementById('motorR').innerText=j.mR;
    document.getElementById('tL').innerText=fmt2(j.tL);
    document.getElementById('tR').innerText=fmt2(j.tR);
    if(j.irF == 1) { document.getElementById('irStateF').innerText="F: EDGE!"; document.getElementById('irStateF').style.borderColor="#ef4444"; document.getElementById('irStateF').style.color="#ef4444"; }
    else { document.getElementById('irStateF').innerText="F: SAFE"; document.getElementById('irStateF').style.borderColor=""; document.getElementById('irStateF').style.color=""; }
    if(j.irB == 1) { document.getElementById('irStateB').innerText="B: EDGE!"; document.getElementById('irStateB').style.borderColor="#ef4444"; document.getElementById('irStateB').style.color="#ef4444"; }
    else { document.getElementById('irStateB').innerText="B: SAFE"; document.getElementById('irStateB').style.borderColor=""; document.getElementById('irStateB').style.color=""; }
    document.getElementById('irRaw').innerText=j.irF + " / " + j.irB;
    document.getElementById('tofDist').innerText=j.tof;
    if(j.atk == 1) { document.getElementById('tofStatus').innerText="ATTACK!"; document.getElementById('tofStatus').style.color="#ef4444"; }
    else if(j.tof > 0 && j.tof < 8190) { document.getElementById('tofStatus').innerText="OK"; document.getElementById('tofStatus').style.color="#34d399"; }
    else { document.getElementById('tofStatus').innerText="OUT OF RANGE"; document.getElementById('tofStatus').style.color="#94a3b8"; }
    let p = ((j.pitch+90)/180)*100; p=Math.max(0,Math.min(100,p));
    document.getElementById('pitchFill').style.width=p+'%';
    // Don't overwrite if user is editing or has unsaved changes
    if(document.activeElement.tagName!=='INPUT' && !dirtyPID && !dirtyMOT){
      setUIFromRobot('kp',fmt2(j.Kp)); setUIFromRobot('ki',fmt2(j.Ki)); setUIFromRobot('kd',fmt2(j.Kd));
      setUIFromRobot('sp',fmt2(j.sp)); setUIFromRobot('trim',fmt2(j.trim)); setUIFromRobot('max',fmt2(j.max)); setUIFromRobot('db',fmt2(j.db)); setUIFromRobot('acc',fmt2(j.acc));
    }
  }).catch(e=>{document.getElementById('conn').innerText='Offline';});
}
setInterval(fetchStatus,300);
fetchStatus();
logMsg('Manual update mode - No auto send. Press UPDATE button.');
</script>
</body>
</html>
)rawliteral";

// Motor
void setMotorRaw(int pinPWM, int pinIn1, int pinIn2, int speed, int ch){
  speed = constrain(speed, -(int)maxSpeed, (int)maxSpeed);
  if(speed>0){digitalWrite(pinIn1,HIGH);digitalWrite(pinIn2,LOW);}
  else if(speed<0){digitalWrite(pinIn1,LOW);digitalWrite(pinIn2,HIGH);speed=-speed;}
  else{digitalWrite(pinIn1,LOW);digitalWrite(pinIn2,LOW);speed=0;}
#ifdef USE_LEDC_V3
  ledcWrite(pinPWM,speed);
#else
  ledcWrite(ch,speed);
#endif
}
void applyMotorSpeeds(){
  int outL=targetSpeedLeft; int outR=targetSpeedRight;
  if(outL>0) outL+= (int)motorDeadband; else if(outL<0) outL-= (int)motorDeadband;
  if(outR>0) outR+= (int)motorDeadband; else if(outR<0) outR-= (int)motorDeadband;
  if(outL>currentSpeedLeft) currentSpeedLeft=min(outL,currentSpeedLeft+(int)accelStep);
  else if(outL<currentSpeedLeft) currentSpeedLeft=max(outL,currentSpeedLeft-(int)accelStep);
  if(outR>currentSpeedRight) currentSpeedRight=min(outR,currentSpeedRight+(int)accelStep);
  else if(outR<currentSpeedRight) currentSpeedRight=max(outR,currentSpeedRight-(int)accelStep);
  setMotorRaw(PIN_PWMA,PIN_AIN1,PIN_AIN2,currentSpeedLeft,0);
  setMotorRaw(PIN_PWMB,PIN_BIN1,PIN_BIN2,currentSpeedRight,1);
}
void stopMotors(){
  targetSpeedLeft=0;targetSpeedRight=0;currentSpeedLeft=0;currentSpeedRight=0;
  setMotorRaw(PIN_PWMA,PIN_AIN1,PIN_AIN2,0,0);
  setMotorRaw(PIN_PWMB,PIN_BIN1,PIN_BIN2,0,1);
}

void handleRoot(){ server.send_P(200,"text/html",INDEX_HTML); }

void handleStatus(){
  String fall=(abs(currentPitch)>FALL_ANGLE)?"FALLEN":"BALANCING";
  String json="{";
  json+="\"pitch\":"+String(currentPitch,2)+",";
  json+="\"gyro\":"+String(gyroRate,2)+",";
  json+="\"mL\":"+String(currentSpeedLeft)+",";
  json+="\"mR\":"+String(currentSpeedRight)+",";
  json+="\"tL\":"+String((float)targetSpeedLeft,2)+",";
  json+="\"tR\":"+String((float)targetSpeedRight,2)+",";
  json+="\"Kp\":"+String(Kp,2)+",";
  json+="\"Ki\":"+String(Ki,2)+",";
  json+="\"Kd\":"+String(Kd,2)+",";
  json+="\"sp\":"+String(setPoint,2)+",";
  json+="\"trim\":"+String(trimAngle,2)+",";
  json+="\"db\":"+String(motorDeadband,2)+",";
  json+="\"acc\":"+String(accelStep,2)+",";
  json+="\"max\":"+String(maxSpeed,2)+",";
  json+="\"irF\":"+String(digitalRead(PIN_IR_FRONT))+",";
  json+="\"irB\":"+String(digitalRead(PIN_IR_BACK))+",";
  json+="\"tof\":"+String(currentDistance)+",";
  json+="\"atk\":"+String(isAttacking ? 1 : 0)+",";
  json+="\"fall\":\""+fall+"\"}";
  server.send(200,"application/json",json);
}

volatile bool requestIntegralReset=false;
void handleSet(){
  bool changed=false;
  if(server.hasArg("kp")){Kp=server.arg("kp").toFloat();changed=true;}
  if(server.hasArg("ki")){Ki=server.arg("ki").toFloat();changed=true;}
  if(server.hasArg("kd")){Kd=server.arg("kd").toFloat();changed=true;}
  if(server.hasArg("sp")){setPoint=server.arg("sp").toFloat();changed=true;}
  if(server.hasArg("trim")){trimAngle=server.arg("trim").toFloat();changed=true;}
  if(server.hasArg("db")){motorDeadband=server.arg("db").toFloat();changed=true;}
  if(server.hasArg("acc")){accelStep=server.arg("acc").toFloat();changed=true;}
  if(server.hasArg("max")){maxSpeed=constrain(server.arg("max").toFloat(),0,MAX_SPEED_LIMIT);changed=true;}
  if(server.hasArg("stop")){stopMotors();}
  if(server.hasArg("reset")){requestIntegralReset=true;}
  if(changed){
    prefs.putFloat("Kp",Kp);prefs.putFloat("Ki",Ki);prefs.putFloat("Kd",Kd);
    prefs.putFloat("sp",setPoint);prefs.putFloat("trim",trimAngle);
    prefs.putFloat("db",motorDeadband);prefs.putFloat("acc",accelStep);prefs.putFloat("max",maxSpeed);
    Serial.printf("[WEB] Update -> Kp:%.2f Ki:%.2f Kd:%.2f SP:%.2f Trim:%.2f max:%.2f db:%.2f acc:%.2f\n",Kp,Ki,Kd,setPoint,trimAngle,maxSpeed,motorDeadband,accelStep);
  }
  server.send(200,"application/json","{\"ok\":true}");
}

void ToFTask(void *pvParameters){
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    VL53L0X_RangingMeasurementData_t measure;
    tof.rangingTest(&measure, false); 
    if(measure.RangeStatus != 4) { currentDistance = measure.RangeMilliMeter; }
    else { currentDistance = -1; }
  }
}

void BalanceTask(void *pvParameters){
  TickType_t xLastWakeTime; const TickType_t xFrequency=pdMS_TO_TICKS(LOOP_TIME_MS);
  float integral=0.0; xLastWakeTime=xTaskGetTickCount();
  while(1){
    vTaskDelayUntil(&xLastWakeTime,xFrequency);
    if(requestIntegralReset){integral=0;requestIntegralReset=false;}
    if(xSemaphoreTake(i2cMutex,pdMS_TO_TICKS(4))==pdTRUE){
      sensors_event_t a,g,temp;
      if(mpu.getEvent(&a,&g,&temp)){
        float accelPitch=atan2(-a.acceleration.x,a.acceleration.z)*180.0/PI;
        gyroRate=g.gyro.y*180.0/PI;
        currentPitch=COMP_ALPHA*(currentPitch+gyroRate*(LOOP_TIME_MS/1000.0))+(1.0-COMP_ALPHA)*accelPitch;
        xSemaphoreGive(i2cMutex);
        if(abs(currentPitch)>FALL_ANGLE){stopMotors();integral=0;}
        else{
          float target=setPoint+trimAngle;
          isAttacking = false;
          
          if (!isEscaping) {
            if (digitalRead(PIN_IR_FRONT) == HIGH) {
              isEscaping = true;
              escapeStartTime = millis();
              escapeDirection = 4.0; // Lean backwards
            } else if (digitalRead(PIN_IR_BACK) == HIGH) {
              isEscaping = true;
              escapeStartTime = millis();
              escapeDirection = -4.0; // Lean forwards
            }
          }
          if (isEscaping) {
            if (millis() - escapeStartTime < 400) {
              target += escapeDirection;
            } else {
              isEscaping = false;
            }
          } else {
            // Priority 2: Attack Mode
            if (currentDistance > 0 && currentDistance <= ATTACK_DISTANCE) {
              isAttacking = true;
              target -= ATTACK_ANGLE; // Lean forwards to push
            }
          }
          
          float error=currentPitch-target;
          integral+=error*(LOOP_TIME_MS/1000.0); integral=constrain(integral,-20.0,20.0);
          float DERIV_FILTER_TAU=0.04; float dt=LOOP_TIME_MS/1000.0; float alphaDeriv=dt/(dt+DERIV_FILTER_TAU);
          static float filteredDeriv=0.0; filteredDeriv=(alphaDeriv*gyroRate)+((1.0-alphaDeriv)*filteredDeriv);
          float output=(Kp*error)+(Ki*integral)+(Kd*filteredDeriv);
          int outSpeed=constrain((int)output,-(int)maxSpeed,(int)maxSpeed);
          targetSpeedLeft=outSpeed; targetSpeedRight=outSpeed;
          
          if (!isEscaping && !isAttacking) {
            targetSpeedLeft += SEARCH_SPEED;
            targetSpeedRight -= SEARCH_SPEED;
          }
          
          applyMotorSpeeds();
        }
      }else{xSemaphoreGive(i2cMutex);stopMotors();}
    }else{stopMotors();}
  }
}

void recoverI2C(){
  pinMode(I2C_SDA,INPUT_PULLUP);pinMode(I2C_SCL,INPUT_PULLUP);delay(10);
  if(digitalRead(I2C_SDA)==LOW){
    pinMode(I2C_SCL,OUTPUT);
    for(int i=0;i<9;i++){digitalWrite(I2C_SCL,HIGH);delayMicroseconds(5);digitalWrite(I2C_SCL,LOW);delayMicroseconds(5);pinMode(I2C_SDA,INPUT_PULLUP);if(digitalRead(I2C_SDA)==HIGH)break;}
  }
}

void setup(){
  pinMode(PIN_IR_FRONT, INPUT);
  pinMode(PIN_IR_BACK, INPUT);
  Serial.begin(115200);delay(1000);recoverI2C();
  Serial.println("\n--- Sumo Balancing WEB %.2f ALL INPUT 2 DECIMAL ---");
  pinMode(PIN_AIN1,OUTPUT);pinMode(PIN_AIN2,OUTPUT);pinMode(PIN_BIN1,OUTPUT);pinMode(PIN_BIN2,OUTPUT);
#ifdef USE_LEDC_V3
  ledcAttach(PIN_PWMA,5000,8);ledcAttach(PIN_PWMB,5000,8);
#else
  ledcSetup(0,5000,8);ledcAttachPin(PIN_PWMA,0);ledcSetup(1,5000,8);ledcAttachPin(PIN_PWMB,1);
#endif
  stopMotors();
  i2cMutex=xSemaphoreCreateMutex();
  Wire.begin(I2C_SDA,I2C_SCL);Wire.setClock(400000);
  if(!mpu.begin(0x68,&Wire)&&!mpu.begin(0x69,&Wire)){Serial.println("FAIL MPU6050");while(1)delay(10);}
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);mpu.setGyroRange(MPU6050_RANGE_500_DEG);mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU OK");

  prefs.begin("sumo-pid",false);
  if(prefs.isKey("Kp")){
    Kp=prefs.getFloat("Kp",Kp);Ki=prefs.getFloat("Ki",Ki);Kd=prefs.getFloat("Kd",Kd);
    setPoint=prefs.getFloat("sp",setPoint);trimAngle=prefs.getFloat("trim",trimAngle);
    motorDeadband=prefs.getFloat("db",motorDeadband);accelStep=prefs.getFloat("acc",accelStep);maxSpeed=prefs.getFloat("max",maxSpeed);
    Serial.printf("Loaded NVS: Kp:%.2f Ki:%.2f Kd:%.2f SP:%.2f DB:%.2f\n",Kp,Ki,Kd,setPoint,motorDeadband);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP("SUMO_BALANCE","",1,0,4);
  Serial.print("AP OPEN SUMO_BALANCE IP: ");Serial.println(WiFi.softAPIP());

  server.on("/",handleRoot);server.on("/status",handleStatus);server.on("/set",handleSet);server.onNotFound(handleRoot);
  server.begin();
  Serial.println("Web http://192.168.4.1 - ALL %.2f");

  Wire1.begin(TOF_SDA, TOF_SCL);
  Wire1.setClock(400000);
  if (!tof.begin(VL53L0X_I2C_ADDR, false, &Wire1)) {
    Serial.println("FAIL VL53L0X");
  } else {
    tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
    Serial.println("TOF OK");
    xTaskCreatePinnedToCore(ToFTask,"ToFTask",4096,NULL,1,NULL,1);
  }

  xTaskCreatePinnedToCore(BalanceTask,"BalanceTask",8192,NULL,2,NULL,1);
}

unsigned long lastDebugTime=0;
void loop(){
  server.handleClient();
  if(millis()-lastDebugTime>1000){
    lastDebugTime=millis();
    Serial.printf("Sudut: %.2f | Motor: %d | Kp:%.2f Ki:%.2f Kd:%.2f SP:%.2f Trim:%.2f max:%.2f db:%.2f acc:%.2f | Clients:%d\n",
      currentPitch,currentSpeedLeft,Kp,Ki,Kd,setPoint,trimAngle,maxSpeed,motorDeadband,accelStep,WiFi.softAPgetStationNum());
  }
  if(Serial.available()){
    String input=Serial.readStringUntil('\n');input.trim();
    if(input.length()>0){
      if(input.startsWith("kp:")){Kp=input.substring(3).toFloat();prefs.putFloat("Kp",Kp);Serial.printf("Kp:%.2f\n",Kp);}
      else if(input.startsWith("ki:")){Ki=input.substring(3).toFloat();prefs.putFloat("Ki",Ki);Serial.printf("Ki:%.2f\n",Ki);}
      else if(input.startsWith("kd:")){Kd=input.substring(3).toFloat();prefs.putFloat("Kd",Kd);Serial.printf("Kd:%.2f\n",Kd);}
      else if(input.startsWith("sp:")){setPoint=input.substring(3).toFloat();prefs.putFloat("sp",setPoint);Serial.printf("SP:%.2f\n",setPoint);}
      else if(input.startsWith("trim:")){trimAngle=input.substring(5).toFloat();prefs.putFloat("trim",trimAngle);Serial.printf("Trim:%.2f\n",trimAngle);}
      else if(input.startsWith("db:")){motorDeadband=input.substring(3).toFloat();prefs.putFloat("db",motorDeadband);Serial.printf("DB:%.2f\n",motorDeadband);}
      else if(input.startsWith("acc:")){accelStep=input.substring(4).toFloat();prefs.putFloat("acc",accelStep);Serial.printf("ACC:%.2f\n",accelStep);}
      else if(input.startsWith("max:")){maxSpeed=input.substring(4).toFloat();prefs.putFloat("max",maxSpeed);Serial.printf("MAX:%.2f\n",maxSpeed);}
    }
  }
  delay(2);
}
