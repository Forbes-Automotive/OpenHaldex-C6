// OpenHaldexC6_Dashboard.ino
// Dashboard with HTML served directly (no SPIFFS needed!)

#include <AsyncWebSocket.h>
#include <ArduinoJson.h>

AsyncWebSocket dashboardWS("/dashboard_ws");

// Function to handle incoming WebSocket messages
void onDashboardWSEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                        void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    sendDashboardUpdate();
    
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, (char *)data);
      
      if (!error) {
        const char *cmd = doc["cmd"];
        
        if (strcmp(cmd, "setMode") == 0) {
          const char *mode = doc["mode"];
          
          if (strcmp(mode, "Stock") == 0) {
            state.mode = MODE_STOCK;
          } else if (strcmp(mode, "FWD") == 0) {
            state.mode = MODE_FWD;
          } else if (strcmp(mode, "6040") == 0) {
            state.mode = MODE_6040;
          } else if (strcmp(mode, "5050") == 0) {
            state.mode = MODE_5050;
          }
          
          sendDashboardUpdate();
        } else if (strcmp(cmd, "getState") == 0) {
          sendDashboardUpdate();
        }
      }
    }
  }
}

// Function to send dashboard data to all connected WebSocket clients
void sendDashboardUpdate() {
  StaticJsonDocument<300> doc;
  
  const char *modeStr = "Stock";
  switch (state.mode) {
    case MODE_STOCK: modeStr = "Stock"; break;
    case MODE_FWD: modeStr = "FWD"; break;
    case MODE_5050: modeStr = "5050"; break;
    case MODE_6040: modeStr = "6040"; break;
    case MODE_7525: modeStr = "7525"; break;
    case MODE_CUSTOM: modeStr = "Custom"; break;
  }
  
  doc["mode"] = modeStr;
  doc["speed"] = received_vehicle_speed;
  doc["locking"] = received_haldex_engagement;
  doc["rpm"] = received_vehicle_rpm;
  doc["boost"] = received_vehicle_boost;
  
  String output;
  serializeJson(doc, output);
  dashboardWS.textAll(output);
}

// Setup function for the dashboard
void setupDashboard() {
  // Attach WebSocket to ESPUI's server
  dashboardWS.onEvent(onDashboardWSEvent);
  ESPUI.WebServer()->addHandler(&dashboardWS);
  
  // Add dashboard route - serve HTML directly
  ESPUI.WebServer()->on("/dashboard", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Build HTML response in chunks to avoid compilation issues
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1.0'><title>OpenHaldex-C6</title><style>";
    html += "*{margin:0;padding:0;box-sizing:border-box}";
    html += "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;background:linear-gradient(135deg,#1e1e1e 0%,#2d2d2d 100%);color:#fff;min-height:100vh;padding:20px}";
    html += ".container{max-width:800px;margin:0 auto}";
    html += "header{text-align:center;margin-bottom:30px;position:relative}";
    html += ".settings-link{position:absolute;top:10px;right:10px;background:rgba(255,255,255,0.1);border:1px solid rgba(255,255,255,0.3);color:#fff;padding:10px 20px;border-radius:8px;text-decoration:none;font-size:0.9em;transition:all 0.3s;font-weight:600}";
    html += ".settings-link:hover{background:rgba(255,255,255,0.2);border-color:#00d4ff;transform:translateY(-2px)}";
    html += "h1{font-size:2.5em;font-weight:700;margin-bottom:10px;background:linear-gradient(45deg,#00d4ff,#00ff88);-webkit-background-clip:text;-webkit-text-fill-color:transparent}";
    html += ".subtitle{font-size:1.1em;color:#aaa}";
    html += ".status-bar{background:rgba(255,255,255,0.05);border-radius:15px;padding:20px;margin-bottom:30px;border:1px solid rgba(255,255,255,0.1)}";
    html += ".status-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:15px}";
    html += ".status-item{text-align:center}";
    html += ".status-label{font-size:0.85em;color:#888;margin-bottom:5px;text-transform:uppercase;letter-spacing:1px}";
    html += ".status-value{font-size:1.8em;font-weight:700;color:#00d4ff}";
    html += ".current-mode{background:rgba(0,212,255,0.1);border:2px solid #00d4ff;border-radius:15px;padding:25px;margin-bottom:30px;text-align:center}";
    html += ".current-mode-label{font-size:0.9em;color:#888;margin-bottom:10px;text-transform:uppercase;letter-spacing:2px}";
    html += ".current-mode-value{font-size:3em;font-weight:900;color:#00d4ff;text-transform:uppercase}";
    html += ".button-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:20px;margin-bottom:30px}";
    html += ".mode-button{background:linear-gradient(135deg,rgba(255,255,255,0.05) 0%,rgba(255,255,255,0.02) 100%);border:2px solid rgba(255,255,255,0.2);border-radius:20px;padding:40px 20px;font-size:1.8em;font-weight:700;color:#fff;cursor:pointer;transition:all 0.3s;text-transform:uppercase;letter-spacing:2px}";
    html += ".mode-button:hover{transform:translateY(-5px);box-shadow:0 10px 30px rgba(0,212,255,0.3);border-color:#00d4ff}";
    html += ".mode-button:active{transform:translateY(-2px)}";
    html += ".mode-button.active{background:linear-gradient(135deg,#00d4ff 0%,#00ff88 100%);border-color:#00d4ff;box-shadow:0 10px 40px rgba(0,212,255,0.5)}";
    html += ".mode-button.stock{background:linear-gradient(135deg,rgba(255,59,59,0.15) 0%,rgba(255,59,59,0.05) 100%);border-color:rgba(255,59,59,0.3)}";
    html += ".mode-button.stock:hover,.mode-button.stock.active{border-color:#ff3b3b;background:linear-gradient(135deg,#ff3b3b 0%,#ff6b6b 100%)}";
    html += ".mode-button.mode-fwd{background:linear-gradient(135deg,rgba(0,255,136,0.15) 0%,rgba(0,255,136,0.05) 100%);border-color:rgba(0,255,136,0.3)}";
    html += ".mode-button.mode-fwd:hover,.mode-button.mode-fwd.active{border-color:#00ff88;background:linear-gradient(135deg,#00ff88 0%,#00cc6a 100%)}";
    html += ".mode-button.mode-6040{background:linear-gradient(135deg,rgba(255,105,180,0.15) 0%,rgba(255,105,180,0.05) 100%);border-color:rgba(255,105,180,0.3)}";
    html += ".mode-button.mode-6040:hover,.mode-button.mode-6040.active{border-color:#ff69b4;background:linear-gradient(135deg,#ff69b4 0%,#ff1493 100%)}";
    html += ".mode-button.mode-5050{background:linear-gradient(135deg,rgba(0,212,255,0.15) 0%,rgba(0,212,255,0.05) 100%);border-color:rgba(0,212,255,0.3)}";
    html += ".mode-button.mode-5050:hover,.mode-button.mode-5050.active{border-color:#00d4ff;background:linear-gradient(135deg,#00d4ff 0%,#00a8cc 100%)}";
    html += ".connection-status{text-align:center;padding:10px;border-radius:10px;margin-bottom:20px;font-weight:600}";
    html += ".connection-status.connected{background:rgba(0,255,136,0.1);color:#00ff88;border:1px solid rgba(0,255,136,0.3)}";
    html += ".connection-status.disconnected{background:rgba(255,59,59,0.1);color:#ff3b3b;border:1px solid rgba(255,59,59,0.3)}";
    html += "footer{text-align:center;color:#666;font-size:0.9em;margin-top:40px}";
    html += "@media (max-width:600px){.settings-link{position:static;display:block;margin-bottom:15px}.button-grid{grid-template-columns:1fr}.mode-button{padding:35px 20px;font-size:1.5em}h1{font-size:2em}.current-mode-value{font-size:2.2em}}";
    html += "</style></head><body><div class='container'><header>";
    html += "<a href='/' class='settings-link'>⚙️ Full Settings</a>";
    html += "<h1>OpenHaldex-C6</h1><p class='subtitle'>Haldex Controller Dashboard</p></header>";
    html += "<div class='connection-status disconnected' id='connectionStatus'>● Connecting...</div>";
    html += "<div class='current-mode'><div class='current-mode-label'>Current Mode</div><div class='current-mode-value' id='currentMode'>---</div></div>";
    html += "<div class='status-bar'><div class='status-grid'>";
    html += "<div class='status-item'><div class='status-label'>Speed</div><div class='status-value' id='speed'>0</div></div>";
    html += "<div class='status-item'><div class='status-label'>Locking</div><div class='status-value' id='locking'>0%</div></div>";
    html += "<div class='status-item'><div class='status-label'>RPM</div><div class='status-value' id='rpm'>0</div></div>";
    html += "<div class='status-item'><div class='status-label'>Boost</div><div class='status-value' id='boost'>0</div></div>";
    html += "</div></div><div class='button-grid'>";
    html += "<button class='mode-button stock' onclick=\"setMode('Stock')\">Stock</button>";
    html += "<button class='mode-button mode-fwd' onclick=\"setMode('FWD')\">FWD</button>";
    html += "<button class='mode-button mode-6040' onclick=\"setMode('6040')\">60/40</button>";
    html += "<button class='mode-button mode-5050' onclick=\"setMode('5050')\">50/50</button>";
    html += "</div><footer><p>Forbes Automotive • OpenHaldex-C6 v1.10</p></footer></div>";
    
    // JavaScript
    html += "<script>";
    html += "var ws=null,reconnectTimer=null;";
    html += "function connectWebSocket(){";
    html += "ws=new WebSocket('ws://'+window.location.hostname+'/dashboard_ws');";
    html += "ws.onopen=function(){";
    html += "document.getElementById('connectionStatus').className='connection-status connected';";
    html += "document.getElementById('connectionStatus').textContent='● Connected';";
    html += "if(ws.readyState===WebSocket.OPEN)ws.send(JSON.stringify({cmd:'getState'}));";
    html += "};";
    html += "ws.onmessage=function(event){";
    html += "try{var data=JSON.parse(event.data);updateDashboard(data);}catch(e){}";
    html += "};";
    html += "ws.onclose=function(){";
    html += "document.getElementById('connectionStatus').className='connection-status disconnected';";
    html += "document.getElementById('connectionStatus').textContent='● Disconnected';";
    html += "if(reconnectTimer)clearTimeout(reconnectTimer);";
    html += "reconnectTimer=setTimeout(connectWebSocket,3000);";
    html += "};";
    html += "}";
    html += "function updateDashboard(data){";
    html += "if(data.mode!==undefined){";
    html += "var modeNames={'Stock':'STOCK','FWD':'FWD','5050':'50/50','6040':'60/40','7525':'75/25','Custom':'CUSTOM'};";
    html += "document.getElementById('currentMode').textContent=modeNames[data.mode]||data.mode;";
    html += "var buttons=document.querySelectorAll('.mode-button');";
    html += "for(var i=0;i<buttons.length;i++)buttons[i].classList.remove('active');";
    html += "var activeBtn=document.querySelector(\".mode-button[onclick=\\\"setMode('\"+data.mode+\"')\\\"]\");";
    html += "if(activeBtn)activeBtn.classList.add('active');";
    html += "}";
    html += "if(data.speed!==undefined)document.getElementById('speed').textContent=data.speed+' km/h';";
    html += "if(data.locking!==undefined)document.getElementById('locking').textContent=data.locking+'%';";
    html += "if(data.rpm!==undefined)document.getElementById('rpm').textContent=data.rpm;";
    html += "if(data.boost!==undefined)document.getElementById('boost').textContent=data.boost+' mbar';";
    html += "}";
    html += "function setMode(mode){";
    html += "if(ws&&ws.readyState===WebSocket.OPEN){";
    html += "ws.send(JSON.stringify({cmd:'setMode',mode:mode}));";
    html += "}else{alert('Not connected to OpenHaldex controller');}";
    html += "}";
    html += "window.addEventListener('load',connectWebSocket);";
    html += "</script></body></html>";
    
    request->send(200, "text/html", html);
  });
  
#if detailedDebugWiFi
  DEBUG("Dashboard initialized at /dashboard");
#endif
}

// Loop function for the dashboard
void loopDashboard() {
  dashboardWS.cleanupClients();
}
