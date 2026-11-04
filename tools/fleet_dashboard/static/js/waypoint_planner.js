/* ==========================================================================
   ConeRobot 2D Cartesian Waypoint Planner
   Coordinate Origin: RTK Base Station (0, 0)
   +X = East in meters, +Y = North in meters
   0 MB extra RAM on rovers - Pure Browser Canvas & Direct WebSocket Streaming
   ========================================================================== */

// --- Global State ---
let activeRobotId = null;
let activeRobotsList = {};
let activeWs = null;
let reconnectTimer = null;

// Base Station Metric Origin (WGS84 Lat/Lon)
// Will be auto-populated if /gps/base_fix arrives from rover RTCM 1005
let baseStationGPS = {
  lat: 47.4979000,
  lon: 19.0402000,
  alt: 100.0,
  locked: false,
  source: 'Default Config'
};

// Robot Telemetry Cache
const robotTelemetry = {
  online: false,
  ip: '',
  port: 8765,
  state: 'IDLE',
  sensorMode: 'GPS (RTK FIX)',
  battery: null,
  headingDeg: 0.0,
  headingRad: 0.0,
  pos: {
    lat: null,
    lon: null,
    x: 0.0, // East in meters from Base
    y: 0.0, // North in meters from Base
  },
  odomPos: { x: 0.0, y: 0.0 },
  rtkStatus: 'Waiting...',
  diffAge: '--',
  satCount: 0,
  trail: [] // Array of {x, y}
};

// Active Target Waypoint
let targetWaypoint = {
  active: false,
  x: 0.0, // East meters from Base (0, 0)
  y: 0.0, // North meters from Base (0, 0)
};

// Canvas Viewport Transformation
const viewState = {
  scale: 22,       // Pixels per meter
  panX: 0,         // Pan offset X in pixels
  panY: 0,         // Pan offset Y in pixels
  isDragging: false,
  dragStartX: 0,
  dragStartY: 0,
  downTime: 0,
  mouseScreenX: 0,
  mouseScreenY: 0,
  mouseWorldX: 0,
  mouseWorldY: 0
};

// Foxglove Client Publish Channel IDs
const CLIENT_CHANNELS = {
  CMD_WAYPOINT: 1,
  CMD_PRIMITIVE: 2
};

// ==============================================================================
// 1. INITIALIZATION & DOM BINDINGS
// ==============================================================================
window.addEventListener('DOMContentLoaded', () => {
  initCanvas();
  bindUIEvents();
  fetchFleetList();
  setInterval(fetchFleetList, 5000);

  // Load config to check if base station coords are pre-set
  fetch('/api/config')
    .then(r => r.json())
    .then(cfg => {
      if (cfg.base_station && cfg.base_station.lat) {
        baseStationGPS.lat = cfg.base_station.lat;
        baseStationGPS.lon = cfg.base_station.lon;
        baseStationGPS.source = 'fleet_config.json';
        updateBaseStationUI();
      } else if (cfg.map && cfg.map.default_lat) {
        baseStationGPS.lat = cfg.map.default_lat;
        baseStationGPS.lon = cfg.map.default_lon;
        updateBaseStationUI();
      }
    }).catch(() => {});

  logMission('System ready. Please select an active robot to begin planning.', 'info');
  requestAnimationFrame(renderLoop);
});

function bindUIEvents() {
  const robotSelect = document.getElementById('select-active-robot');
  robotSelect.addEventListener('change', (e) => {
    const id = parseInt(e.target.value, 10);
    if (id) selectRobot(id);
  });

  // Canvas View Controls
  document.getElementById('btn-zoom-in').addEventListener('click', () => zoomView(1.25));
  document.getElementById('btn-zoom-out').addEventListener('click', () => zoomView(0.80));
  document.getElementById('btn-center-base').addEventListener('click', centerOnBase);
  document.getElementById('btn-center-robot').addEventListener('click', centerOnRobot);
  document.getElementById('btn-fit-view').addEventListener('click', fitAll);

  // Waypoint Action Buttons
  document.getElementById('btn-dispatch-waypoint').addEventListener('click', dispatchWaypoint);
  document.getElementById('btn-return-home').addEventListener('click', returnToBase);
  document.getElementById('btn-estop').addEventListener('click', triggerEmergencyStop);
  document.getElementById('btn-clear-target').addEventListener('click', clearTarget);
  document.getElementById('btn-set-manual').addEventListener('click', applyManualCoordinates);

  // Network Scan
  document.getElementById('btn-scan-network').addEventListener('click', () => {
    fetch('/api/scan')
      .then(r => r.json())
      .then(() => {
        logMission('Triggered background network scan for active rovers...', 'info');
        setTimeout(fetchFleetList, 1500);
      });
  });

  // Manual Base Station Override
  document.getElementById('btn-edit-base').addEventListener('click', promptManualBaseCoords);

  // Coordinate Inputs
  document.getElementById('input-target-x').addEventListener('change', updateTargetFromInputs);
  document.getElementById('input-target-y').addEventListener('change', updateTargetFromInputs);
}

// ==============================================================================
// 2. FLEET DISCOVERY & WEBSOCKET ENGINE
// ==============================================================================
async function fetchFleetList() {
  try {
    const res = await fetch('/api/fleet');
    if (!res.ok) return;
    const data = await res.json();
    const robots = data.robots || {};
    activeRobotsList = robots;

    const select = document.getElementById('select-active-robot');
    const currentVal = select.value;

    let html = '';
    let onlineCount = 0;
    let firstOnlineId = null;

    for (let id = 1; id <= 13; id++) {
      const r = robots[id];
      if (r && r.online) {
        onlineCount++;
        if (!firstOnlineId) firstOnlineId = id;
        html += `<option value="${id}">🤖 ${r.name || `ConeRobot ${id < 10 ? '0' + id : id}`} (${r.ip}) [ONLINE]</option>`;
      } else if (r) {
        html += `<option value="${id}" style="color: #64748b;">ConeRobot ${id < 10 ? '0' + id : id} [Offline]</option>`;
      }
    }

    if (onlineCount === 0) {
      html = '<option value="">No online robots found (Click Scan Network)</option>';
    }

    select.innerHTML = html;

    if (currentVal && robots[currentVal] && robots[currentVal].online) {
      select.value = currentVal;
    } else if (firstOnlineId && !activeRobotId) {
      select.value = firstOnlineId;
      selectRobot(firstOnlineId);
    }
  } catch (err) {
    console.warn('Error fetching fleet:', err);
  }
}

function selectRobot(id) {
  if (activeRobotId === id && activeWs && activeWs.readyState === WebSocket.OPEN) return;
  activeRobotId = id;
  const r = activeRobotsList[id];
  if (!r) return;

  logMission(`Connecting to Robot ${id < 10 ? '0' + id : id} at ${r.ip}:${r.port || 8765}...`, 'info');
  connectWebSocket(id, r.ip, r.port || 8765);
}

function connectWebSocket(id, ip, port) {
  if (activeWs) {
    try { activeWs.close(); } catch (e) {}
    activeWs = null;
  }
  if (reconnectTimer) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }

  const wsUrl = `ws://${ip}:${port}`;
  let ws;
  try {
    ws = new WebSocket(wsUrl, ["foxglove.sdk.v1", "foxglove.websocket.v1"]);
  } catch (e) {
    try { ws = new WebSocket(wsUrl); } catch (err) {
      handleWsDisconnect(id, ip, port, err.message);
      return;
    }
  }

  ws.binaryType = "arraybuffer";
  activeWs = ws;

  const subIdToTopic = {};
  const channelIdToTopic = {};
  let subCounter = 1;

  ws.onopen = () => {
    logMission(`🟢 Connected to Robot ${id < 10 ? '0' + id : id} (${ip}:${port})!`, 'success');
    updateConnBadge(true, ip);
    robotTelemetry.online = true;
    robotTelemetry.ip = ip;
    robotTelemetry.trail = []; // reset trail on fresh connect

    // 1. Identify Client
    try {
      ws.send(JSON.stringify({ op: "clientInfo", name: "ConeRobotWaypointPlanner" }));
    } catch (e) {}

    // 2. Advertise Client Publish Channels (/cmd_waypoint and /cmd_primitive)
    try {
      ws.send(JSON.stringify({
        op: "advertise",
        channels: [
          {
            id: CLIENT_CHANNELS.CMD_WAYPOINT,
            topic: "/cmd_waypoint",
            encoding: "cdr",
            schemaName: "geometry_msgs/msg/Vector3"
          },
          {
            id: CLIENT_CHANNELS.CMD_PRIMITIVE,
            topic: "/cmd_primitive",
            encoding: "cdr",
            schemaName: "geometry_msgs/msg/Vector3"
          }
        ]
      }));
      logMission('Advertised client channels: /cmd_waypoint and /cmd_primitive (Vector3 CDR).', 'info');
    } catch (e) {
      console.error('Failed to advertise client channels:', e);
    }
  };

  ws.onmessage = (event) => {
    if (typeof event.data === "string") {
      try {
        const msg = JSON.parse(event.data);
        if (msg.op === "advertise" && Array.isArray(msg.channels)) {
          const subscriptions = [];
          const targetTopics = [
            '/fix',
            '/gps/base_fix',
            '/gps/status',
            '/imu/heading',
            '/step_status',
            '/odom',
            '/battery_state'
          ];

          msg.channels.forEach(ch => {
            const rawTopic = ch.topic || '';
            const topic = rawTopic.startsWith('/') ? rawTopic : '/' + rawTopic;
            if (targetTopics.includes(topic) || topic.endsWith('/fix') || topic.endsWith('/heading')) {
              const subId = subCounter++;
              subIdToTopic[subId] = topic;
              channelIdToTopic[ch.id] = topic;
              subscriptions.push({ id: subId, channelId: ch.id });
            }
          });

          if (subscriptions.length > 0) {
            ws.send(JSON.stringify({ op: "subscribe", subscriptions: subscriptions }));
          }
        } else if (msg.op === "message" && msg.data) {
          const topic = subIdToTopic[msg.subscriptionId] || channelIdToTopic[msg.channelId];
          handleTelemetryMessage(topic, msg.data);
        }
      } catch (e) {}
    } else if (event.data instanceof ArrayBuffer) {
      handleBinaryFoxgloveMessage(event.data, subIdToTopic, channelIdToTopic);
    }
  };

  ws.onerror = () => {};

  ws.onclose = () => {
    handleWsDisconnect(id, ip, port, 'Closed');
  };
}

function handleWsDisconnect(id, ip, port, reason) {
  updateConnBadge(false);
  robotTelemetry.online = false;
  activeWs = null;
  if (reconnectTimer) clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(() => {
    if (activeRobotId === id) connectWebSocket(id, ip, port);
  }, 4000);
}

function updateConnBadge(isOnline, ip) {
  const pill = document.getElementById('robot-conn-pill');
  if (isOnline) {
    pill.className = 'state-badge idle';
    pill.textContent = `Online (${ip})`;
  } else {
    pill.className = 'state-badge error';
    pill.textContent = 'Offline / Connecting';
  }
}

// ==============================================================================
// 3. TELEMETRY PROCESSING & COORDINATE MAPPING
// ==============================================================================
function handleBinaryFoxgloveMessage(buffer, subIdToTopic, channelIdToTopic) {
  if (buffer.byteLength < 17) return;
  const view = new DataView(buffer);
  const subId = view.getUint32(1, true);
  const topic = subIdToTopic[subId];
  if (!topic) return;

  const dataStart = 17; // ROS2 CDR payload starts after Foxglove 13-byte envelope + 4-byte CDR header

  // 1. /fix (sensor_msgs/msg/NavSatFix)
  if (topic === '/fix' || topic.endsWith('/fix')) {
    if (topic.includes('base')) {
      handleBaseFixBinary(view, dataStart);
    } else {
      handleRoverFixBinary(view, dataStart);
    }
  }
  // 2. /imu/heading (std_msgs/msg/Float32)
  else if (topic === '/imu/heading' || topic.endsWith('/heading')) {
    try {
      const headingRad = view.getFloat32(dataStart, true);
      robotTelemetry.headingRad = headingRad;
      robotTelemetry.headingDeg = (headingRad * 180.0 / Math.PI + 360.0) % 360.0;
      updateTelemetryUI();
    } catch (e) {}
  }
  // 3. /step_status (std_msgs/msg/String)
  else if (topic === '/step_status' || topic.endsWith('/step_status')) {
    try {
      const strLen = view.getUint32(dataStart, true);
      if (strLen > 0 && strLen < 256) {
        let str = '';
        for (let i = 0; i < strLen - 1; i++) {
          str += String.fromCharCode(view.getUint8(dataStart + 4 + i));
        }
        handleStepStatusString(str);
      }
    } catch (e) {}
  }
  // 4. /battery_state (sensor_msgs/msg/BatteryState)
  else if (topic === '/battery_state' || topic.includes('battery')) {
    try {
      // Float32 voltage at offset after Header
      let ptr = dataStart;
      ptr += 8; // skip stamp
      const frameIdLen = view.getUint32(ptr, true);
      ptr += 4 + Math.ceil(frameIdLen / 4) * 4;
      const voltage = view.getFloat32(ptr, true);
      if (voltage > 5.0 && voltage < 30.0) {
        robotTelemetry.battery = voltage.toFixed(1);
        updateTelemetryUI();
      }
    } catch (e) {}
  }
  // 5. /gps/status (std_msgs/msg/String)
  else if (topic === '/gps/status' || topic.endsWith('/gps/status')) {
    try {
      const strLen = view.getUint32(dataStart, true);
      if (strLen > 0 && strLen < 512) {
        let str = '';
        for (let i = 0; i < strLen - 1; i++) {
          str += String.fromCharCode(view.getUint8(dataStart + 4 + i));
        }
        handleGpsStatusString(str);
      }
    } catch (e) {}
  }
}

function handleRoverFixBinary(view, dataStart) {
  try {
    let ptr = dataStart;
    ptr += 8; // skip timestamp (sec, nanosec)
    const frameIdLen = view.getUint32(ptr, true);
    ptr += 4 + Math.ceil(frameIdLen / 4) * 4; // aligned frame_id

    // NavSatStatus (status: int8, service: uint16)
    const status = view.getInt8(ptr);
    ptr += 1;
    ptr += 1; // alignment padding
    ptr += 2; // service uint16
    ptr += 4; // align to 8 bytes for float64

    const lat = view.getFloat64(ptr, true);
    const lon = view.getFloat64(ptr + 8, true);
    const alt = view.getFloat64(ptr + 16, true);

    if (Math.abs(lat) > 0.0001 && Math.abs(lon) > 0.0001) {
      robotTelemetry.pos.lat = lat;
      robotTelemetry.pos.lon = lon;

      // Transform Global Lat/Lon into Base (0, 0) Metric Cartesian Coordinates
      const enu = latLonToEnuMeters(lat, lon, baseStationGPS.lat, baseStationGPS.lon);
      robotTelemetry.pos.x = enu.x; // East in meters
      robotTelemetry.pos.y = enu.y; // North in meters

      // Add to breadcrumb trail
      const last = robotTelemetry.trail[robotTelemetry.trail.length - 1];
      if (!last || Math.hypot(enu.x - last.x, enu.y - last.y) >= 0.15) {
        robotTelemetry.trail.push({ x: enu.x, y: enu.y });
        if (robotTelemetry.trail.length > 500) robotTelemetry.trail.shift();
      }

      updateTelemetryUI();
      updateTargetMetrics();
    }
  } catch (e) {}
}

function handleBaseFixBinary(view, dataStart) {
  try {
    let ptr = dataStart + 8;
    const frameIdLen = view.getUint32(ptr, true);
    ptr += 4 + Math.ceil(frameIdLen / 4) * 4;
    ptr += 4; // status + service
    ptr += 4; // alignment to float64

    const bLat = view.getFloat64(ptr, true);
    const bLon = view.getFloat64(ptr + 8, true);
    const bAlt = view.getFloat64(ptr + 16, true);

    if (Math.abs(bLat) > 0.001 && Math.abs(bLon) > 0.001) {
      if (!baseStationGPS.locked || Math.abs(baseStationGPS.lat - bLat) > 1e-6) {
        baseStationGPS.lat = bLat;
        baseStationGPS.lon = bLon;
        baseStationGPS.alt = bAlt;
        baseStationGPS.locked = true;
        baseStationGPS.source = 'RTCM 1005 (Decoded from Rover)';
        logMission(`🏰 RTK Base Station (0, 0) locked via RTCM 1005: ${bLat.toFixed(7)}°, ${bLon.toFixed(7)}°`, 'success');
        updateBaseStationUI();
      }
    }
  } catch (e) {}
}

function handleGpsStatusString(str) {
  if (!str) return;
  // Parse RTK fix mode
  if (str.includes('RTK FIX') || str.includes('RTK Fix')) {
    robotTelemetry.rtkStatus = 'RTK FIX (Mode 4)';
    robotTelemetry.sensorMode = 'GPS (RTK FIX)';
  } else if (str.includes('FLOAT') || str.includes('Float')) {
    robotTelemetry.rtkStatus = 'RTK FLOAT (Mode 5)';
    robotTelemetry.sensorMode = 'GPS (RTK FLOAT)';
  } else if (str.includes('3D FIX') || str.includes('3D Fix')) {
    robotTelemetry.rtkStatus = '3D FIX (Mode 1/2)';
    robotTelemetry.sensorMode = 'GPS (3D FIX)';
  }

  // Parse age of diff corrections
  const ageMatch = str.match(/Corr\.\s*age:\s*([\d.]+s|-)/i);
  if (ageMatch) robotTelemetry.diffAge = ageMatch[1];

  updateTelemetryUI();
}

function handleStepStatusString(str) {
  if (!str) return;
  let state = 'IDLE';
  if (str.includes('IDLE')) state = 'IDLE';
  else if (str.includes('EXECUTING') || str.includes('MOVING') || str.includes('DRIVE') || str.includes('TURN')) state = 'EXECUTING';
  else if (str.includes('SENSOR_ERROR')) state = 'SENSOR_ERROR';
  else if (str.includes('CANCELING')) state = 'CANCELING';

  robotTelemetry.state = state;
  updateTelemetryUI();
}

// Convert WGS84 Lat/Lon to local Cartesian ENU meters relative to reference (Base 0, 0)
function latLonToEnuMeters(lat, lon, refLat, refLon) {
  const R = 6371000.0; // Earth radius in meters
  const dLatRad = (lat - refLat) * Math.PI / 180.0;
  const dLonRad = (lon - refLon) * Math.PI / 180.0;
  const latAvgRad = ((lat + refLat) / 2.0) * Math.PI / 180.0;

  const eastM = dLonRad * Math.cos(latAvgRad) * R;
  const northM = dLatRad * R;
  return { x: eastM, y: northM };
}

// ==============================================================================
// 4. UI UPDATE HELPERS
// ==============================================================================
function updateTelemetryUI() {
  document.getElementById('tel-base-pos').textContent = `X: ${robotTelemetry.pos.x >= 0 ? '+' : ''}${robotTelemetry.pos.x.toFixed(2)}m, Y: ${robotTelemetry.pos.y >= 0 ? '+' : ''}${robotTelemetry.pos.y.toFixed(2)}m`;
  document.getElementById('tel-sensor-mode').textContent = robotTelemetry.sensorMode;

  const rtkEl = document.getElementById('tel-rtk-status');
  rtkEl.textContent = robotTelemetry.rtkStatus;
  if (robotTelemetry.rtkStatus.includes('FIX')) rtkEl.style.color = '#10b981';
  else if (robotTelemetry.rtkStatus.includes('FLOAT')) rtkEl.style.color = '#f59e0b';
  else rtkEl.style.color = '#ef4444';

  document.getElementById('tel-diff-age').textContent = robotTelemetry.diffAge;
  document.getElementById('tel-heading').textContent = `${robotTelemetry.headingDeg.toFixed(1)}° ENU`;
  document.getElementById('tel-battery').textContent = robotTelemetry.battery ? `${robotTelemetry.battery} V` : '-- V';

  const stateBadge = document.getElementById('telemetry-state-badge');
  stateBadge.textContent = robotTelemetry.state;
  if (robotTelemetry.state === 'IDLE') stateBadge.className = 'state-badge idle';
  else if (robotTelemetry.state === 'EXECUTING') stateBadge.className = 'state-badge executing';
  else stateBadge.className = 'state-badge error';
}

function updateBaseStationUI() {
  document.getElementById('header-base-coords').textContent = `${baseStationGPS.lat.toFixed(6)}°, ${baseStationGPS.lon.toFixed(6)}°`;
  document.getElementById('base-ref-lat').textContent = `${baseStationGPS.lat.toFixed(8)}°`;
  document.getElementById('base-ref-lon').textContent = `${baseStationGPS.lon.toFixed(8)}°`;
  document.getElementById('base-ref-source').textContent = baseStationGPS.source;
}

function updateTargetMetrics() {
  if (!targetWaypoint.active) {
    document.getElementById('metric-dist').textContent = '-- m';
    document.getElementById('metric-bearing').textContent = '--°';
    document.getElementById('header-active-target').textContent = 'None';
    return;
  }

  const dx = targetWaypoint.x - robotTelemetry.pos.x;
  const dy = targetWaypoint.y - robotTelemetry.pos.y;
  const dist = Math.hypot(dx, dy);

  // Bearing in standard ENU: 0 = East (+X), 90 = North (+Y)
  let bearingRad = Math.atan2(dy, dx);
  let bearingDeg = (bearingRad * 180.0 / Math.PI + 360.0) % 360.0;

  document.getElementById('metric-dist').textContent = `${dist.toFixed(2)} m`;
  document.getElementById('metric-bearing').textContent = `${bearingDeg.toFixed(1)}°`;
  document.getElementById('header-active-target').textContent = `(${targetWaypoint.x >= 0 ? '+' : ''}${targetWaypoint.x.toFixed(2)}m, ${targetWaypoint.y >= 0 ? '+' : ''}${targetWaypoint.y.toFixed(2)}m)`;
}

function logMission(msg, type = 'info') {
  const box = document.getElementById('mission-log-box');
  if (!box) return;
  const d = new Date();
  const timeStr = d.toTimeString().split(' ')[0] + '.' + String(d.getMilliseconds()).padStart(3, '0');
  const line = document.createElement('div');
  line.className = `term-line ${type}`;
  line.innerHTML = `<span class="term-time">[${timeStr}]</span> ${escapeHtml(msg)}`;
  box.appendChild(line);
  box.scrollTop = box.scrollHeight;
}

function clearMissionLog() {
  const box = document.getElementById('mission-log-box');
  if (box) box.innerHTML = '';
}

function escapeHtml(s) {
  return String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
}

// ==============================================================================
// 5. WAYPOINT DISPATCH & ACTION LOGIC
// ==============================================================================
function encodeVector3CDR(channelId, x, y, z) {
  // 33-byte ArrayBuffer:
  // Byte 0: Opcode 1 (clientMessage)
  // Byte 1..4: uint32 LE (channelId)
  // Byte 5..8: ROS 2 CDR Header [0x00, 0x01, 0x00, 0x00] (CDR_LE)
  // Byte 9..16: Float64 LE (x)
  // Byte 17..24: Float64 LE (y)
  // Byte 25..32: Float64 LE (z)
  const buf = new ArrayBuffer(33);
  const view = new DataView(buf);
  view.setUint8(0, 1);
  view.setUint32(1, channelId, true);
  view.setUint8(5, 0x00);
  view.setUint8(6, 0x01);
  view.setUint8(7, 0x00);
  view.setUint8(8, 0x00);
  view.setFloat64(9, Number(x), true);
  view.setFloat64(17, Number(y), true);
  view.setFloat64(25, Number(z), true);
  return buf;
}

function dispatchWaypoint() {
  if (!targetWaypoint.active) {
    alert('Please click on the 2D map to set a target waypoint first!');
    return;
  }
  if (!activeRobotId) {
    alert('No active robot selected! Please select an online robot.');
    return;
  }

  const x = targetWaypoint.x;
  const y = targetWaypoint.y;
  const z = 0.0; // 0.0 indicates Base-relative ENU meters to primitive_motion_controller

  logMission(`🚀 Dispatching Robot ${activeRobotId} -> Target: (X: ${x.toFixed(2)}m, Y: ${y.toFixed(2)}m)...`, 'info');

  // 1. Direct Foxglove WebSocket Binary Dispatch (0 MB rover RAM)
  let sentViaWs = false;
  if (activeWs && activeWs.readyState === WebSocket.OPEN) {
    try {
      const cdrBuf = encodeVector3CDR(CLIENT_CHANNELS.CMD_WAYPOINT, x, y, z);
      activeWs.send(cdrBuf);
      sentViaWs = true;
      logMission(`✔ Sent /cmd_waypoint via Foxglove WebSocket to Rover!`, 'success');
    } catch (err) {
      console.warn('WebSocket send error:', err);
    }
  }

  // 2. HTTP Relay Fallback to /api/waypoint
  fetch('/api/waypoint', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      robot_id: activeRobotId,
      robot_ip: robotTelemetry.ip,
      type: 'GOTO_BASE_XY',
      x: x,
      y: y,
      z: z
    })
  }).then(r => r.json())
    .then(data => {
      if (!sentViaWs) {
        logMission(`✔ Dispatched via HTTP Backend Relay!`, 'success');
      }
    }).catch(err => {
      if (!sentViaWs) {
        logMission(`✖ Failed to send command: ${err.message}`, 'error');
      }
    });
}

function returnToBase() {
  targetWaypoint.active = true;
  targetWaypoint.x = 0.00;
  targetWaypoint.y = 0.00;
  document.getElementById('input-target-x').value = '0.00';
  document.getElementById('input-target-y').value = '0.00';
  updateTargetMetrics();
  logMission(`🏠 Target set to RTK Base Station (0.00m, 0.00m).`, 'warn');
  dispatchWaypoint();
}

function triggerEmergencyStop() {
  logMission(`🛑 EMERGENCY STOP TRIGGERED! Halting robot immediately...`, 'error');

  // Send /cmd_primitive with {x: 0, y: 0, z: 0}
  if (activeWs && activeWs.readyState === WebSocket.OPEN) {
    try {
      const stopBuf = encodeVector3CDR(CLIENT_CHANNELS.CMD_PRIMITIVE, 0.0, 0.0, 0.0);
      activeWs.send(stopBuf);
      logMission(`✔ Emergency stop packet broadcasted on /cmd_primitive!`, 'warn');
    } catch (e) {}
  }

  fetch('/api/waypoint', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      robot_id: activeRobotId,
      type: 'STOP',
      x: 0.0,
      y: 0.0,
      z: 0.0
    })
  });
}

function clearTarget() {
  targetWaypoint.active = false;
  targetWaypoint.x = 0.0;
  targetWaypoint.y = 0.0;
  document.getElementById('input-target-x').value = '0.00';
  document.getElementById('input-target-y').value = '0.00';
  updateTargetMetrics();
  logMission('Target waypoint cleared.', 'info');
}

function nudgeCoord(axis, delta) {
  const input = document.getElementById(axis === 'x' ? 'input-target-x' : 'input-target-y');
  let val = parseFloat(input.value) || 0.0;
  val = Math.round((val + delta) * 100) / 100;
  input.value = val.toFixed(2);
  updateTargetFromInputs();
}

function updateTargetFromInputs() {
  const x = parseFloat(document.getElementById('input-target-x').value) || 0.0;
  const y = parseFloat(document.getElementById('input-target-y').value) || 0.0;
  targetWaypoint.x = Math.round(x * 100) / 100;
  targetWaypoint.y = Math.round(y * 100) / 100;
  targetWaypoint.active = true;
  updateTargetMetrics();
}

function applyManualCoordinates() {
  updateTargetFromInputs();
  logMission(`Applied manual coordinates: (X: ${targetWaypoint.x.toFixed(2)}m, Y: ${targetWaypoint.y.toFixed(2)}m)`, 'info');
}

function promptManualBaseCoords() {
  const current = `${baseStationGPS.lat.toFixed(7)}, ${baseStationGPS.lon.toFixed(7)}`;
  const input = prompt('Enter RTK Base Station Latitude, Longitude (WGS84 degrees):', current);
  if (!input) return;
  const parts = input.split(',').map(s => parseFloat(s.trim()));
  if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
    baseStationGPS.lat = parts[0];
    baseStationGPS.lon = parts[1];
    baseStationGPS.locked = true;
    baseStationGPS.source = 'Manual User Override';
    logMission(`Base Station reference updated: ${parts[0]}°, ${parts[1]}°`, 'success');
    updateBaseStationUI();
    // Re-project robot position
    if (robotTelemetry.pos.lat !== null) {
      const enu = latLonToEnuMeters(robotTelemetry.pos.lat, robotTelemetry.pos.lon, parts[0], parts[1]);
      robotTelemetry.pos.x = enu.x;
      robotTelemetry.pos.y = enu.y;
      updateTelemetryUI();
      updateTargetMetrics();
    }
  } else {
    alert('Invalid coordinate format. Example: 47.4979123, 19.0402123');
  }
}

// ==============================================================================
// 6. HIGH-PERFORMANCE 2D CANVAS CARTESIAN ENGINE
// ==============================================================================
let canvas, ctx;

function initCanvas() {
  canvas = document.getElementById('waypoint-canvas');
  ctx = canvas.getContext('2d');

  function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
  }

  window.addEventListener('resize', resizeCanvas);
  resizeCanvas();

  // Mouse Interactivity
  canvas.addEventListener('mousedown', (e) => {
    viewState.isDragging = true;
    viewState.dragStartX = e.clientX;
    viewState.dragStartY = e.clientY;
    viewState.downTime = Date.now();
  });

  window.addEventListener('mousemove', (e) => {
    const rect = canvas.getBoundingClientRect();
    const screenX = e.clientX - rect.left;
    const screenY = e.clientY - rect.top;
    viewState.mouseScreenX = screenX;
    viewState.mouseScreenY = screenY;

    if (viewState.isDragging) {
      const dx = e.clientX - viewState.dragStartX;
      const dy = e.clientY - viewState.dragStartY;
      viewState.panX += dx;
      viewState.panY += dy;
      viewState.dragStartX = e.clientX;
      viewState.dragStartY = e.clientY;
    }

    // World coordinates under cursor
    const world = canvasToWorld(screenX, screenY);
    viewState.mouseWorldX = world.x;
    viewState.mouseWorldY = world.y;

    // Update bottom cursor coordinate pill
    const distFromBase = Math.hypot(world.x, world.y);
    const signX = world.x >= 0 ? '+' : '';
    const signY = world.y >= 0 ? '+' : '';
    document.getElementById('cursor-coord-text').textContent =
      `X: ${signX}${world.x.toFixed(2)} m (${world.x >= 0 ? 'E' : 'W'}) | Y: ${signY}${world.y.toFixed(2)} m (${world.y >= 0 ? 'N' : 'S'})`;
    document.getElementById('cursor-dist-text').textContent = `(Dist: ${distFromBase.toFixed(2)} m)`;
  });

  window.addEventListener('mouseup', (e) => {
    if (!viewState.isDragging) return;
    viewState.isDragging = false;

    // Check if this was a clean CLICK (not a drag)
    const rect = canvas.getBoundingClientRect();
    if (
      e.clientX >= rect.left && e.clientX <= rect.right &&
      e.clientY >= rect.top && e.clientY <= rect.bottom
    ) {
      const dt = Date.now() - viewState.downTime;
      const dragDist = Math.hypot(e.clientX - viewState.dragStartX, e.clientY - viewState.dragStartY);
      if (dt < 400 && dragDist < 5) {
        // Place Target Waypoint
        const screenX = e.clientX - rect.left;
        const screenY = e.clientY - rect.top;
        const world = canvasToWorld(screenX, screenY);

        targetWaypoint.x = Math.round(world.x * 100) / 100;
        targetWaypoint.y = Math.round(world.y * 100) / 100;
        targetWaypoint.active = true;

        document.getElementById('input-target-x').value = targetWaypoint.x.toFixed(2);
        document.getElementById('input-target-y').value = targetWaypoint.y.toFixed(2);
        updateTargetMetrics();

        logMission(`Target placed at (X: ${targetWaypoint.x.toFixed(2)}m, Y: ${targetWaypoint.y.toFixed(2)}m). Ready to dispatch!`, 'info');
      }
    }
  });

  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const rect = canvas.getBoundingClientRect();
    const cursorX = e.clientX - rect.left;
    const cursorY = e.clientY - rect.top;

    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
    zoomViewAroundPoint(zoomFactor, cursorX, cursorY);
  }, { passive: false });
}

// Convert World (Meters) to Canvas (Pixels)
function worldToCanvas(wx, wy) {
  const dpr = window.devicePixelRatio || 1;
  const cx = (canvas.width / dpr) / 2 + viewState.panX;
  const cy = (canvas.height / dpr) / 2 + viewState.panY;

  return {
    x: cx + wx * viewState.scale,
    y: cy - wy * viewState.scale // Invert Y because canvas Y points down, North (+Y) points up
  };
}

// Convert Canvas (Pixels) to World (Meters)
function canvasToWorld(cx_px, cy_px) {
  const dpr = window.devicePixelRatio || 1;
  const cx = (canvas.width / dpr) / 2 + viewState.panX;
  const cy = (canvas.height / dpr) / 2 + viewState.panY;

  return {
    x: (cx_px - cx) / viewState.scale,
    y: (cy - cy_px) / viewState.scale
  };
}

function zoomView(factor) {
  const dpr = window.devicePixelRatio || 1;
  const midX = (canvas.width / dpr) / 2;
  const midY = (canvas.height / dpr) / 2;
  zoomViewAroundPoint(factor, midX, midY);
}

function zoomViewAroundPoint(factor, pointX, pointY) {
  const oldScale = viewState.scale;
  const newScale = Math.max(3.0, Math.min(250.0, oldScale * factor));
  if (oldScale === newScale) return;

  // Keep the point under cursor fixed in world space
  const world = canvasToWorld(pointX, pointY);
  viewState.scale = newScale;
  const newScreen = worldToCanvas(world.x, world.y);
  viewState.panX += (pointX - newScreen.x);
  viewState.panY += (pointY - newScreen.y);
}

function centerOnBase() {
  viewState.panX = 0;
  viewState.panY = 0;
}

function centerOnRobot() {
  viewState.panX = -robotTelemetry.pos.x * viewState.scale;
  viewState.panY = robotTelemetry.pos.y * viewState.scale;
}

function fitAll() {
  const points = [{ x: 0, y: 0 }];
  if (robotTelemetry.pos.x !== 0 || robotTelemetry.pos.y !== 0) {
    points.push({ x: robotTelemetry.pos.x, y: robotTelemetry.pos.y });
  }
  if (targetWaypoint.active) {
    points.push({ x: targetWaypoint.x, y: targetWaypoint.y });
  }

  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  points.forEach(p => {
    minX = Math.min(minX, p.x);
    maxX = Math.max(maxX, p.x);
    minY = Math.min(minY, p.y);
    maxY = Math.max(maxY, p.y);
  });

  // Add 4-meter margin
  minX -= 4; maxX += 4;
  minY -= 4; maxY += 4;

  const dpr = window.devicePixelRatio || 1;
  const cw = (canvas.width / dpr);
  const ch = (canvas.height / dpr);

  const spanX = Math.max(8, maxX - minX);
  const spanY = Math.max(8, maxY - minY);

  viewState.scale = Math.min(cw / spanX, ch / spanY);
  const midWorldX = (minX + maxX) / 2;
  const midWorldY = (minY + maxY) / 2;

  viewState.panX = -midWorldX * viewState.scale;
  viewState.panY = midWorldY * viewState.scale;
}

// ==============================================================================
// 7. RENDER LOOP
// ==============================================================================
function renderLoop() {
  if (canvas && ctx) {
    drawCartesianScene();
  }
  requestAnimationFrame(renderLoop);
}

function drawCartesianScene() {
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.width / dpr;
  const height = canvas.height / dpr;

  ctx.save();
  ctx.scale(dpr, dpr);

  // Background
  ctx.fillStyle = '#090d16';
  ctx.fillRect(0, 0, width, height);

  // Dynamic Adaptive Cartesian Grid
  drawGrid(width, height);

  // Coordinate Axes (+X East, +Y North)
  drawAxes(width, height);

  // Base Station Pin at (0, 0)
  drawBaseStation();

  // Robot Breadcrumb Trail
  drawRobotTrail();

  // Selected Robot (Position, Heading Cone, Chassis)
  drawRobot();

  // Target Waypoint Pin & Trajectory Vector Line
  if (targetWaypoint.active) {
    drawTargetWaypoint();
  }

  // Crosshair Cursor Indicator
  drawCrosshairCursor();

  ctx.restore();
}

function drawGrid(width, height) {
  const scale = viewState.scale;

  // Determine grid intervals based on zoom level
  let minorStep = 1.0;
  let majorStep = 5.0;

  if (scale >= 60) {
    minorStep = 0.2; majorStep = 1.0;
  } else if (scale >= 25) {
    minorStep = 0.5; majorStep = 2.5;
  } else if (scale >= 12) {
    minorStep = 1.0; majorStep = 5.0;
  } else if (scale >= 5) {
    minorStep = 2.0; majorStep = 10.0;
  } else {
    minorStep = 5.0; majorStep = 25.0;
  }

  const topLeft = canvasToWorld(0, 0);
  const bottomRight = canvasToWorld(width, height);

  const startX = Math.floor(topLeft.x / minorStep) * minorStep;
  const endX = Math.ceil(bottomRight.x / minorStep) * minorStep;
  const startY = Math.floor(bottomRight.y / minorStep) * minorStep;
  const endY = Math.ceil(topLeft.y / minorStep) * minorStep;

  // 1. Minor Grid Lines
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#172033';
  ctx.beginPath();
  for (let x = startX; x <= endX; x += minorStep) {
    const scr = worldToCanvas(x, 0);
    ctx.moveTo(scr.x, 0);
    ctx.lineTo(scr.x, height);
  }
  for (let y = startY; y <= endY; y += minorStep) {
    const scr = worldToCanvas(0, y);
    ctx.moveTo(0, scr.y);
    ctx.lineTo(width, scr.y);
  }
  ctx.stroke();

  // 2. Major Grid Lines with Metric Labels
  ctx.lineWidth = 1.2;
  ctx.strokeStyle = '#22304d';
  ctx.fillStyle = '#64748b';
  ctx.font = '10px "JetBrains Mono", monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';

  for (let x = Math.floor(topLeft.x / majorStep) * majorStep; x <= endX; x += majorStep) {
    if (Math.abs(x) < 0.001) continue; // Skip 0 axis (drawn separately)
    const scr = worldToCanvas(x, 0);
    ctx.beginPath();
    ctx.moveTo(scr.x, 0);
    ctx.lineTo(scr.x, height);
    ctx.stroke();

    // Metric coordinate label
    const labelY = Math.max(16, Math.min(height - 24, worldToCanvas(0, 0).y + 6));
    ctx.fillText(`${x > 0 ? '+' : ''}${x.toFixed(0)}m`, scr.x, labelY);
  }

  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let y = Math.floor(bottomRight.y / majorStep) * majorStep; y <= endY; y += majorStep) {
    if (Math.abs(y) < 0.001) continue; // Skip 0 axis
    const scr = worldToCanvas(0, y);
    ctx.beginPath();
    ctx.moveTo(0, scr.y);
    ctx.lineTo(width, scr.y);
    ctx.stroke();

    const labelX = Math.max(34, Math.min(width - 16, worldToCanvas(0, 0).x - 6));
    ctx.fillText(`${y > 0 ? '+' : ''}${y.toFixed(0)}m`, labelX, scr.y);
  }
}

function drawAxes(width, height) {
  const origin = worldToCanvas(0, 0);

  // +X Axis (East) - Cyan
  ctx.lineWidth = 2.0;
  ctx.strokeStyle = 'rgba(2, 132, 199, 0.7)';
  ctx.beginPath();
  ctx.moveTo(0, origin.y);
  ctx.lineTo(width, origin.y);
  ctx.stroke();

  // +Y Axis (North) - Green
  ctx.strokeStyle = 'rgba(16, 185, 129, 0.7)';
  ctx.beginPath();
  ctx.moveTo(origin.x, 0);
  ctx.lineTo(origin.x, height);
  ctx.stroke();

  // Axis Labels & Arrowheads
  ctx.font = 'bold 11px "JetBrains Mono", monospace';

  // East Label (+X)
  ctx.fillStyle = '#38bdf8';
  ctx.textAlign = 'right';
  ctx.fillText('EAST (+X) →', width - 12, origin.y - 8);

  // North Label (+Y)
  ctx.fillStyle = '#34d399';
  ctx.textAlign = 'left';
  ctx.fillText('↑ NORTH (+Y)', origin.x + 8, 18);
}

function drawBaseStation() {
  const origin = worldToCanvas(0, 0);
  const now = Date.now() / 1000;

  // Radar Pulse Rings around Base (0, 0)
  const pulseRadius = (now % 2.5) * 16 * (viewState.scale / 22);
  const pulseAlpha = Math.max(0, 1.0 - (now % 2.5) / 2.5);

  ctx.beginPath();
  ctx.arc(origin.x, origin.y, Math.max(8, pulseRadius), 0, Math.PI * 2);
  ctx.strokeStyle = `rgba(16, 185, 129, ${pulseAlpha * 0.6})`;
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Base Station Marker (Golden/Emerald Beacon)
  ctx.beginPath();
  ctx.arc(origin.x, origin.y, 9, 0, Math.PI * 2);
  ctx.fillStyle = '#10b981';
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2.5;
  ctx.stroke();

  // Landmark Icon Badge
  ctx.font = 'bold 10px "JetBrains Mono", monospace';
  ctx.fillStyle = '#ffffff';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';

  // Base Label Tag
  const tagY = origin.y + 14;
  ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  ctx.fillRect(origin.x - 52, tagY - 2, 104, 18);
  ctx.strokeStyle = 'rgba(16, 185, 129, 0.5)';
  ctx.lineWidth = 1;
  ctx.strokeRect(origin.x - 52, tagY - 2, 104, 18);

  ctx.fillStyle = '#34d399';
  ctx.fillText('🏰 BASE (0, 0)', origin.x, tagY + 2);
}

function drawRobotTrail() {
  if (robotTelemetry.trail.length < 2) return;

  ctx.beginPath();
  const start = worldToCanvas(robotTelemetry.trail[0].x, robotTelemetry.trail[0].y);
  ctx.moveTo(start.x, start.y);

  for (let i = 1; i < robotTelemetry.trail.length; i++) {
    const pt = worldToCanvas(robotTelemetry.trail[i].x, robotTelemetry.trail[i].y);
    ctx.lineTo(pt.x, pt.y);
  }

  ctx.strokeStyle = 'rgba(56, 189, 248, 0.35)';
  ctx.lineWidth = 2.0;
  ctx.setLineDash([4, 4]);
  ctx.stroke();
  ctx.setLineDash([]);
}

function drawRobot() {
  const rScreen = worldToCanvas(robotTelemetry.pos.x, robotTelemetry.pos.y);
  const yawRad = robotTelemetry.headingRad; // Heading in ENU radians (0 = East, +CCW)

  // 1. Directional Heading Cone / Field of View
  const coneDist = 2.5 * viewState.scale; // 2.5 meters ahead
  const halfAngle = 0.35; // ~20 deg spread
  const leftAngle = yawRad - halfAngle;
  const rightAngle = yawRad + halfAngle;

  ctx.beginPath();
  ctx.moveTo(rScreen.x, rScreen.y);
  ctx.lineTo(rScreen.x + Math.cos(leftAngle) * coneDist, rScreen.y - Math.sin(leftAngle) * coneDist);
  ctx.arc(rScreen.x, rScreen.y, coneDist, -rightAngle, -leftAngle);
  ctx.closePath();
  ctx.fillStyle = 'rgba(2, 132, 199, 0.15)';
  ctx.fill();

  // 2. Heading Pointer Vector
  ctx.beginPath();
  ctx.moveTo(rScreen.x, rScreen.y);
  ctx.lineTo(rScreen.x + Math.cos(yawRad) * (coneDist * 0.9), rScreen.y - Math.sin(yawRad) * (coneDist * 0.9));
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2.5;
  ctx.stroke();

  // 3. Robot Chassis (Skid-steer dual tracks)
  ctx.save();
  ctx.translate(rScreen.x, rScreen.y);
  ctx.rotate(-yawRad); // Invert rotation for canvas space

  const chassisW = Math.max(14, 0.38 * viewState.scale); // ~38cm track width
  const chassisL = chassisW * 1.25;

  // Dual Treads
  ctx.fillStyle = '#334155';
  ctx.fillRect(-chassisL / 2, -chassisW / 2 - 4, chassisL, 4); // Left tread
  ctx.fillRect(-chassisL / 2, chassisW / 2, chassisL, 4);      // Right tread

  // Chassis Body
  ctx.fillStyle = '#0284c7';
  ctx.beginPath();
  ctx.roundRect(-chassisL / 2, -chassisW / 2, chassisL, chassisW, 4);
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2.0;
  ctx.stroke();

  // Forward Direction Notch
  ctx.fillStyle = '#fbbf24';
  ctx.beginPath();
  ctx.moveTo(chassisL / 2 - 2, -chassisW / 4);
  ctx.lineTo(chassisL / 2 + 4, 0);
  ctx.lineTo(chassisL / 2 - 2, chassisW / 4);
  ctx.closePath();
  ctx.fill();

  ctx.restore();

  // 4. Robot Label Pill
  const name = activeRobotId ? `🤖 R${activeRobotId < 10 ? '0' + activeRobotId : activeRobotId}` : '🤖 Robot';
  ctx.font = 'bold 11px "JetBrains Mono", monospace';
  ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  ctx.fillRect(rScreen.x - 38, rScreen.y - 32, 76, 18);
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1;
  ctx.strokeRect(rScreen.x - 38, rScreen.y - 32, 76, 18);

  ctx.fillStyle = '#f8fafc';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.fillText(name, rScreen.x, rScreen.y - 29);
}

function drawTargetWaypoint() {
  const rScreen = worldToCanvas(robotTelemetry.pos.x, robotTelemetry.pos.y);
  const tScreen = worldToCanvas(targetWaypoint.x, targetWaypoint.y);

  // 1. Dynamic Trajectory Line (Dashed Vector from Robot to Target)
  ctx.beginPath();
  ctx.moveTo(rScreen.x, rScreen.y);
  ctx.lineTo(tScreen.x, tScreen.y);
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 2.0;
  ctx.setLineDash([6, 4]);
  ctx.stroke();
  ctx.setLineDash([]);

  // 2. Trajectory Info Pill at Midpoint (Distance & Bearing)
  const midX = (rScreen.x + tScreen.x) / 2;
  const midY = (rScreen.y + tScreen.y) / 2;
  const dist = Math.hypot(targetWaypoint.x - robotTelemetry.pos.x, targetWaypoint.y - robotTelemetry.pos.y);

  ctx.fillStyle = 'rgba(15, 23, 42, 0.92)';
  ctx.fillRect(midX - 44, midY - 12, 88, 20);
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 1;
  ctx.strokeRect(midX - 44, midY - 12, 88, 20);

  ctx.font = 'bold 10px "JetBrains Mono", monospace';
  ctx.fillStyle = '#fbbf24';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(`📏 ${dist.toFixed(2)} m`, midX, midY - 1);

  // 3. Target Waypoint Pin with Concentric Target Rings
  const now = Date.now() / 1000;
  const pulse = Math.sin(now * 4) * 3;

  ctx.beginPath();
  ctx.arc(tScreen.x, tScreen.y, 14 + pulse, 0, Math.PI * 2);
  ctx.strokeStyle = 'rgba(245, 158, 11, 0.4)';
  ctx.lineWidth = 2;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(tScreen.x, tScreen.y, 8, 0, Math.PI * 2);
  ctx.fillStyle = '#f59e0b';
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2.5;
  ctx.stroke();

  // Target Crosshair
  ctx.beginPath();
  ctx.moveTo(tScreen.x - 12, tScreen.y);
  ctx.lineTo(tScreen.x + 12, tScreen.y);
  ctx.moveTo(tScreen.x, tScreen.y - 12);
  ctx.lineTo(tScreen.x, tScreen.y + 12);
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Target Coordinate Callout Tag
  const signX = targetWaypoint.x >= 0 ? '+' : '';
  const signY = targetWaypoint.y >= 0 ? '+' : '';
  const label = `🎯 (${signX}${targetWaypoint.x.toFixed(2)}m, ${signY}${targetWaypoint.y.toFixed(2)}m)`;

  ctx.font = 'bold 10px "JetBrains Mono", monospace';
  ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  ctx.fillRect(tScreen.x - 65, tScreen.y + 14, 130, 18);
  ctx.strokeStyle = 'rgba(245, 158, 11, 0.6)';
  ctx.lineWidth = 1;
  ctx.strokeRect(tScreen.x - 65, tScreen.y + 14, 130, 18);

  ctx.fillStyle = '#fcd34d';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.fillText(label, tScreen.x, tScreen.y + 17);
}

function drawCrosshairCursor() {
  if (viewState.isDragging) return;
  const x = viewState.mouseScreenX;
  const y = viewState.mouseScreenY;

  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.2)';
  ctx.setLineDash([3, 3]);

  ctx.beginPath();
  ctx.moveTo(x, 0); ctx.lineTo(x, canvas.height);
  ctx.moveTo(0, y); ctx.lineTo(canvas.width, y);
  ctx.stroke();

  ctx.setLineDash([]);
}
