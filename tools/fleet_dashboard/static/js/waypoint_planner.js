/* ==========================================================================
   ConeRobot 2D Cartesian Waypoint Planner
   Coordinate Origin: RTK Base Station (0, 0)
   +X = East in meters, +Y = North in meters
   Multi-Robot Fleet Visualization & Direct Single-Robot Dispatch
   ========================================================================== */

// --- Global Fleet State ---
let activeRobotId = null;
let activeRobotsList = {};
// Note: activeWebSockets, reconnectTimers and realTelemetry are already declared globally in state.js

// Per-robot breadcrumb trail history: { [id]: [{x, y}, ...] }
const robotTrails = {};

// Base Station Metric Origin (WGS84 Lat/Lon)
let baseStationGPS = {
  lat: null,
  lon: null,
  alt: 0.0,
  locked: false,
  source: 'Not Set'
};

// Selected Robot Telemetry View State
const robotTelemetry = {
  online: false,
  ip: '',
  port: 8765,
  state: 'IDLE',
  sensorMode: 'GPS',
  battery: null,
  headingDeg: 0.0,
  headingRad: 0.0,
  pos: {
    lat: null,
    lon: null,
    x: 0.0, // East in meters from Base (0, 0)
    y: 0.0, // North in meters from Base (0, 0)
  },
  odomPos: { x: 0.0, y: 0.0 },
  rtkStatus: 'Waiting...',
  diffAge: '--',
  satCount: 0,
};

// Global LiDAR point cloud display toggle
window.showLidarOnMap = true;

// Active Target Waypoint for the Selected Robot
let targetWaypoint = {
  active: false,
  x: 0.0, // East meters from Base (0, 0)
  y: 0.0, // North meters from Base (0, 0)
};

// Canvas Viewport Transformation
const viewState = {
  scale: 25,       // Pixels per meter
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
// 1. INITIALIZATION & BASE COORDINATE PERSISTENCE
// ==============================================================================
window.addEventListener('DOMContentLoaded', () => {
  if (typeof realTelemetry === 'undefined') {
    window.realTelemetry = {};
  }
  for (let i = 1; i <= 13; i++) {
    if (!realTelemetry[i] && typeof createEmptyRobotTelemetry === 'function') {
      realTelemetry[i] = createEmptyRobotTelemetry(i);
    }
    robotTrails[i] = [];
  }

  initCanvas();
  bindUIEvents();
  loadSavedBaseStation();
  fetchFleetList();
  setInterval(fetchFleetList, 4000);

  // Sync telemetry loop at 20 Hz
  setInterval(syncTelemetryFromSharedState, 50);

  logMission('Multi-Robot Cartesian Waypoint Planner initialized.', 'info');
  requestAnimationFrame(renderLoop);
});

function loadSavedBaseStation() {
  const saved = localStorage.getItem('conerobot_base_station');
  if (saved) {
    try {
      const parsed = JSON.parse(saved);
      if (parsed.lat && parsed.lon) {
        baseStationGPS.lat = parseFloat(parsed.lat);
        baseStationGPS.lon = parseFloat(parsed.lon);
        baseStationGPS.alt = parseFloat(parsed.alt || 0);
        baseStationGPS.locked = true;
        baseStationGPS.source = 'Saved (LocalStorage)';
        updateBaseStationUI();
        logMission(`Loaded saved Base Station: ${baseStationGPS.lat.toFixed(7)}°, ${baseStationGPS.lon.toFixed(7)}°`, 'info');
        return;
      }
    } catch (e) {}
  }

  fetch('/api/config')
    .then(r => r.json())
    .then(cfg => {
      if (cfg.base_station && cfg.base_station.lat) {
        baseStationGPS.lat = parseFloat(cfg.base_station.lat);
        baseStationGPS.lon = parseFloat(cfg.base_station.lon);
        baseStationGPS.alt = parseFloat(cfg.base_station.alt || 0);
        baseStationGPS.locked = true;
        baseStationGPS.source = 'fleet_config.json';
        updateBaseStationUI();
      } else {
        fetchBaseFromCaster(true);
      }
    }).catch(() => {
      fetchBaseFromCaster(true);
    });
}

function saveBaseStation(lat, lon, alt = 0.0, source = 'Manual') {
  baseStationGPS.lat = Number(lat);
  baseStationGPS.lon = Number(lon);
  baseStationGPS.alt = Number(alt);
  baseStationGPS.locked = true;
  baseStationGPS.source = source;

  localStorage.setItem('conerobot_base_station', JSON.stringify({
    lat: baseStationGPS.lat,
    lon: baseStationGPS.lon,
    alt: baseStationGPS.alt,
    source: source
  }));

  updateBaseStationUI();
  for (let i = 1; i <= 13; i++) {
    robotTrails[i] = [];
  }
  centerOnBase();
  logMission(`Base Station (0, 0) updated & locked to: ${baseStationGPS.lat.toFixed(8)}°, ${baseStationGPS.lon.toFixed(8)}°`, 'success');
}

function fetchBaseFromCaster(isSilent = false) {
  if (!isSilent) logMission('Querying RTK Base Station Caster (port 8080)...', 'info');

  fetch('/api/base_status')
    .then(r => r.json())
    .then(data => {
      if (data && data.latitude && data.longitude) {
        const bLat = parseFloat(data.latitude);
        const bLon = parseFloat(data.longitude);
        const bAlt = parseFloat(data.altitude || 0);
        const host = data.connected_host || 'Port 8080';
        saveBaseStation(bLat, bLon, bAlt, `Base Caster (${host})`);
      } else if (!isSilent) {
        logMission('Could not fetch from Caster on port 8080. Enter coordinates manually or click "Set Robot as Base".', 'warn');
      }
    }).catch(err => {
      if (!isSilent) logMission(`Base Caster query error: ${err.message}`, 'warn');
    });
}

function setRobotAsBase() {
  if (robotTelemetry.pos.lat === null || robotTelemetry.pos.lon === null) {
    alert('Selected robot has not reported a valid GPS fix yet!');
    return;
  }
  saveBaseStation(robotTelemetry.pos.lat, robotTelemetry.pos.lon, 0.0, `Robot ${activeRobotId} GPS Position`);
}

function bindUIEvents() {
  const robotSelect = document.getElementById('select-active-robot');
  robotSelect.addEventListener('change', (e) => {
    const id = parseInt(e.target.value, 10);
    if (id) selectRobot(id);
  });

  // Canvas View Controls
  document.getElementById('btn-zoom-in').addEventListener('click', () => zoomView(1.25));
  document.getElementById('btn-zoom-out').addEventListener('click', () => zoomView(0.80));

  const btnToggleLidar = document.getElementById('btn-toggle-lidar');
  if (btnToggleLidar) {
    btnToggleLidar.addEventListener('click', () => {
      window.showLidarOnMap = !window.showLidarOnMap;
      btnToggleLidar.textContent = window.showLidarOnMap ? '📡 LiDAR: ON' : '📡 LiDAR: OFF';
      btnToggleLidar.className = window.showLidarOnMap ? 'canvas-btn active' : 'canvas-btn';
      logMission(`LiDAR obstacle points on map: ${window.showLidarOnMap ? 'ENABLED' : 'DISABLED'}`, 'info');
    });
  }

  document.getElementById('btn-center-base').addEventListener('click', centerOnBase);
  document.getElementById('btn-center-robot').addEventListener('click', centerOnRobot);
  document.getElementById('btn-fit-view').addEventListener('click', fitAll);

  // Waypoint Action Buttons
  document.getElementById('btn-dispatch-waypoint').addEventListener('click', dispatchWaypoint);
  document.getElementById('btn-return-home').addEventListener('click', returnToBase);
  document.getElementById('btn-estop').addEventListener('click', triggerEmergencyStop);
  document.getElementById('btn-clear-target').addEventListener('click', clearTarget);
  document.getElementById('btn-set-manual').addEventListener('click', applyManualCoordinates);

  // Base Station Controls
  document.getElementById('btn-save-base').addEventListener('click', () => {
    const lat = parseFloat(document.getElementById('base-input-lat').value);
    const lon = parseFloat(document.getElementById('base-input-lon').value);
    if (!isNaN(lat) && !isNaN(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180) {
      saveBaseStation(lat, lon, 0.0, 'Manual User Input');
    } else {
      alert('Please enter valid Latitude [-90..90] and Longitude [-180..180].');
    }
  });

  document.getElementById('btn-fetch-caster').addEventListener('click', () => fetchBaseFromCaster(false));
  document.getElementById('btn-set-robot-as-base').addEventListener('click', setRobotAsBase);

  // Network Scan
  document.getElementById('btn-scan-network').addEventListener('click', () => {
    fetch('/api/scan')
      .then(r => r.json())
      .then(() => {
        logMission('Scanning Wi-Fi network for active ConeRobots...', 'info');
        setTimeout(fetchFleetList, 1500);
      });
  });

  // Coordinate Inputs
  document.getElementById('input-target-x').addEventListener('change', updateTargetFromInputs);
  document.getElementById('input-target-y').addEventListener('change', updateTargetFromInputs);
}

// ==============================================================================
// 2. FLEET DISCOVERY & MULTI-ROBOT WEBSOCKET ENGINE
// ==============================================================================
async function fetchFleetList() {
  try {
    const res = await fetch('/api/fleet');
    if (!res.ok) return;
    const data = await res.json();
    if (data.base_station && typeof updateBaseStationLink === 'function') {
      updateBaseStationLink(data.base_station);
    }
    const robots = data.robots || {};
    activeRobotsList = robots;

    const select = document.getElementById('select-active-robot');
    const currentVal = select.value;

    let html = '';
    let onlineCount = 0;
    let firstOnlineId = null;

    for (let id = 1; id <= 13; id++) {
      const r = robots[id];
      if (r && r.online && r.ip) {
        onlineCount++;
        if (!firstOnlineId) firstOnlineId = id;
        html += `<option value="${id}">🤖 ${r.name || `ConeRobot ${id < 10 ? '0' + id : id}`} (${r.ip}) [ONLINE]</option>`;

        // Connect WebSocket to each online robot in parallel
        if (!activeWebSockets[id] && !reconnectTimers[id]) {
          connectRobotWebSocket(id, r.ip, r.port || 8765);
        }
      } else {
        if (r) {
          html += `<option value="${id}" style="color: #64748b;">ConeRobot ${id < 10 ? '0' + id : id} [Offline]</option>`;
        }
        if (activeWebSockets[id]) {
          try { activeWebSockets[id].close(); } catch (e) {}
          delete activeWebSockets[id];
        }
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
  activeRobotId = id;
  const select = document.getElementById('select-active-robot');
  if (select && select.value !== String(id)) {
    select.value = id;
  }

  const r = activeRobotsList[id];
  const isOnline = Boolean(r && r.online && activeWebSockets[id] && activeWebSockets[id].readyState === WebSocket.OPEN);
  updateConnBadge(isOnline, r?.ip || '');

  syncTelemetryFromSharedState();
  updateTargetMetrics();
  logMission(`Selected Robot ${id < 10 ? '0' + id : id} for control.`, 'info');
}

function connectRobotWebSocket(id, ip, port) {
  const wsUrl = `ws://${ip}:${port}`;
  let ws;
  try {
    ws = new WebSocket(wsUrl, ["foxglove.sdk.v1", "foxglove.websocket.v1"]);
  } catch (e) {
    try { ws = new WebSocket(wsUrl); } catch (err) {
      scheduleReconnect(id, ip, port);
      return;
    }
  }

  ws.binaryType = "arraybuffer";
  activeWebSockets[id] = ws;

  const subIdToTopic = {};
  const channelIdToTopic = {};
  let subCounter = 1;

  ws.onopen = () => {
    logMission(`🟢 Connected to Robot ${id < 10 ? '0' + id : id} (${ip}:${port})!`, 'success');
    if (activeRobotId === id) updateConnBadge(true, ip);

    if (!realTelemetry[id]) {
      realTelemetry[id] = createEmptyRobotTelemetry(id);
    }
    realTelemetry[id].online = true;
    realTelemetry[id].ip = ip;

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
    } catch (e) {}
  };

  ws.onmessage = (event) => {
    if (typeof event.data === "string") {
      try {
        const msg = JSON.parse(event.data);
        if (msg.op === "advertise" && Array.isArray(msg.channels)) {
          const subscriptions = [];
          const targetTopics = [
            '/scan',
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
        }
      } catch (e) {}
    } else if (event.data instanceof ArrayBuffer) {
      if (typeof handleBinaryFoxgloveMessage === 'function') {
        handleBinaryFoxgloveMessage(id, event.data, subIdToTopic, channelIdToTopic);
      }
    }
  };

  ws.onerror = () => {};

  ws.onclose = () => {
    delete activeWebSockets[id];
    if (realTelemetry[id]) realTelemetry[id].online = false;
    if (activeRobotId === id) updateConnBadge(false);
    scheduleReconnect(id, ip, port);
  };
}

function scheduleReconnect(id, ip, port) {
  if (reconnectTimers[id]) return;
  reconnectTimers[id] = setTimeout(() => {
    delete reconnectTimers[id];
    if (activeRobotsList[id] && activeRobotsList[id].online) {
      connectRobotWebSocket(id, ip, port);
    }
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
// 3. SYNCHRONIZE TELEMETRY & MULTI-ROBOT METRIC PROJECTION
// ==============================================================================
function syncTelemetryFromSharedState() {
  if (baseStationGPS.lat === null || baseStationGPS.lon === null) {
    // If not set, check if any robot received /gps/base_fix
    for (let id = 1; id <= 13; id++) {
      const r = realTelemetry[id];
      if (r && r.base_gps && r.base_gps.lat) {
        saveBaseStation(r.base_gps.lat, r.base_gps.lon, r.base_gps.alt || 0, `Rover R${id} RTCM 1005`);
        break;
      }
    }
    // Or lock to first available rover GPS fix
    if (baseStationGPS.lat === null) {
      for (let id = 1; id <= 13; id++) {
        const r = realTelemetry[id];
        if (r && r.gps && r.gps.lat && Math.abs(r.gps.lat) > 0.001) {
          saveBaseStation(r.gps.lat, r.gps.lon, 0.0, `Auto (Robot R${id} First Fix)`);
          break;
        }
      }
    }
  }

  // Update Cartesian (X, Y) for ALL robots
  for (let id = 1; id <= 13; id++) {
    const r = realTelemetry[id];
    if (!r || !r.online) continue;

    const lat = r.gps?.lat;
    const lon = r.gps?.lon;

    if (lat && lon && Math.abs(lat) > 0.001 && Math.abs(lon) > 0.001 && baseStationGPS.lat !== null) {
      const enu = latLonToEnuMeters(lat, lon, baseStationGPS.lat, baseStationGPS.lon);
      r.metricX = enu.x;
      r.metricY = enu.y;

      // Update breadcrumb trail for this robot
      if (!robotTrails[id]) robotTrails[id] = [];
      const trail = robotTrails[id];
      const last = trail[trail.length - 1];
      if (!last || Math.hypot(enu.x - last.x, enu.y - last.y) >= 0.15) {
        trail.push({ x: enu.x, y: enu.y });
        if (trail.length > 300) trail.shift();
      }
    }
  }

  // Sync selected active robot view
  if (activeRobotId && realTelemetry[activeRobotId]) {
    const sel = realTelemetry[activeRobotId];
    robotTelemetry.online = sel.online;
    robotTelemetry.ip = sel.ip;
    robotTelemetry.pos.lat = sel.gps?.lat;
    robotTelemetry.pos.lon = sel.gps?.lon;
    robotTelemetry.pos.x = sel.metricX || 0.0;
    robotTelemetry.pos.y = sel.metricY || 0.0;

    if (sel.imu && sel.imu.heading !== null && !isNaN(sel.imu.heading)) {
      robotTelemetry.headingDeg = sel.imu.heading;
      robotTelemetry.headingRad = (sel.imu.heading * Math.PI) / 180.0;
    }

    robotTelemetry.rtkStatus = sel.gps?.mode || 'NO FIX';
    robotTelemetry.diffAge = sel.gps?.diff_age || '--';
    robotTelemetry.satCount = sel.gps?.sats || 0;
    robotTelemetry.battery = sel.health?.battery_v || null;

    if (sel.gps?.mode === 'RTK FIX') robotTelemetry.sensorMode = 'GPS (RTK FIX)';
    else if (sel.gps?.mode === 'FLOAT') robotTelemetry.sensorMode = 'GPS (RTK FLOAT)';
    else if (sel.gps?.mode) robotTelemetry.sensorMode = `GPS (${sel.gps.mode})`;

    updateTelemetryUI();
    updateTargetMetrics();
  }
}

function latLonToEnuMeters(lat, lon, refLat, refLon) {
  const R = 6371000.0;
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
  const signX = robotTelemetry.pos.x >= 0 ? '+' : '';
  const signY = robotTelemetry.pos.y >= 0 ? '+' : '';
  document.getElementById('tel-base-pos').textContent = `X: ${signX}${robotTelemetry.pos.x.toFixed(2)}m, Y: ${signY}${robotTelemetry.pos.y.toFixed(2)}m`;
  document.getElementById('tel-sensor-mode').textContent = robotTelemetry.sensorMode;

  const rtkEl = document.getElementById('tel-rtk-status');
  rtkEl.textContent = `${robotTelemetry.rtkStatus} (${robotTelemetry.satCount} Sats)`;
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
  if (baseStationGPS.lat !== null && baseStationGPS.lon !== null) {
    document.getElementById('header-base-coords').textContent = `${baseStationGPS.lat.toFixed(6)}°, ${baseStationGPS.lon.toFixed(6)}°`;
    document.getElementById('base-input-lat').value = baseStationGPS.lat.toFixed(7);
    document.getElementById('base-input-lon').value = baseStationGPS.lon.toFixed(7);
    document.getElementById('base-ref-source').textContent = baseStationGPS.source;
    document.getElementById('base-lock-badge').className = 'state-badge idle';
    document.getElementById('base-lock-badge').textContent = 'Locked (0, 0)';
  } else {
    document.getElementById('header-base-coords').textContent = 'Not Set (Click Fetch :8080)';
    document.getElementById('base-lock-badge').className = 'state-badge error';
    document.getElementById('base-lock-badge').textContent = 'Unset';
  }
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
  const z = 0.0;

  logMission(`🚀 Dispatching Robot ${activeRobotId} -> Target: (X: ${x.toFixed(2)}m, Y: ${y.toFixed(2)}m)...`, 'info');

  let sentViaWs = false;
  const ws = activeWebSockets[activeRobotId];
  if (ws && ws.readyState === WebSocket.OPEN) {
    try {
      const cdrBuf = encodeVector3CDR(CLIENT_CHANNELS.CMD_WAYPOINT, x, y, z);
      ws.send(cdrBuf);
      sentViaWs = true;
      logMission(`✔ Sent /cmd_waypoint to Robot ${activeRobotId} via WebSocket!`, 'success');
    } catch (err) {
      console.warn('WebSocket send error:', err);
    }
  }

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
      if (!sentViaWs) logMission(`✔ Dispatched via HTTP Backend Relay!`, 'success');
    }).catch(err => {
      if (!sentViaWs) logMission(`✖ Failed to send command: ${err.message}`, 'error');
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
  logMission(`🛑 EMERGENCY STOP TRIGGERED! Halting selected robot...`, 'error');

  const ws = activeWebSockets[activeRobotId];
  if (ws && ws.readyState === WebSocket.OPEN) {
    try {
      const stopBuf = encodeVector3CDR(CLIENT_CHANNELS.CMD_PRIMITIVE, 0.0, 0.0, 0.0);
      ws.send(stopBuf);
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
  logMission(`Applied coordinates: (X: ${targetWaypoint.x.toFixed(2)}m, Y: ${targetWaypoint.y.toFixed(2)}m)`, 'info');
}

// ==============================================================================
// 6. HIGH-PERFORMANCE 2D CANVAS CARTESIAN ENGINE
// ==============================================================================
let canvas, ctx;

function initCanvas() {
  canvas = document.getElementById('waypoint-canvas');
  if (!canvas) {
    console.error('Cannot find #waypoint-canvas element!');
    return;
  }
  ctx = canvas.getContext('2d');

  function resizeCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const w = rect.width || canvas.parentElement?.clientWidth || (window.innerWidth - 420);
    const h = rect.height || canvas.parentElement?.clientHeight || (window.innerHeight - 100);
    canvas.width = Math.max(300, Math.floor(w)) * dpr;
    canvas.height = Math.max(300, Math.floor(h)) * dpr;
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

    const world = canvasToWorld(screenX, screenY);
    viewState.mouseWorldX = world.x;
    viewState.mouseWorldY = world.y;

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

    const rect = canvas.getBoundingClientRect();
    if (
      e.clientX >= rect.left && e.clientX <= rect.right &&
      e.clientY >= rect.top && e.clientY <= rect.bottom
    ) {
      const dt = Date.now() - viewState.downTime;
      const dragDist = Math.hypot(e.clientX - viewState.dragStartX, e.clientY - viewState.dragStartY);

      if (dt < 400 && dragDist < 6) {
        const screenX = e.clientX - rect.left;
        const screenY = e.clientY - rect.top;

        // 1. Check if user clicked directly on any robot to SELECT it!
        let clickedRobotId = null;
        for (let id = 1; id <= 13; id++) {
          const r = realTelemetry[id];
          if (r && r.online && r.metricX !== undefined) {
            const rScreen = worldToCanvas(r.metricX, r.metricY);
            const clickDist = Math.hypot(screenX - rScreen.x, screenY - rScreen.y);
            if (clickDist <= 22) {
              clickedRobotId = id;
              break;
            }
          }
        }

        if (clickedRobotId) {
          selectRobot(clickedRobotId);
          return;
        }

        // 2. Otherwise, set the Target Waypoint for the currently selected robot
        const world = canvasToWorld(screenX, screenY);
        targetWaypoint.x = Math.round(world.x * 100) / 100;
        targetWaypoint.y = Math.round(world.y * 100) / 100;
        targetWaypoint.active = true;

        document.getElementById('input-target-x').value = targetWaypoint.x.toFixed(2);
        document.getElementById('input-target-y').value = targetWaypoint.y.toFixed(2);
        updateTargetMetrics();

        logMission(`Target placed at (X: ${targetWaypoint.x.toFixed(2)}m, Y: ${targetWaypoint.y.toFixed(2)}m).`, 'info');
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

function worldToCanvas(wx, wy) {
  const dpr = window.devicePixelRatio || 1;
  const cx = (canvas.width / dpr) / 2 + viewState.panX;
  const cy = (canvas.height / dpr) / 2 + viewState.panY;

  return {
    x: cx + wx * viewState.scale,
    y: cy - wy * viewState.scale
  };
}

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
  const newScale = Math.max(2.0, Math.min(300.0, oldScale * factor));
  if (oldScale === newScale) return;

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

  // Include ALL online robots
  for (let id = 1; id <= 13; id++) {
    const r = realTelemetry[id];
    if (r && r.online && r.metricX !== undefined) {
      points.push({ x: r.metricX, y: r.metricY });
    }
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

  minX -= 4; maxX += 4;
  minY -= 4; maxY += 4;

  const dpr = window.devicePixelRatio || 1;
  const cw = (canvas.width / dpr);
  const ch = (canvas.height / dpr);

  const spanX = Math.max(8, maxX - minX);
  const spanY = Math.max(8, maxY - minY);

  const calcScale = Math.min(cw / spanX, ch / spanY);
  if (!isNaN(calcScale) && isFinite(calcScale) && calcScale > 1.0) {
    viewState.scale = Math.max(2.0, Math.min(250.0, calcScale));
  }
  const midWorldX = (minX + maxX) / 2;
  const midWorldY = (minY + maxY) / 2;

  viewState.panX = -midWorldX * viewState.scale;
  viewState.panY = midWorldY * viewState.scale;
}

// ==============================================================================
// 7. RENDER LOOP (DRAWS BASE, ALL ONLINE ROBOTS, TARGET)
// ==============================================================================
function renderLoop() {
  requestAnimationFrame(renderLoop);
  if (canvas && ctx) {
    try {
      drawCartesianScene();
    } catch (err) {
      console.error('[RENDER ERROR]', err);
    }
  }
}

function drawCartesianScene() {
  if (!canvas || !ctx) return;
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.width / dpr;
  const height = canvas.height / dpr;
  if (width < 10 || height < 10) return;

  ctx.save();
  ctx.scale(dpr, dpr);

  ctx.fillStyle = '#090d16';
  ctx.fillRect(0, 0, width, height);

  // Dynamic Cartesian Grid
  drawGrid(width, height);

  // Coordinate Axes (+X East, +Y North)
  drawAxes(width, height);

  // Base Station Pin at (0, 0)
  drawBaseStation();

  // Draw 2D LiDAR Obstacle Points for all online robots on the field
  drawAllRobotLidarPoints();

  // Draw Trails for All Robots
  drawAllRobotTrails();

  // Draw ALL Online Robots on the Field
  drawAllRobots();

  // Target Waypoint Pin & Vector Line (for Active Robot)
  if (targetWaypoint.active) {
    drawTargetWaypoint();
  }

  // Crosshair Cursor Indicator
  drawCrosshairCursor();

  // Warning Banner if Base Station is far off
  const distFromBase = Math.hypot(robotTelemetry.pos.x, robotTelemetry.pos.y);
  if (distFromBase > 5000 && robotTelemetry.pos.lat !== null) {
    ctx.fillStyle = 'rgba(239, 68, 68, 0.9)';
    ctx.fillRect(width / 2 - 220, 16, 440, 26);
    ctx.font = 'bold 11px "Outfit", sans-serif';
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('⚠️ Base Station differs by > 5km! Click "Fetch :8080" or "Set Robot as Base"', width / 2, 29);
  }

  ctx.restore();
}

function drawGrid(width, height) {
  const scale = viewState.scale;

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

  // 1. Minor Lines
  ctx.lineWidth = 1;
  ctx.strokeStyle = '#172033';
  ctx.beginPath();
  for (let x = startX; x <= endX; x += minorStep) {
    const scr = worldToCanvas(x, 0);
    ctx.moveTo(scr.x, 0); ctx.lineTo(scr.x, height);
  }
  for (let y = startY; y <= endY; y += minorStep) {
    const scr = worldToCanvas(0, y);
    ctx.moveTo(0, scr.y); ctx.lineTo(width, scr.y);
  }
  ctx.stroke();

  // 2. Major Lines with Numbers
  ctx.lineWidth = 1.2;
  ctx.strokeStyle = '#22304d';
  ctx.fillStyle = '#64748b';
  ctx.font = '10px "JetBrains Mono", monospace';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';

  for (let x = Math.floor(topLeft.x / majorStep) * majorStep; x <= endX; x += majorStep) {
    if (Math.abs(x) < 0.001) continue;
    const scr = worldToCanvas(x, 0);
    ctx.beginPath();
    ctx.moveTo(scr.x, 0); ctx.lineTo(scr.x, height);
    ctx.stroke();

    const labelY = Math.max(16, Math.min(height - 24, worldToCanvas(0, 0).y + 6));
    ctx.fillText(`${x > 0 ? '+' : ''}${x.toFixed(0)}m`, scr.x, labelY);
  }

  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (let y = Math.floor(bottomRight.y / majorStep) * majorStep; y <= endY; y += majorStep) {
    if (Math.abs(y) < 0.001) continue;
    const scr = worldToCanvas(0, y);
    ctx.beginPath();
    ctx.moveTo(0, scr.y); ctx.lineTo(width, scr.y);
    ctx.stroke();

    const labelX = Math.max(34, Math.min(width - 16, worldToCanvas(0, 0).x - 6));
    ctx.fillText(`${y > 0 ? '+' : ''}${y.toFixed(0)}m`, labelX, scr.y);
  }
}

function drawAxes(width, height) {
  const origin = worldToCanvas(0, 0);

  ctx.lineWidth = 2.0;
  ctx.strokeStyle = 'rgba(2, 132, 199, 0.7)';
  ctx.beginPath();
  ctx.moveTo(0, origin.y); ctx.lineTo(width, origin.y);
  ctx.stroke();

  ctx.strokeStyle = 'rgba(16, 185, 129, 0.7)';
  ctx.beginPath();
  ctx.moveTo(origin.x, 0); ctx.lineTo(origin.x, height);
  ctx.stroke();

  ctx.font = 'bold 11px "JetBrains Mono", monospace';

  ctx.fillStyle = '#38bdf8';
  ctx.textAlign = 'right';
  ctx.fillText('EAST (+X) →', width - 12, origin.y - 8);

  ctx.fillStyle = '#34d399';
  ctx.textAlign = 'left';
  ctx.fillText('↑ NORTH (+Y)', origin.x + 8, 18);
}

function drawBaseStation() {
  const origin = worldToCanvas(0, 0);
  const now = Date.now() / 1000;

  const pulseRadius = (now % 2.5) * 16 * (viewState.scale / 25);
  const pulseAlpha = Math.max(0, 1.0 - (now % 2.5) / 2.5);

  ctx.beginPath();
  ctx.arc(origin.x, origin.y, Math.max(8, pulseRadius), 0, Math.PI * 2);
  ctx.strokeStyle = `rgba(16, 185, 129, ${pulseAlpha * 0.6})`;
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(origin.x, origin.y, 9, 0, Math.PI * 2);
  ctx.fillStyle = '#10b981';
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 2.5;
  ctx.stroke();

  const tagY = origin.y + 14;
  ctx.fillStyle = 'rgba(15, 23, 42, 0.9)';
  ctx.fillRect(origin.x - 52, tagY - 2, 104, 18);
  ctx.strokeStyle = 'rgba(16, 185, 129, 0.5)';
  ctx.lineWidth = 1;
  ctx.strokeRect(origin.x - 52, tagY - 2, 104, 18);

  ctx.font = 'bold 10px "JetBrains Mono", monospace';
  ctx.fillStyle = '#34d399';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  ctx.fillText('🏰 BASE (0, 0)', origin.x, tagY + 2);
}

function drawAllRobotLidarPoints() {
  if (!window.showLidarOnMap) return;

  const now = Date.now();
  const pointSize = Math.max(2.0, Math.min(4.5, 0.09 * viewState.scale));

  for (let id = 1; id <= 13; id++) {
    const r = realTelemetry[id];
    if (!r || !r.online || r.metricX === undefined || !r.lidar || !r.lidar.points || r.lidar.points.length === 0) {
      continue;
    }

    // Only draw fresh scans (within last 3.5s)
    if (now - (r.lidar.last_scan_time || 0) > 3500) continue;

    const isSelected = (id === activeRobotId);
    const rx = r.metricX;
    const ry = r.metricY;

    const headingDeg = r.imu?.heading || 0.0;
    const yawRad = (headingDeg * Math.PI) / 180.0;
    const cosYaw = Math.cos(yawRad);
    const sinYaw = Math.sin(yawRad);

    const pts = r.lidar.points;
    const len = pts.length;

    let ptColor = '#38bdf8'; // Selected robot: bright cyan
    if (!isSelected) {
      const palette = ['#34d399', '#a78bfa', '#f472b6', '#fbbf24', '#4ade80'];
      ptColor = palette[(id - 1) % palette.length];
    }
    ctx.fillStyle = ptColor;

    for (let i = 0; i < len; i++) {
      const p = pts[i];
      const lx = p[0];
      const ly = p[1];

      // Body frame (lx forward, ly left) to global ENU (+X East, +Y North)
      const gx = rx + (lx * cosYaw - ly * sinYaw);
      const gy = ry + (lx * sinYaw + ly * cosYaw);

      const scr = worldToCanvas(gx, gy);

      if (scr.x < -10 || scr.x > canvas.width || scr.y < -10 || scr.y > canvas.height) {
        continue;
      }

      ctx.fillRect(scr.x - pointSize / 2, scr.y - pointSize / 2, pointSize, pointSize);
    }
  }
}

function drawAllRobotTrails() {
  for (let id = 1; id <= 13; id++) {
    const trail = robotTrails[id];
    if (!trail || trail.length < 2) continue;

    const isSelected = (id === activeRobotId);
    ctx.beginPath();
    const start = worldToCanvas(trail[0].x, trail[0].y);
    ctx.moveTo(start.x, start.y);

    for (let i = 1; i < trail.length; i++) {
      const pt = worldToCanvas(trail[i].x, trail[i].y);
      ctx.lineTo(pt.x, pt.y);
    }

    ctx.strokeStyle = isSelected ? 'rgba(56, 189, 248, 0.45)' : 'rgba(148, 163, 184, 0.2)';
    ctx.lineWidth = isSelected ? 2.0 : 1.2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);
  }
}

function drawAllRobots() {
  for (let id = 1; id <= 13; id++) {
    const r = realTelemetry[id];
    if (!r || !r.online || r.metricX === undefined) continue;

    const isSelected = (id === activeRobotId);
    const rScreen = worldToCanvas(r.metricX, r.metricY);

    const headingDeg = r.imu?.heading || 0.0;
    const yawRad = (headingDeg * Math.PI) / 180.0;

    const mode = r.gps?.mode || 'NO FIX';
    let statusColor = '#10b981'; // Green for RTK FIX
    if (mode === 'FLOAT' || mode === '3D FIX') statusColor = '#f59e0b';
    else if (mode === 'NO FIX') statusColor = '#ef4444';

    // 1. Heading Cone (Rendered for selected robot)
    if (isSelected) {
      const coneDist = 2.5 * viewState.scale;
      const halfAngle = 0.35;
      const leftAngle = yawRad - halfAngle;
      const rightAngle = yawRad + halfAngle;

      ctx.beginPath();
      ctx.moveTo(rScreen.x, rScreen.y);
      ctx.lineTo(rScreen.x + Math.cos(leftAngle) * coneDist, rScreen.y - Math.sin(leftAngle) * coneDist);
      ctx.arc(rScreen.x, rScreen.y, coneDist, -rightAngle, -leftAngle);
      ctx.closePath();
      ctx.fillStyle = 'rgba(2, 132, 199, 0.16)';
      ctx.fill();

      // Heading Vector Line
      ctx.beginPath();
      ctx.moveTo(rScreen.x, rScreen.y);
      ctx.lineTo(rScreen.x + Math.cos(yawRad) * (coneDist * 0.9), rScreen.y - Math.sin(yawRad) * (coneDist * 0.9));
      ctx.strokeStyle = '#38bdf8';
      ctx.lineWidth = 2.5;
      ctx.stroke();
    } else {
      // Shorter heading arrow for unselected robots
      const arrowLen = 1.2 * viewState.scale;
      ctx.beginPath();
      ctx.moveTo(rScreen.x, rScreen.y);
      ctx.lineTo(rScreen.x + Math.cos(yawRad) * arrowLen, rScreen.y - Math.sin(yawRad) * arrowLen);
      ctx.strokeStyle = statusColor;
      ctx.lineWidth = 1.8;
      ctx.stroke();
    }

    // 2. Robot Chassis Body
    ctx.save();
    ctx.translate(rScreen.x, rScreen.y);
    ctx.rotate(-yawRad);

    const chassisW = Math.max(12, 0.38 * viewState.scale);
    const chassisL = chassisW * 1.25;

    // Dual Tracks
    ctx.fillStyle = isSelected ? '#1e293b' : '#334155';
    ctx.fillRect(-chassisL / 2, -chassisW / 2 - 3, chassisL, 3);
    ctx.fillRect(-chassisL / 2, chassisW / 2, chassisL, 3);

    // Chassis
    ctx.fillStyle = isSelected ? '#0284c7' : '#1e293b';
    ctx.beginPath();
    ctx.roundRect(-chassisL / 2, -chassisW / 2, chassisL, chassisW, 3);
    ctx.fill();

    ctx.strokeStyle = isSelected ? '#ffffff' : statusColor;
    ctx.lineWidth = isSelected ? 2.2 : 1.5;
    ctx.stroke();

    // Direction Notch
    ctx.fillStyle = isSelected ? '#fbbf24' : statusColor;
    ctx.beginPath();
    ctx.moveTo(chassisL / 2 - 2, -chassisW / 4);
    ctx.lineTo(chassisL / 2 + 3, 0);
    ctx.lineTo(chassisL / 2 - 2, chassisW / 4);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    // 3. Robot Name Label Tag
    const rNum = id < 10 ? '0' + id : id;
    const labelText = isSelected ? `🤖 R${rNum} [SEL]` : `R${rNum}`;

    ctx.font = 'bold 10px "JetBrains Mono", monospace';
    ctx.fillStyle = isSelected ? 'rgba(2, 132, 199, 0.95)' : 'rgba(15, 23, 42, 0.9)';
    ctx.fillRect(rScreen.x - 30, rScreen.y - 28, 60, 16);
    ctx.strokeStyle = isSelected ? '#ffffff' : statusColor;
    ctx.lineWidth = 1;
    ctx.strokeRect(rScreen.x - 30, rScreen.y - 28, 60, 16);

    ctx.fillStyle = '#f8fafc';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(labelText, rScreen.x, rScreen.y - 20);
  }
}

function drawTargetWaypoint() {
  const rScreen = worldToCanvas(robotTelemetry.pos.x, robotTelemetry.pos.y);
  const tScreen = worldToCanvas(targetWaypoint.x, targetWaypoint.y);

  // 1. Dynamic Trajectory Line from Selected Robot to Target
  ctx.beginPath();
  ctx.moveTo(rScreen.x, rScreen.y);
  ctx.lineTo(tScreen.x, tScreen.y);
  ctx.strokeStyle = '#f59e0b';
  ctx.lineWidth = 2.0;
  ctx.setLineDash([6, 4]);
  ctx.stroke();
  ctx.setLineDash([]);

  // 2. Info Pill at Midpoint
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

  // 3. Target Waypoint Pin
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

  // Crosshair
  ctx.beginPath();
  ctx.moveTo(tScreen.x - 12, tScreen.y);
  ctx.lineTo(tScreen.x + 12, tScreen.y);
  ctx.moveTo(tScreen.x, tScreen.y - 12);
  ctx.lineTo(tScreen.x, tScreen.y + 12);
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  // Target Callout Tag
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
