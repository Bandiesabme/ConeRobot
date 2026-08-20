/* ==========================================================================
   ConeRobot 13-Fleet Mission Control - Ultra Clean Architecture
   100% Real Live Sensor Data - Zero Fake Data - Heterogeneous Fleet (LiDAR & GPS)
   ========================================================================== */

let map = null;
let robotMarkers = {};
let selectedRobotId = null;
let activeWebSockets = {}; // { 1: WebSocket }
let reconnectTimers = {};  // { 1: timerId }
let realTelemetry = {};    // { 1: { ... } }

let configuredTopics = {
  scan: '/scan',
  gps_fix: '/fix',
  gps_status: '/gps/status',
  heading: '/imu/heading',
  imu_data: '/imu/data',
  step_status: '/step_status',
  battery: '/battery_state'
};

document.addEventListener('DOMContentLoaded', () => {
  initMap();
  initCardsGridDOM();
  initEventListeners();
  startDiscoveryLoop();
  
  // Smooth, throttled UI update loop (10 FPS)
  setInterval(renderUIUpdates, 100);
});

// --- Debug Logger ---
function logDebug(msg, level = 'info') {
  const container = document.getElementById('debug-log-container');
  if (!container) return;
  const timeStr = new Date().toLocaleTimeString();
  let color = '#94a3b8';
  if (level === 'success') color = '#34d399';
  if (level === 'warn') color = '#f59e0b';
  if (level === 'error') color = '#f87171';
  if (level === 'topic') color = '#38bdf8';

  const line = document.createElement('div');
  line.style.color = color;
  line.innerHTML = `<span style="color: #64748b;">[${timeStr}]</span> ${msg}`;
  container.appendChild(line);
  container.scrollTop = container.scrollHeight;
}

// --- 1. Map Initialization ---
function initMap() {
  map = L.map('fleet-map', {
    center: [47.4979, 19.0402],
    zoom: 16,
    zoomControl: true
  });

  L.tileLayer('https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png', {
    maxZoom: 22,
    subdomains: 'abcd',
    attribution: '&copy; <a href="https://carto.com/">CARTO</a> &copy; OpenStreetMap'
  }).addTo(map);

  document.getElementById('btn-fit-map').addEventListener('click', fitAllRobotsInMap);
}

function createRobotIcon(robotId, headingDeg, gpsMode, isLidarOnly) {
  const rNum = String(robotId).padStart(2, '0');
  let strokeColor = '#10b981'; // Green for RTK FIX
  if (isLidarOnly) strokeColor = '#3b82f6'; // Blue for LiDAR unit
  else if (gpsMode === 'FLOAT' || gpsMode === '3D FIX') strokeColor = '#f59e0b';
  else if (gpsMode === 'NO FIX' || !gpsMode) strokeColor = '#ef4444';

  const rot = (headingDeg !== null && !isNaN(headingDeg)) ? headingDeg : 0;

  const svgHtml = `
    <div class="robot-map-pin" style="transform: rotate(${rot}deg);" title="ConeRobot ${rNum}">
      <svg width="34" height="34" viewBox="0 0 34 34">
        <polygon points="17,2 29,28 17,22 5,28" fill="${strokeColor}" fill-opacity="0.9" stroke="#ffffff" stroke-width="1.5" />
      </svg>
      <div class="robot-pin-label" style="transform: rotate(-${rot}deg);">R${rNum}</div>
    </div>
  `;

  return L.divIcon({
    className: 'custom-robot-leaflet-icon',
    html: svgHtml,
    iconSize: [34, 34],
    iconAnchor: [17, 17]
  });
}

function updateMapMarkers() {
  if (!map) return;
  const activeValidIds = new Set();

  Object.keys(realTelemetry).forEach(id => {
    const r = realTelemetry[id];
    if (!r || !r.online) return;

    const lat = r.gps?.lat;
    const lon = r.gps?.lon;
    const heading = r.imu?.heading;
    const gpsMode = r.gps?.mode || 'NO FIX';
    const isLidarOnly = !r.gps?.present && r.lidar?.present;

    if (lat && lon && Math.abs(lat) > 0.0001 && Math.abs(lon) > 0.0001) {
      activeValidIds.add(String(id));
      const icon = createRobotIcon(id, heading, gpsMode, isLidarOnly);

      if (robotMarkers[id]) {
        robotMarkers[id].setLatLng([lat, lon]);
        robotMarkers[id].setIcon(icon);
      } else {
        const marker = L.marker([lat, lon], { icon: icon }).addTo(map);
        marker.on('click', () => openRobotDetailModal(id));
        robotMarkers[id] = marker;
        map.setView([lat, lon], 19);
      }
    }
  });

  Object.keys(robotMarkers).forEach(id => {
    if (!activeValidIds.has(String(id))) {
      map.removeLayer(robotMarkers[id]);
      delete robotMarkers[id];
    }
  });
}

function fitAllRobotsInMap() {
  const coords = [];
  Object.values(robotMarkers).forEach(m => coords.push(m.getLatLng()));
  if (coords.length > 0) {
    const bounds = L.latLngBounds(coords);
    map.fitBounds(bounds, { padding: [40, 40], maxZoom: 20 });
  }
}

// --- 2. Card Grid DOM Initialization (Built Once) ---
function initCardsGridDOM() {
  const container = document.getElementById('robot-cards-container');
  container.innerHTML = '';

  for (let i = 1; i <= 13; i++) {
    const rNum = String(i).padStart(2, '0');
    const card = document.createElement('div');
    card.className = 'robot-card offline';
    card.id = `card-robot-${i}`;
    card.onclick = () => openRobotDetailModal(i);

    card.innerHTML = `
      <div class="card-top">
        <div class="robot-id-wrap">
          <div style="display: flex; align-items: center; gap: 6px;">
            <span class="robot-card-title">ConeRobot ${rNum}</span>
            <span class="robot-type-pill" id="r${i}-type-pill" style="font-size: 0.65rem; padding: 2px 6px; border-radius: 4px; background: rgba(255,255,255,0.06); color: var(--text-muted);">Auto</span>
          </div>
          <span class="robot-card-ip" id="r${i}-ip">conerobot${rNum}.local</span>
        </div>
        <span class="status-badge red" id="r${i}-badge">OFFLINE</span>
      </div>

      <div class="card-metrics">
        <div class="metric-row">
          <span class="metric-key">LiDAR (/scan)</span>
          <span class="metric-val" id="r${i}-lidar">--</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">GPS Fix</span>
          <span class="metric-val" id="r${i}-gps">--</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">IMU Heading</span>
          <span class="metric-val" id="r${i}-heading">--</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Step Controller</span>
          <span class="metric-val" id="r${i}-step">--</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Battery</span>
          <span class="metric-val" id="r${i}-battery" style="color: var(--text-muted);">No Sensor</span>
        </div>
      </div>

      <div class="card-footer">
        <span class="ping-tag" id="r${i}-ping">⚪ Offline</span>
        <span id="r${i}-foot-info">Port 8765</span>
      </div>
    `;

    container.appendChild(card);

    realTelemetry[i] = {
      id: i,
      ip: '',
      online: false,
      lidar: { present: false, freq_hz: 0, sample_count: 0, min_dist: null, last_scan_time: 0 },
      gps: { present: false, mode: 'NO FIX', lat: null, lon: null, alt: null, sats: 0, hdop: null },
      imu: { heading: null, pitch: null, roll: null, yaw: null },
      motion: { state: 'Offline', step_progress: '--' },
      health: { cpu_temp: '--', cpu_load: '--', battery_v: '--' }
    };
  }
}

// --- 3. Discovery Loop ---
async function startDiscoveryLoop() {
  logDebug('Discovery engine started. Fetching discovered fleet...', 'info');

  async function pollFleet() {
    try {
      const res = await fetch('/api/fleet');
      if (res.ok) {
        const data = await res.json();
        configuredTopics = data.topics || configuredTopics;
        const robots = data.robots || {};

        for (let i = 1; i <= 13; i++) {
          const rob = robots[i];
          if (rob && rob.online && rob.ip) {
            realTelemetry[i].ip = rob.ip;
            
            // Connect only if socket is not already created / open
            if (!activeWebSockets[i] && !reconnectTimers[i]) {
              connectToRobotWebSocket(i, rob.ip, rob.port || 8765);
            }
          }
        }
      }
    } catch (e) {}
  }

  await pollFleet();
  setInterval(pollFleet, 5000);
}

function connectToRobotWebSocket(robotId, ip, port) {
  if (activeWebSockets[robotId]) {
    const st = activeWebSockets[robotId].readyState;
    if (st === WebSocket.CONNECTING || st === WebSocket.OPEN) {
      return; // Already connecting or open
    }
  }

  const wsUrl = `ws://${ip}:${port}`;
  logDebug(`[Robot ${robotId}] Connecting to ${wsUrl}...`, 'info');

  let ws;
  try {
    ws = new WebSocket(wsUrl, ["foxglove.sdk.v1", "foxglove.websocket.v1"]);
  } catch (e) {
    try { ws = new WebSocket(wsUrl); } catch(err) {
      logDebug(`[Robot ${robotId}] WebSocket error: ${err}`, 'error');
      scheduleReconnect(robotId, ip, port);
      return;
    }
  }

  activeWebSockets[robotId] = ws;
  ws.binaryType = "arraybuffer";

  let channelMap = {};
  let subIdCounter = 1;

  ws.onopen = () => {
    logDebug(`🟢 [Robot ${robotId}] Connected to ${wsUrl}!`, 'success');
    realTelemetry[robotId].online = true;
    
    // Send Foxglove clientInfo handshake
    try {
      ws.send(JSON.stringify({
        op: "clientInfo",
        name: "ConeRobotFleetDashboard"
      }));
    } catch(e){}
  };

  ws.onmessage = (event) => {
    realTelemetry[robotId].online = true;

    if (typeof event.data === "string") {
      try {
        const msg = JSON.parse(event.data);
        
        // 1. Channel Advertisement
        if (msg.op === "advertise" && Array.isArray(msg.channels)) {
          const subscriptions = [];
          let hasLidarTopic = false;
          let hasGpsTopic = false;
          const targetTopics = [
            '/scan',
            '/fix',
            '/gps/status',
            '/imu/heading',
            '/imu/data',
            '/step_status',
            '/battery_state',
            '/foxglove_bridge/sysinfo',
            '/robot/diagnostics'
          ];

          const seen = new Set();
          msg.channels.forEach(ch => {
            const topic = ch.topic;
            if (topic === configuredTopics.scan || topic === '/scan') hasLidarTopic = true;
            if (topic === configuredTopics.gps_fix || topic === '/fix') hasGpsTopic = true;

            const isTarget = targetTopics.includes(topic) || Object.values(configuredTopics).includes(topic);

            if (isTarget && !seen.has(topic)) {
              seen.add(topic);
              const subId = subIdCounter++;
              channelMap[subId] = topic;
              channelMap[ch.id] = topic;
              subscriptions.push({
                id: subId,
                channelId: ch.id
              });
            }
          });

          realTelemetry[robotId].lidar.present = hasLidarTopic;
          realTelemetry[robotId].gps.present = hasGpsTopic;

          const topicNames = subscriptions.map(s => channelMap[s.id]).join(', ');
          logDebug(`🛰️ [Robot ${robotId}] Subscribed to ${subscriptions.length} topics: ${topicNames}`, 'topic');

          if (subscriptions.length > 0) {
            ws.send(JSON.stringify({ op: "subscribe", subscriptions: subscriptions }));
          }
        }
        // 2. Message Data
        else if (msg.op === "message" && msg.data) {
          const topic = channelMap[msg.subscriptionId] || channelMap[msg.channelId] || channelMap[msg.id];
          handleParsedRobotMessage(robotId, topic, msg.data);
        }
      } catch (e) {}
    } else if (event.data instanceof ArrayBuffer) {
      handleBinaryFoxgloveMessage(robotId, event.data, channelMap);
    }
  };

  ws.onerror = () => {
    // Suppress unneeded browser warnings
  };

  ws.onclose = (event) => {
    logDebug(`🔴 [Robot ${robotId}] WebSocket Disconnected (Code: ${event.code}).`, 'warn');
    delete activeWebSockets[robotId];
    if (realTelemetry[robotId]) {
      realTelemetry[robotId].online = false;
    }
    scheduleReconnect(robotId, ip, port);
  };
}

function scheduleReconnect(robotId, ip, port) {
  if (reconnectTimers[robotId]) return;
  reconnectTimers[robotId] = setTimeout(() => {
    delete reconnectTimers[robotId];
    connectToRobotWebSocket(robotId, ip, port);
  }, 4000);
}

function handleSysInfoData(r, sys) {
  if (!r || !sys) return;
  let cpu = null;
  if (sys.total_cpu_percent !== undefined) {
    cpu = `${Number(sys.total_cpu_percent).toFixed(1)}%`;
  } else if (sys.process_cpu_percent !== undefined) {
    cpu = `${Number(sys.process_cpu_percent).toFixed(1)}%`;
  } else if (sys.cpu_percentage !== undefined) {
    cpu = `${Math.round(sys.cpu_percentage * 100)}%`;
  }

  let ram = null;
  if (sys.used_memory !== undefined && sys.total_memory !== undefined) {
    const usedMb = Math.round(sys.used_memory / (1024 * 1024));
    const totalMb = Math.round(sys.total_memory / (1024 * 1024));
    ram = `${usedMb} MB / ${totalMb} MB`;
  } else if (sys.used_memory !== undefined) {
    ram = `${Math.round(sys.used_memory / (1024 * 1024))} MB`;
  }

  r.health.cpu_percent = cpu || '--';
  r.health.ram_mb = ram || '--';
  if (cpu || ram) {
    r.health.cpu_load = [cpu ? `${cpu} CPU` : null, ram].filter(Boolean).join(' | ');
  }
}

function cdrAlign(ptr, alignment) {
  const rel = ptr - 17;
  const alignedRel = (rel + (alignment - 1)) & ~(alignment - 1);
  return 17 + alignedRel;
}

function handleBinaryFoxgloveMessage(robotId, buffer, channelMap) {
  const view = new DataView(buffer);
  if (buffer.byteLength < 17) return;
  const op = view.getUint8(0);
  
  // Opcode 1 = Message Data
  if (op === 1) {
    const subId = view.getUint32(1, true);
    const topic = channelMap[subId];
    if (!topic) {
      return; // Skip messages for unmapped or unsubscribed channels
    }

    const r = realTelemetry[robotId];
    if (!r) return;

    try {
      // 0. foxglove.SystemInfo (/foxglove_bridge/sysinfo)
      if (topic === '/foxglove_bridge/sysinfo' || topic.endsWith('sysinfo')) {
        const jsonBytes = new Uint8Array(buffer, 13);
        const rawText = new TextDecoder('utf-8').decode(jsonBytes);
        const firstBrace = rawText.indexOf('{');
        const lastBrace = rawText.lastIndexOf('}');
        if (firstBrace !== -1 && lastBrace > firstBrace) {
          try {
            const jsonStr = rawText.substring(firstBrace, lastBrace + 1);
            const sys = JSON.parse(jsonStr);
            handleSysInfoData(r, sys);
          } catch(e) {
            // Silently ignore non-JSON or partial binary packets
          }
        }
        return;
      }

      // CDR payload starts at byte 13.
      // CDR 4-byte header is at 13..16. Data starts at byte 17.
      let ptr = 17;

      // 1. sensor_msgs/msg/LaserScan (/scan)
      if (topic === configuredTopics.scan || topic === '/scan') {
        r.lidar.present = true;
        const now = Date.now();
        if (r.lidar.last_scan_time > 0) {
          const dt = (now - r.lidar.last_scan_time) / 1000;
          if (dt > 0.01) r.lidar.freq_hz = Number((1.0 / dt).toFixed(1));
        }
        r.lidar.last_scan_time = now;
      }

      // 2. sensor_msgs/msg/NavSatFix (/fix)
      else if (topic === configuredTopics.gps_fix || topic === '/fix') {
        r.gps.present = true;
        // Skip ROS 2 Header
        ptr += 8;
        const frameLen = view.getUint32(ptr, true);
        ptr += 4 + frameLen;

        // NavSatStatus: status (int8), service (uint16)
        const statusInt = view.getInt8(ptr); ptr += 1;
        ptr = cdrAlign(ptr, 2);
        ptr += 2;             // service
        ptr = cdrAlign(ptr, 8); // align 8 for float64

        if (ptr + 24 <= buffer.byteLength) {
          const lat = view.getFloat64(ptr, true); ptr += 8;
          const lon = view.getFloat64(ptr, true); ptr += 8;
          const alt = view.getFloat64(ptr, true); ptr += 8;

          r.gps.lat = Number(lat.toFixed(7));
          r.gps.lon = Number(lon.toFixed(7));
          r.gps.alt = Number(alt.toFixed(1));

          if (r.gps.mode === 'NO FIX' && Math.abs(lat) > 0.0001) {
            r.gps.mode = '3D FIX';
          }
        }
      }

      // 3. std_msgs/msg/Float32 (/imu/heading)
      else if (topic === configuredTopics.heading || topic === '/imu/heading') {
        if (ptr + 4 <= buffer.byteLength) {
          let val = view.getFloat32(ptr, true);
          val = ((val % 360) + 360) % 360;
          const rounded = Number(val.toFixed(1));
          if (r.imu.heading !== rounded) {
            r.imu.heading = rounded;
            logDebug(`🧭 [Robot ${robotId}] Live Heading: ${r.imu.heading}°`, 'topic');
          }
        }
      }

      // 4. std_msgs/msg/String (/gps/status, /step_status)
      else if (topic === configuredTopics.gps_status || topic === '/gps/status') {
        r.gps.present = true;
        const strLen = view.getUint32(ptr, true); ptr += 4;
        if (strLen > 0 && ptr + strLen <= buffer.byteLength) {
          const strBytes = new Uint8Array(buffer, ptr, strLen);
          const str = new TextDecoder('utf-8').decode(strBytes).replace(/\0/g, '').trim();
          if (str.toUpperCase().includes('RTK FIX') || str.includes('4')) r.gps.mode = 'RTK FIX';
          else if (str.toUpperCase().includes('FLOAT') || str.includes('5')) r.gps.mode = 'FLOAT';
          else if (str.toUpperCase().includes('3D') || str.includes('1') || str.includes('2')) r.gps.mode = '3D FIX';
          else if (str) r.gps.mode = str;

          const satMatch = str.match(/Sats:\s*(\d+)/i) || str.match(/(\d+)\s*sats/i);
          if (satMatch) r.gps.sats = parseInt(satMatch[1]);
        }
      }

      else if (topic === configuredTopics.step_status || topic === '/step_status') {
        const strLen = view.getUint32(ptr, true); ptr += 4;
        if (strLen > 0 && ptr + strLen <= buffer.byteLength) {
          const strBytes = new Uint8Array(buffer, ptr, strLen);
          const str = new TextDecoder('utf-8').decode(strBytes).replace(/\0/g, '').trim();
          r.motion.step_progress = str;
          r.motion.state = str.includes('Driving') || str.includes('Turning') ? 'Moving' : 'Standby';
        }
      }

      // 6. std_msgs/msg/String (/robot/diagnostics)
      else if (topic === '/robot/diagnostics' || topic === 'robot/diagnostics') {
        const strLen = view.getUint32(ptr, true); ptr += 4;
        if (strLen > 0 && ptr + strLen <= buffer.byteLength) {
          const strBytes = new Uint8Array(buffer, ptr, strLen);
          const str = new TextDecoder('utf-8').decode(strBytes).replace(/\0/g, '').trim();
          try {
            const diag = JSON.parse(str);
            if (diag.cpu_temp) r.health.cpu_temp = diag.cpu_temp;
          } catch(e) {
            if (str.includes('°C') || str.includes('C')) r.health.cpu_temp = str;
          }
        }
      }

      // 7. foxglove.SystemInfo (/foxglove_bridge/sysinfo)
      else if (topic === '/foxglove_bridge/sysinfo' || topic === 'foxglove_bridge/sysinfo' || (topic && topic.includes('sysinfo'))) {
        try {
          const jsonBytes = new Uint8Array(buffer, 13);
          const rawText = new TextDecoder('utf-8').decode(jsonBytes);
          const jsonMatch = rawText.substring(rawText.indexOf('{'), rawText.lastIndexOf('}') + 1);
          if (jsonMatch) {
            const sys = JSON.parse(jsonMatch);
            handleSysInfoData(r, sys);
          }
        } catch(e) {}
      }

      // 5. sensor_msgs/msg/Imu (/imu/data)
      else if (topic === configuredTopics.imu_data || topic === '/imu/data') {
        // Skip Header: sec (4), nsec (4)
        ptr += 8;
        const frameLen = view.getUint32(ptr, true);
        ptr += 4 + frameLen;
        
        // Align to 8 bytes relative to CDR buffer start
        ptr = cdrAlign(ptr, 8);

        if (ptr + 32 <= buffer.byteLength) {
          const x = view.getFloat64(ptr, true); ptr += 8;
          const y = view.getFloat64(ptr, true); ptr += 8;
          const z = view.getFloat64(ptr, true); ptr += 8;
          const w = view.getFloat64(ptr, true); ptr += 8;

          const sinr_cosp = 2 * (w * x + y * z);
          const cosr_cosp = 1 - 2 * (x * x + y * y);
          const roll = Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI);

          const sinp = 2 * (w * y - z * x);
          let pitch;
          if (Math.abs(sinp) >= 1) pitch = Math.sign(sinp) * 90;
          else pitch = Math.asin(sinp) * (180 / Math.PI);

          const siny_cosp = 2 * (w * z + x * y);
          const cosy_cosp = 1 - 2 * (y * y + z * z);
          let yaw = Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI);
          yaw = ((yaw % 360) + 360) % 360;
          if (yaw >= 359.95) yaw = 0.0;

          r.imu.roll = Number(roll.toFixed(1));
          r.imu.pitch = Number(pitch.toFixed(1));
          r.imu.yaw = Number(yaw.toFixed(1));
          
          const prevH = r.imu.heading;
          r.imu.heading = Number(yaw.toFixed(1));
          if (prevH === null || Math.abs((prevH || 0) - r.imu.heading) > 0.4) {
            logDebug(`🧭 [Robot ${robotId}] IMU Live Yaw: ${r.imu.heading}° (Pitch: ${r.imu.pitch}°, Roll: ${r.imu.roll}°)`, 'topic');
          }
        }
      }

    } catch (e) {}
  }
}

function handleParsedRobotMessage(robotId, topic, data) {
  const r = realTelemetry[robotId];
  if (!r) return;

  // 1. LiDAR LaserScan (/scan)
  if (topic === configuredTopics.scan || topic === '/scan') {
    r.lidar.present = true;
    const now = Date.now();
    if (r.lidar.last_scan_time > 0) {
      const dt = (now - r.lidar.last_scan_time) / 1000;
      if (dt > 0.01) {
        r.lidar.freq_hz = Number((1.0 / dt).toFixed(1));
      }
    }
    r.lidar.last_scan_time = now;

    if (Array.isArray(data.ranges)) {
      r.lidar.sample_count = data.ranges.length;
      let minD = Infinity;
      for (let i = 0; i < data.ranges.length; i++) {
        const d = data.ranges[i];
        if (d > 0.05 && d < 12.0 && d < minD) {
          minD = d;
        }
      }
      r.lidar.min_dist = minD !== Infinity ? Number(minD.toFixed(2)) : null;
    }
  }

  // 2. GPS Fix (/fix)
  if (topic === configuredTopics.gps_fix || topic === '/fix') {
    r.gps.present = true;
    if (data.latitude !== undefined && data.longitude !== undefined) {
      r.gps.lat = Number(data.latitude);
      r.gps.lon = Number(data.longitude);
      r.gps.alt = data.altitude ? Number(data.altitude).toFixed(1) : '--';
      const statusInt = data.status?.status ?? data.status;
      if (statusInt >= 0 && r.gps.mode === 'NO FIX') r.gps.mode = '3D FIX';
    }
  }

  // 3. GPS Status (/gps/status)
  if (topic === configuredTopics.gps_status || topic === '/gps/status') {
    r.gps.present = true;
    const str = typeof data === 'string' ? data : (data.data || '');
    if (str) {
      if (str.toUpperCase().includes('RTK FIX') || str.includes('4')) r.gps.mode = 'RTK FIX';
      else if (str.toUpperCase().includes('FLOAT') || str.includes('5')) r.gps.mode = 'FLOAT';
      else if (str.toUpperCase().includes('3D') || str.includes('1') || str.includes('2')) r.gps.mode = '3D FIX';
      else r.gps.mode = str;

      const satMatch = str.match(/Sats:\s*(\d+)/i) || str.match(/(\d+)\s*sats/i);
      if (satMatch) r.gps.sats = parseInt(satMatch[1]);
    }
  }

  // 4. IMU Heading (/imu/heading)
  if (topic === configuredTopics.heading || topic === '/imu/heading') {
    const val = typeof data === 'number' ? data : (data.data !== undefined ? Number(data.data) : null);
    if (val !== null && !isNaN(val)) {
      r.imu.heading = Number(val.toFixed(1));
    }
  }

  // 5. IMU Data Orientation (/imu/data)
  if (topic === configuredTopics.imu_data || topic === '/imu/data') {
    if (data.orientation) {
      const { x, y, z, w } = data.orientation;
      const sinr_cosp = 2 * (w * x + y * z);
      const cosr_cosp = 1 - 2 * (x * x + y * y);
      r.imu.roll = (Math.atan2(sinr_cosp, cosr_cosp) * (180 / Math.PI)).toFixed(1);

      const sinp = 2 * (w * y - z * x);
      if (Math.abs(sinp) >= 1) r.imu.pitch = (Math.sign(sinp) * 90).toFixed(1);
      else r.imu.pitch = (Math.asin(sinp) * (180 / Math.PI)).toFixed(1);

      const siny_cosp = 2 * (w * z + x * y);
      const cosy_cosp = 1 - 2 * (y * y + z * z);
      let yaw = (Math.atan2(siny_cosp, cosy_cosp) * (180 / Math.PI));
      if (yaw < 0) yaw += 360;
      r.imu.yaw = yaw.toFixed(1);
      if (r.imu.heading === null) r.imu.heading = Number(yaw.toFixed(1));
    }
  }

  // 6. Step Motion Status (/step_status)
  if (topic === configuredTopics.step_status || topic === '/step_status') {
    const stepStr = typeof data === 'string' ? data : (data.data || '--');
    r.motion.step_progress = stepStr;
    r.motion.state = stepStr.includes('Driving') || stepStr.includes('Turning') ? 'Moving' : 'Standby';
  }

  // 7. Battery State (/battery_state)
  if (topic === configuredTopics.battery || topic === '/battery_state') {
    if (data.voltage !== undefined) r.health.battery_v = `${Number(data.voltage).toFixed(1)}V`;
  }

  // 8. Foxglove System Info (/foxglove_bridge/sysinfo)
  if (topic === '/foxglove_bridge/sysinfo' || topic === 'foxglove_bridge/sysinfo') {
    handleSysInfoData(r, data);
  }

  // 9. Robot Diagnostics (/robot/diagnostics)
  if (topic === '/robot/diagnostics' || topic === 'robot/diagnostics') {
    if (typeof data === 'object' && data.cpu_temp) {
      r.health.cpu_temp = data.cpu_temp;
    } else if (typeof data === 'string') {
      try {
        const parsed = JSON.parse(data);
        if (parsed.cpu_temp) r.health.cpu_temp = parsed.cpu_temp;
      } catch(e) {
        if (data.includes('°C') || data.includes('C')) r.health.cpu_temp = data;
      }
    }
  }
}

// --- 4. High Performance UI Update Engine (0 Lag) ---
function renderUIUpdates() {
  let activeCount = 0;
  let idleCount = 0;
  let offlineCount = 0;
  let rtkCount = 0;

  for (let i = 1; i <= 13; i++) {
    const r = realTelemetry[i];
    const isAlive = Boolean(r.online);

    const card = document.getElementById(`card-robot-${i}`);
    const badge = document.getElementById(`r${i}-badge`);
    const typePill = document.getElementById(`r${i}-type-pill`);
    const ipEl = document.getElementById(`r${i}-ip`);
    const lidarEl = document.getElementById(`r${i}-lidar`);
    const gpsEl = document.getElementById(`r${i}-gps`);
    const headEl = document.getElementById(`r${i}-heading`);
    const stepEl = document.getElementById(`r${i}-step`);
    const pingEl = document.getElementById(`r${i}-ping`);

    if (isAlive) {
      const isMoving = r.motion.state === 'Moving';
      if (isMoving) activeCount++;
      else idleCount++;

      if (r.gps.mode === 'RTK FIX') rtkCount++;

      card.className = `robot-card ${isMoving ? 'active' : 'idle'}`;
      badge.className = `status-badge ${isMoving ? 'green' : 'yellow'}`;
      badge.innerText = isMoving ? 'MOVING' : 'ONLINE';

      // Heterogeneous Robot Type Pill
      if (r.lidar.present && r.gps.present) {
        typePill.innerText = 'LiDAR + GPS';
        typePill.style.color = '#38bdf8';
      } else if (r.lidar.present) {
        typePill.innerText = 'LiDAR Robot';
        typePill.style.color = '#60a5fa';
      } else if (r.gps.present) {
        typePill.innerText = 'GPS Robot';
        typePill.style.color = '#34d399';
      } else {
        typePill.innerText = 'Standard';
        typePill.style.color = 'var(--text-muted)';
      }

      ipEl.innerText = r.ip;
      
      // LiDAR Metric
      if (r.lidar.present) {
        if (r.lidar.freq_hz > 0) {
          lidarEl.innerText = `🟢 Active (${r.lidar.freq_hz} Hz)`;
          lidarEl.className = 'metric-val gps-rtk';
        } else {
          lidarEl.innerText = '🟢 Active';
          lidarEl.className = 'metric-val gps-float';
        }
      } else {
        lidarEl.innerText = '⚪ Not Equipped';
        lidarEl.className = 'metric-val';
        lidarEl.style.color = 'var(--text-muted)';
      }

      // GPS Metric
      if (r.gps.present) {
        let gpsTxt = (r.gps.lat && r.gps.lon) ? `${r.gps.mode} (${r.gps.sats} Sats)` : `${r.gps.mode} (No Lock)`;
        gpsEl.innerText = gpsTxt;
        gpsEl.className = `metric-val ${r.gps.mode === 'RTK FIX' ? 'gps-rtk' : (r.gps.mode === 'FLOAT' ? 'gps-float' : 'gps-none')}`;
      } else {
        gpsEl.innerText = 'Not Equipped';
        gpsEl.className = 'metric-val';
        gpsEl.style.color = 'var(--text-muted)';
      }

      headEl.innerText = (r.imu.heading !== null) ? `${Number(r.imu.heading).toFixed(1)}°` : 'Waiting for IMU...';
      stepEl.innerText = r.motion.step_progress;
      pingEl.innerText = '🟢 Connected';

      const battEl = document.getElementById(`r${i}-battery`);
      if (battEl) {
        let sysTxt = '--';
        if (r.health.cpu_temp !== '--' && r.health.cpu_load) {
          sysTxt = `${r.health.cpu_temp} (${r.health.cpu_load})`;
        } else if (r.health.cpu_temp !== '--') {
          sysTxt = `${r.health.cpu_temp}`;
        } else if (r.health.cpu_load) {
          sysTxt = `${r.health.cpu_load}`;
        }
        battEl.innerText = sysTxt;
      }
    } else {
      offlineCount++;
      card.className = 'robot-card offline';
      badge.className = 'status-badge red';
      badge.innerText = 'OFFLINE';
      typePill.innerText = 'Auto';
      typePill.style.color = 'var(--text-muted)';
      ipEl.innerText = `conerobot${String(i).padStart(2, '0')}.local`;
      lidarEl.innerText = '--';
      lidarEl.className = 'metric-val';
      gpsEl.innerText = '--';
      gpsEl.className = 'metric-val';
      headEl.innerText = '--';
      stepEl.innerText = '--';
      pingEl.innerText = '⚪ Offline';
      const battEl = document.getElementById(`r${i}-battery`);
      if (battEl) battEl.innerText = '--';
    }
  }

  document.getElementById('stat-active').innerText = activeCount;
  document.getElementById('stat-idle').innerText = idleCount;
  document.getElementById('stat-offline').innerText = offlineCount;
  document.getElementById('stat-rtk').innerText = rtkCount;
  document.getElementById('last-update').innerText = `Live: ${new Date().toLocaleTimeString()}`;

  updateMapMarkers();

  if (selectedRobotId && realTelemetry[selectedRobotId]) {
    updateModalContent(realTelemetry[selectedRobotId]);
  }
}

// --- 5. Detail Modal ---
function openRobotDetailModal(robotId) {
  selectedRobotId = robotId;
  const modal = document.getElementById('robot-detail-modal');
  modal.classList.remove('hidden');
  const r = realTelemetry[robotId];
  if (r) updateModalContent(r);
}

function updateModalContent(r) {
  const rNum = String(r.id).padStart(2, '0');
  document.getElementById('modal-robot-title').innerText = `ConeRobot ${rNum}`;
  document.getElementById('modal-ip').innerText = r.ip || 'Offline';
  
  if (r.gps.present) {
    document.getElementById('modal-gps-mode').innerText = `${r.gps.mode} (${r.gps.sats || 0} Sats)`;
  } else {
    document.getElementById('modal-gps-mode').innerText = 'No GPS Module';
  }

  document.getElementById('modal-ping').innerText = r.online ? 'Connected' : 'Offline';

  // LiDAR Widget in Modal
  if (r.lidar.present) {
    const isScanActive = (r.lidar.freq_hz > 0);
    document.getElementById('modal-lidar-status').innerText = isScanActive ? '🟢 Active & Streaming' : '🟡 Topic Present';
    document.getElementById('modal-lidar-status').style.color = isScanActive ? '#10b981' : '#f59e0b';
    document.getElementById('modal-lidar-freq').innerText = r.lidar.freq_hz > 0 ? `${r.lidar.freq_hz} Hz` : 'Waiting stream...';
  } else {
    document.getElementById('modal-lidar-status').innerText = '⚪ Not Equipped';
    document.getElementById('modal-lidar-status').style.color = 'var(--text-muted)';
    document.getElementById('modal-lidar-freq').innerText = 'N/A';
  }

  const heading = r.imu.heading;
  if (heading !== null && !isNaN(heading)) {
    document.getElementById('modal-heading-val').innerText = `${heading}°`;
    document.getElementById('modal-needle').style.transform = `rotate(${heading}deg)`;
    document.getElementById('modal-yaw').innerText = `${heading}°`;
  } else {
    document.getElementById('modal-heading-val').innerText = '--';
    document.getElementById('modal-yaw').innerText = 'No Data';
  }

  const cpuEl = document.getElementById('modal-cpu');
  if (cpuEl) cpuEl.innerText = r.health.cpu_percent || '--';

  const ramEl = document.getElementById('modal-ram');
  if (ramEl) ramEl.innerText = r.health.ram_mb || '--';

  const tempEl = document.getElementById('modal-temp');
  if (tempEl) {
    tempEl.innerText = r.health.cpu_temp !== '--' ? r.health.cpu_temp : '--';
    if (r.health.cpu_temp !== '--') {
      const num = parseFloat(r.health.cpu_temp);
      if (!isNaN(num)) {
        tempEl.style.color = num > 80 ? '#ef4444' : (num > 70 ? '#f59e0b' : '#10b981');
      }
    }
  }

  document.getElementById('modal-lat').innerText = r.gps.lat ? r.gps.lat.toFixed(7) : (r.gps.present ? 'NO FIX' : 'Not Equipped');
  document.getElementById('modal-lon').innerText = r.gps.lon ? r.gps.lon.toFixed(7) : (r.gps.present ? 'NO FIX' : 'Not Equipped');
  document.getElementById('modal-alt').innerText = r.gps.alt ? `${r.gps.alt} m` : '--';
  document.getElementById('modal-hdop').innerText = r.gps.hdop || '--';

  document.getElementById('modal-motion-state').innerText = r.motion.state;
  document.getElementById('modal-speed').innerText = '--';
  document.getElementById('modal-dist-progress').innerText = r.motion.step_progress;
  document.getElementById('modal-turn-progress').innerText = '--';
}

function initEventListeners() {
  document.getElementById('btn-close-modal').onclick = () => {
    document.getElementById('robot-detail-modal').classList.add('hidden');
    selectedRobotId = null;
  };

  document.getElementById('btn-close-settings').onclick = () => {
    document.getElementById('settings-modal').classList.add('hidden');
  };

  document.getElementById('btn-settings').onclick = async () => {
    document.getElementById('settings-modal').classList.remove('hidden');
    try {
      const res = await fetch('/api/config');
      if (res.ok) {
        const cfg = await res.json();
        const top = cfg.topics || {};
        document.getElementById('robot1-ip').value = (cfg.robots && cfg.robots[0]?.ip) || (cfg.robots && cfg.robots[0]?.host) || '';
        document.getElementById('topic-lidar').value = top.scan || '/scan';
        document.getElementById('topic-gps').value = top.gps_fix || '/fix';
        document.getElementById('topic-heading').value = top.heading || '/imu/heading';
        document.getElementById('topic-imu').value = top.imu_data || '/imu/data';
        document.getElementById('topic-step').value = top.step_status || '/step_status';
        document.getElementById('topic-battery').value = top.battery || '/battery_state';
      }
    } catch(e) {}
  };

  document.getElementById('settings-form').onsubmit = async (e) => {
    e.preventDefault();
    const r1Input = document.getElementById('robot1-ip').value.trim();
    const updatedTopics = {
      scan: document.getElementById('topic-lidar').value,
      gps_fix: document.getElementById('topic-gps').value,
      heading: document.getElementById('topic-heading').value,
      imu_data: document.getElementById('topic-imu').value,
      step_status: document.getElementById('topic-step').value,
      battery: document.getElementById('topic-battery').value
    };

    try {
      const res = await fetch('/api/config');
      const cfg = await res.json();
      cfg.topics = { ...cfg.topics, ...updatedTopics };
      if (r1Input && cfg.robots && cfg.robots.length > 0) {
        if (r1Input.includes('.')) {
          if (r1Input.endsWith('.local')) cfg.robots[0].host = r1Input;
          else cfg.robots[0].ip = r1Input;
        }
      }

      await fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(cfg)
      });

      document.getElementById('settings-modal').classList.add('hidden');
      fetch('/api/scan');
    } catch (e) {
      alert('Failed to save settings: ' + e);
    }
  };

  document.getElementById('btn-scan').onclick = async () => {
    const btn = document.getElementById('btn-scan');
    btn.innerHTML = 'Scanning...';
    btn.disabled = true;
    try {
      await fetch('/api/scan');
      setTimeout(() => {
        btn.innerHTML = `
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M21.5 2v6h-6M21.34 15.57a10 10 0 1 1-.57-8.38l5.67-5.67"/>
          </svg> Scan Network
        `;
        btn.disabled = false;
      }, 2500);
    } catch(e) {
      btn.disabled = false;
    }
  };

  const btnToggle = document.getElementById('btn-toggle-debug');
  if (btnToggle) {
    btnToggle.onclick = () => {
      const dbgSection = document.getElementById('debug-section');
      if (dbgSection) {
        dbgSection.style.display = dbgSection.style.display === 'none' ? 'block' : 'none';
      }
    };
  }

  const btnClear = document.getElementById('btn-clear-debug');
  if (btnClear) {
    btnClear.onclick = () => {
      const container = document.getElementById('debug-log-container');
      if (container) container.innerHTML = '';
    };
  }
}
