/* ==========================================================================
   ConeRobot Manual Drive (WASD Teleop)
   Publishes geometry_msgs/msg/Twist directly on /cmd_vel via Foxglove WebSocket
   (Zero extra ROS 2 nodes or RAM on the Raspberry Pi 5).
   ========================================================================== */

let activeRobotId = null;
let fleetSnapshot = {};

// Foxglove Client Publish Channels
const TELEOP_CHANNELS = {
  CMD_VEL: 1,
  CMD_PRIMITIVE: 2
};

// Active pressed keys (keyboard + on-screen pointer hold)
const pressedDirs = new Set(); // 'w', 'a', 's', 'd'
let turboHeld = false;
let manualLinearMax = parseFloat(localStorage.getItem('conerobot_teleop_lin') || '0.30') || 0.30;
let manualAngularMax = parseFloat(localStorage.getItem('conerobot_teleop_ang') || '1.10') || 1.10;

let lastCmdLinear = 0.0;
let lastCmdAngular = 0.0;
let wasMoving = false;
let cancelledMissionOnTakeover = false;

// ==============================================================================
// 1. CDR ENCODERS (Foxglove WebSocket Client Publish Opcode = 1)
// ==============================================================================
function encodeTwistCDR(channelId, linearX, angularZ) {
  // 1B op(1) + 4B channelId + 4B CDR LE header + 6 x 8B float64 = 57 bytes
  const buf = new ArrayBuffer(57);
  const view = new DataView(buf);
  view.setUint8(0, 1);
  view.setUint32(1, channelId, true);
  // CDR Little-Endian encapsulation header
  view.setUint8(5, 0x00);
  view.setUint8(6, 0x01);
  view.setUint8(7, 0x00);
  view.setUint8(8, 0x00);
  // geometry_msgs/msg/Twist: linear (x, y, z), angular (x, y, z)
  view.setFloat64(9, Number(linearX) || 0.0, true);
  view.setFloat64(17, 0.0, true);
  view.setFloat64(25, 0.0, true);
  view.setFloat64(33, 0.0, true);
  view.setFloat64(41, 0.0, true);
  view.setFloat64(49, Number(angularZ) || 0.0, true);
  return buf;
}

function encodeVector3CDR(channelId, x, y, z) {
  const buf = new ArrayBuffer(33);
  const view = new DataView(buf);
  view.setUint8(0, 1);
  view.setUint32(1, channelId, true);
  view.setUint8(5, 0x00);
  view.setUint8(6, 0x01);
  view.setUint8(7, 0x00);
  view.setUint8(8, 0x00);
  view.setFloat64(9, Number(x) || 0.0, true);
  view.setFloat64(17, Number(y) || 0.0, true);
  view.setFloat64(25, Number(z) || 0.0, true);
  return buf;
}

// ==============================================================================
// 2. INITIALIZATION
// ==============================================================================
document.addEventListener('DOMContentLoaded', () => {
  for (let i = 1; i <= 13; i++) {
    if (!realTelemetry[i]) {
      realTelemetry[i] = createEmptyRobotTelemetry(i);
    }
  }

  initSpeedControls();
  bindKeyboardAndPointerControls();
  bindUIButtons();

  fetchFleetAndSync();
  setInterval(fetchFleetAndSync, 4000);

  // 15 Hz command publish loop (well inside MDD10's 500 ms safety watchdog)
  setInterval(teleopControlTick, 66);

  // 10 Hz telemetry & radar UI refresh
  setInterval(renderTeleopUI, 100);

  logTeleop('WASD Manual Drive ready. Select an online robot and hold W / A / S / D.', 'info');
});

function logTeleop(msg, level = 'info') {
  const box = document.getElementById('teleop-log-box');
  if (!box) return;
  const now = new Date().toLocaleTimeString();
  const row = document.createElement('div');
  row.className = `log-${level}`;
  row.textContent = `[${now}] ${msg}`;
  box.appendChild(row);
  while (box.children.length > 60) {
    box.removeChild(box.firstChild);
  }
  box.scrollTop = box.scrollHeight;
}

// ==============================================================================
// 3. FLEET DISCOVERY & FOXGLOVE WEBSOCKET CONNECTION
// ==============================================================================
async function fetchFleetAndSync() {
  try {
    const res = await fetch('/api/fleet');
    if (!res.ok) return;
    const data = await res.json();
    const robots = data.robots || {};
    fleetSnapshot = robots;

    if (data.base_station && typeof updateBaseStationLink === 'function') {
      updateBaseStationLink(data.base_station);
    }

    const select = document.getElementById('teleop-robot-select');
    let html = '';
    let firstOnline = null;
    let onlineCount = 0;

    for (let id = 1; id <= 13; id++) {
      const rob = robots[id];
      if (rob && rob.online && rob.ip) {
        onlineCount++;
        if (!firstOnline) firstOnline = id;
        const rNum = String(id).padStart(2, '0');
        html += `<option value="${id}">🤖 ${rob.name || `ConeRobot ${rNum}`} (${rob.ip}) [ONLINE]</option>`;

        realTelemetry[id].ip = rob.ip;
        const existingWs = activeWebSockets[id];
        const ipChanged = Boolean(existingWs && existingWs._robotIp && existingWs._robotIp !== rob.ip);
        const wsStale = Boolean(
          existingWs &&
          existingWs.readyState === WebSocket.OPEN &&
          existingWs._lastMsgTime &&
          (Date.now() - existingWs._lastMsgTime > 12000)
        );
        if (ipChanged || wsStale) {
          closeRobotWs(id);
          connectTeleopWebSocket(id, rob.ip, rob.port || 8765);
        } else if (!existingWs && !reconnectTimers[id]) {
          connectTeleopWebSocket(id, rob.ip, rob.port || 8765);
        }
      } else {
        closeRobotWs(id);
        if (realTelemetry[id]) realTelemetry[id].online = false;
      }
    }

    if (onlineCount === 0) {
      html = '<option value="">No online robots found — click Scan</option>';
    }

    if (select && select.innerHTML !== html) {
      select.innerHTML = html;
    }

    if (activeRobotId && robots[activeRobotId] && robots[activeRobotId].online) {
      if (select) select.value = String(activeRobotId);
    } else if (firstOnline) {
      selectTeleopRobot(firstOnline);
      if (select) select.value = String(firstOnline);
    } else {
      activeRobotId = null;
      selectedRobotId = null;
    }
  } catch (e) {}
}

function closeRobotWs(id) {
  if (reconnectTimers[id]) {
    clearTimeout(reconnectTimers[id]);
    delete reconnectTimers[id];
  }
  const ws = activeWebSockets[id];
  if (ws) {
    try {
      ws.onclose = null;
      ws.onerror = null;
      ws.onmessage = null;
      ws.close();
    } catch (e) {}
    delete activeWebSockets[id];
  }
}

function selectTeleopRobot(newId) {
  // Always halt previously selected robot before switching targets!
  if (activeRobotId && activeRobotId !== newId) {
    sendZeroTwist(activeRobotId);
  }
  pressedDirs.clear();
  updateKeyHighlights();
  activeRobotId = newId;
  selectedRobotId = newId;
  cancelledMissionOnTakeover = false;

  const rNum = String(newId).padStart(2, '0');
  const ip = fleetSnapshot[newId]?.ip || realTelemetry[newId]?.ip || '';
  logTeleop(`Selected ConeRobot ${rNum}${ip ? ` (${ip})` : ''} for manual WASD drive.`, 'success');
}

function connectTeleopWebSocket(id, ip, port) {
  const wsUrl = `ws://${ip}:${port}`;
  let ws;
  try {
    ws = new WebSocket(wsUrl, ['foxglove.sdk.v1', 'foxglove.websocket.v1']);
  } catch (e) {
    try { ws = new WebSocket(wsUrl); } catch (err) {
      return;
    }
  }

  ws.binaryType = 'arraybuffer';
  ws._robotIp = ip;
  ws._lastMsgTime = Date.now();
  activeWebSockets[id] = ws;

  const subIdToTopic = {};
  const channelIdToTopic = {};
  let subCounter = 1;

  ws.onopen = () => {
    ws._lastMsgTime = Date.now();
    if (!realTelemetry[id]) realTelemetry[id] = createEmptyRobotTelemetry(id);
    realTelemetry[id].online = true;
    realTelemetry[id].ip = ip;
    logTeleop(`🟢 Connected to ConeRobot ${String(id).padStart(2, '0')} (${ip}:${port})`, 'success');

    try {
      ws.send(JSON.stringify({ op: 'clientInfo', name: 'ConeRobotManualTeleop' }));
    } catch (e) {}

    // Advertise client publish channels: /cmd_vel (Twist) & /cmd_primitive (E-Stop cancel)
    try {
      ws.send(JSON.stringify({
        op: 'advertise',
        channels: [
          {
            id: TELEOP_CHANNELS.CMD_VEL,
            topic: '/cmd_vel',
            encoding: 'cdr',
            schemaName: 'geometry_msgs/msg/Twist'
          },
          {
            id: TELEOP_CHANNELS.CMD_PRIMITIVE,
            topic: '/cmd_primitive',
            encoding: 'cdr',
            schemaName: 'geometry_msgs/msg/Vector3'
          }
        ]
      }));
    } catch (e) {}
  };

  ws.onmessage = (event) => {
    ws._lastMsgTime = Date.now();
    if (realTelemetry[id]) realTelemetry[id].online = true;

    if (typeof event.data === 'string') {
      try {
        const msg = JSON.parse(event.data);
        if (msg.op === 'advertise' && Array.isArray(msg.channels)) {
          const targetTopics = [
            '/scan',
            '/odom',
            '/fix',
            '/gps/status',
            '/imu/heading',
            '/imu/data',
            '/step_status',
            '/battery_state',
            '/foxglove_bridge/sysinfo',
            '/robot/diagnostics',
            '/robot/system_info'
          ];
          const subs = [];
          const seen = new Set();
          msg.channels.forEach(ch => {
            const rawTopic = ch.topic || '';
            const topic = rawTopic.startsWith('/') ? rawTopic : '/' + rawTopic;
            if (targetTopics.includes(topic) && !seen.has(rawTopic)) {
              seen.add(rawTopic);
              const sid = subCounter++;
              subIdToTopic[sid] = rawTopic;
              channelIdToTopic[ch.id] = rawTopic;
              subs.push({ id: sid, channelId: ch.id });
            }
          });
          if (subs.length > 0) {
            ws.send(JSON.stringify({ op: 'subscribe', subscriptions: subs }));
          }
        } else if (msg.op === 'message' && msg.data) {
          const topic = subIdToTopic[msg.subscriptionId] || channelIdToTopic[msg.channelId];
          if (topic && typeof handleParsedRobotMessage === 'function') {
            handleParsedRobotMessage(id, topic, msg.data);
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
    logTeleop(`🔴 Disconnected from ConeRobot ${String(id).padStart(2, '0')}`, 'warn');
    if (!reconnectTimers[id]) {
      reconnectTimers[id] = setTimeout(() => {
        delete reconnectTimers[id];
        if (fleetSnapshot[id] && fleetSnapshot[id].online && fleetSnapshot[id].ip) {
          connectTeleopWebSocket(id, fleetSnapshot[id].ip, fleetSnapshot[id].port || 8765);
        }
      }, 3000);
    }
  };
}

// ==============================================================================
// 4. WASD COMMAND COMPUTATION & PUBLISHING LOOP (15 Hz)
// ==============================================================================
function computeTargetTwist() {
  let linAxis = 0;
  if (pressedDirs.has('w')) linAxis += 1;
  if (pressedDirs.has('s')) linAxis -= 1;

  let angAxis = 0;
  if (pressedDirs.has('a')) angAxis += 1; // ROS REP-103: +angular.z = CCW / Left
  if (pressedDirs.has('d')) angAxis -= 1; // -angular.z = CW / Right

  const boost = turboHeld ? 1.5 : 1.0;
  const maxLin = Math.min(0.95, manualLinearMax * boost);
  const maxAng = Math.min(2.80, manualAngularMax * boost);

  // When driving diagonally (e.g. W+A), gently soften turn rate for smooth curves
  const diagTurnScale = (linAxis !== 0 && angAxis !== 0) ? 0.75 : 1.0;

  return {
    linear: linAxis * maxLin,
    angular: angAxis * maxAng * diagTurnScale
  };
}

function sendTwist(robotId, linearX, angularZ) {
  const ws = activeWebSockets[robotId];
  if (!ws || ws.readyState !== WebSocket.OPEN) return false;
  try {
    ws.send(encodeTwistCDR(TELEOP_CHANNELS.CMD_VEL, linearX, angularZ));
    return true;
  } catch (e) {
    return false;
  }
}

function sendZeroTwist(robotId) {
  if (!robotId) return;
  sendTwist(robotId, 0.0, 0.0);
  setTimeout(() => sendTwist(robotId, 0.0, 0.0), 40);
}

function triggerManualEstop(logMsg = true) {
  pressedDirs.clear();
  turboHeld = false;
  updateKeyHighlights();
  lastCmdLinear = 0.0;
  lastCmdAngular = 0.0;
  wasMoving = false;

  if (!activeRobotId) return;
  const ws = activeWebSockets[activeRobotId];
  if (ws && ws.readyState === WebSocket.OPEN) {
    try {
      // Cancel any active autonomous waypoint primitive AND send immediate 0 Twist
      ws.send(encodeVector3CDR(TELEOP_CHANNELS.CMD_PRIMITIVE, 0.0, 0.0, 0.0));
      sendZeroTwist(activeRobotId);
    } catch (e) {}
  }

  const btn = document.getElementById('btn-teleop-estop');
  if (btn) {
    btn.classList.add('active');
    setTimeout(() => btn.classList.remove('active'), 180);
  }
  if (logMsg) {
    logTeleop(`🛑 STOP sent to ConeRobot ${String(activeRobotId).padStart(2, '0')}.`, 'error');
  }
}

function teleopControlTick() {
  const { linear, angular } = computeTargetTwist();
  const isDriving = (Math.abs(linear) > 1e-4 || Math.abs(angular) > 1e-4);

  lastCmdLinear = linear;
  lastCmdAngular = angular;

  const txEl = document.getElementById('teleop-tx-rate');

  if (!activeRobotId || !activeWebSockets[activeRobotId] || activeWebSockets[activeRobotId].readyState !== WebSocket.OPEN) {
    if (txEl) txEl.textContent = 'Offline';
    wasMoving = false;
    return;
  }

  if (isDriving) {
    // First manual input after connecting/switching cancels any autonomous waypoint
    if (!cancelledMissionOnTakeover) {
      cancelledMissionOnTakeover = true;
      try {
        activeWebSockets[activeRobotId].send(encodeVector3CDR(TELEOP_CHANNELS.CMD_PRIMITIVE, 0.0, 0.0, 0.0));
      } catch (e) {}
    }
    sendTwist(activeRobotId, linear, angular);
    wasMoving = true;
    if (txEl) txEl.textContent = '15 Hz TX';
  } else {
    if (wasMoving) {
      sendZeroTwist(activeRobotId);
      wasMoving = false;
    }
    if (txEl) txEl.textContent = 'Idle';
  }
}

// ==============================================================================
// 5. KEYBOARD, POINTER & SPEED CONTROLS
// ==============================================================================
function mapKeyToDir(e) {
  const k = (e.key || '').toLowerCase();
  if (k === 'w' || k === 'arrowup') return 'w';
  if (k === 's' || k === 'arrowdown') return 's';
  if (k === 'a' || k === 'arrowleft') return 'a';
  if (k === 'd' || k === 'arrowright') return 'd';
  return null;
}

function updateKeyHighlights() {
  ['w', 'a', 's', 'd'].forEach(dir => {
    const el = document.getElementById(`key-${dir}`);
    if (el) {
      if (pressedDirs.has(dir)) el.classList.add('active');
      else el.classList.remove('active');
    }
  });
}

function bindKeyboardAndPointerControls() {
  window.addEventListener('keydown', (e) => {
    // Ignore typing inside form inputs
    const tag = (e.target && e.target.tagName) ? e.target.tagName.toLowerCase() : '';
    if (tag === 'input' || tag === 'textarea') return;

    if (e.key === 'Shift') {
      turboHeld = true;
      return;
    }
    if (e.code === 'Space' || e.key === ' ') {
      e.preventDefault();
      triggerManualEstop(true);
      return;
    }
    if ((e.key || '').toLowerCase() === 'q') {
      e.preventDefault();
      nudgeSpeed(-0.05, -0.15);
      return;
    }
    if ((e.key || '').toLowerCase() === 'e') {
      e.preventDefault();
      nudgeSpeed(0.05, 0.15);
      return;
    }

    const dir = mapKeyToDir(e);
    if (dir) {
      e.preventDefault();
      // Prevent select dropdown from stealing arrow keys
      if (document.activeElement && document.activeElement.tagName.toLowerCase() === 'select') {
        document.activeElement.blur();
      }
      if (!pressedDirs.has(dir)) {
        pressedDirs.add(dir);
        updateKeyHighlights();
      }
    }
  });

  window.addEventListener('keyup', (e) => {
    if (e.key === 'Shift') {
      turboHeld = false;
      return;
    }
    const dir = mapKeyToDir(e);
    if (dir && pressedDirs.has(dir)) {
      pressedDirs.delete(dir);
      updateKeyHighlights();
    }
  });

  // Safety deadman: release all keys & send immediate stop if window loses focus
  const releaseAllSafe = () => {
    if (pressedDirs.size > 0 || wasMoving) {
      pressedDirs.clear();
      turboHeld = false;
      updateKeyHighlights();
      if (activeRobotId) sendZeroTwist(activeRobotId);
      wasMoving = false;
    }
  };
  window.addEventListener('blur', releaseAllSafe);
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) releaseAllSafe();
  });
  window.addEventListener('beforeunload', releaseAllSafe);

  // On-screen WASD hold buttons (Mouse & Touch)
  document.querySelectorAll('.drive-key[data-dir]').forEach(btn => {
    const dir = btn.getAttribute('data-dir');
    const onPress = (e) => {
      e.preventDefault();
      pressedDirs.add(dir);
      updateKeyHighlights();
    };
    const onRelease = (e) => {
      e.preventDefault();
      pressedDirs.delete(dir);
      updateKeyHighlights();
    };
    btn.addEventListener('pointerdown', onPress);
    btn.addEventListener('pointerup', onRelease);
    btn.addEventListener('pointerleave', onRelease);
    btn.addEventListener('pointercancel', onRelease);
  });
}

function initSpeedControls() {
  const linSlider = document.getElementById('slider-linear');
  const angSlider = document.getElementById('slider-angular');

  manualLinearMax = Math.max(0.10, Math.min(0.80, manualLinearMax));
  manualAngularMax = Math.max(0.30, Math.min(2.50, manualAngularMax));

  if (linSlider) {
    linSlider.value = manualLinearMax.toFixed(2);
    linSlider.addEventListener('input', (e) => {
      setSpeeds(parseFloat(e.target.value), manualAngularMax);
    });
  }
  if (angSlider) {
    angSlider.value = manualAngularMax.toFixed(2);
    angSlider.addEventListener('input', (e) => {
      setSpeeds(manualLinearMax, parseFloat(e.target.value));
    });
  }

  document.querySelectorAll('.preset-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const lin = parseFloat(btn.getAttribute('data-lin'));
      const ang = parseFloat(btn.getAttribute('data-ang'));
      setSpeeds(lin, ang);
    });
  });

  updateSpeedLabels();
}

function setSpeeds(lin, ang) {
  manualLinearMax = Math.max(0.10, Math.min(0.80, Number(lin) || 0.30));
  manualAngularMax = Math.max(0.30, Math.min(2.50, Number(ang) || 1.10));
  localStorage.setItem('conerobot_teleop_lin', manualLinearMax.toFixed(2));
  localStorage.setItem('conerobot_teleop_ang', manualAngularMax.toFixed(2));
  const linSlider = document.getElementById('slider-linear');
  const angSlider = document.getElementById('slider-angular');
  if (linSlider) linSlider.value = manualLinearMax.toFixed(2);
  if (angSlider) angSlider.value = manualAngularMax.toFixed(2);
  updateSpeedLabels();
}

function nudgeSpeed(dLin, dAng) {
  setSpeeds(manualLinearMax + dLin, manualAngularMax + dAng);
  logTeleop(`Speed set -> Linear ${manualLinearMax.toFixed(2)} m/s, Turn ${manualAngularMax.toFixed(2)} rad/s`, 'info');
}

function updateSpeedLabels() {
  const lLbl = document.getElementById('slider-linear-label');
  const aLbl = document.getElementById('slider-angular-label');
  if (lLbl) lLbl.textContent = `${manualLinearMax.toFixed(2)} m/s`;
  if (aLbl) aLbl.textContent = `${manualAngularMax.toFixed(2)} rad/s`;

  document.querySelectorAll('.preset-btn').forEach(btn => {
    const lin = parseFloat(btn.getAttribute('data-lin'));
    const ang = parseFloat(btn.getAttribute('data-ang'));
    const isMatch = Math.abs(lin - manualLinearMax) < 0.015 && Math.abs(ang - manualAngularMax) < 0.05;
    btn.classList.toggle('active', isMatch);
  });
}

function bindUIButtons() {
  const select = document.getElementById('teleop-robot-select');
  if (select) {
    select.addEventListener('change', (e) => {
      const id = parseInt(e.target.value, 10);
      if (id) selectTeleopRobot(id);
      select.blur(); // Return keyboard focus so WASD doesn't change dropdown
    });
  }

  const estopBtn = document.getElementById('btn-teleop-estop');
  if (estopBtn) {
    estopBtn.addEventListener('click', () => triggerManualEstop(true));
  }

  const scanBtn = document.getElementById('btn-teleop-scan');
  if (scanBtn) {
    scanBtn.addEventListener('click', async () => {
      scanBtn.disabled = true;
      scanBtn.textContent = 'Scanning...';
      logTeleop('Scanning local network for active ConeRobots...', 'info');
      try {
        await fetch('/api/scan');
        setTimeout(() => {
          fetchFleetAndSync();
          scanBtn.disabled = false;
          scanBtn.textContent = '🔄 Scan';
        }, 2000);
      } catch (e) {
        scanBtn.disabled = false;
        scanBtn.textContent = '🔄 Scan';
      }
    });
  }

  const clearBtn = document.getElementById('btn-clear-teleop-log');
  if (clearBtn) {
    clearBtn.addEventListener('click', () => {
      const box = document.getElementById('teleop-log-box');
      if (box) box.innerHTML = '';
    });
  }

  const lidarBtn = document.getElementById('btn-teleop-lidar-toggle');
  if (lidarBtn) {
    lidarBtn.addEventListener('click', () => {
      lidarRadarOpen = !lidarRadarOpen;
      const wrap = document.getElementById('teleop-lidar-wrap');
      const hint = document.getElementById('teleop-lidar-hint');
      if (wrap) wrap.style.display = lidarRadarOpen ? 'block' : 'none';
      if (hint) hint.style.display = lidarRadarOpen ? 'none' : 'block';
      lidarBtn.textContent = lidarRadarOpen ? 'Hide Radar' : 'Show Radar';
    });
  }
}

// ==============================================================================
// 6. TELEMETRY & GAUGE RENDERING (10 Hz)
// ==============================================================================
function renderTrackMeter(barId, valId, wheelSpeedMps) {
  const bar = document.getElementById(barId);
  const val = document.getElementById(valId);
  const clamped = Math.max(-1.0, Math.min(1.0, wheelSpeedMps));
  const pct = Math.round(clamped * 100);
  if (val) val.textContent = `${pct >= 0 ? '+' : ''}${pct}%`;
  if (bar) {
    const halfH = Math.abs(clamped) * 50.0;
    if (clamped >= 0) {
      bar.style.top = `${50.0 - halfH}%`;
      bar.style.height = `${halfH}%`;
      bar.style.background = '#10b981';
    } else {
      bar.style.top = '50%';
      bar.style.height = `${halfH}%`;
      bar.style.background = '#f59e0b';
    }
  }
}

function renderTeleopUI() {
  // Commanded velocity meters
  const linEl = document.getElementById('cmd-linear-val');
  const angEl = document.getElementById('cmd-angular-val');
  const turboEl = document.getElementById('cmd-turbo-val');
  const driveStateEl = document.getElementById('teleop-drive-state');

  if (linEl) linEl.textContent = `${lastCmdLinear >= 0 ? '+' : ''}${lastCmdLinear.toFixed(2)} m/s`;
  if (angEl) angEl.textContent = `${lastCmdAngular >= 0 ? '+' : ''}${lastCmdAngular.toFixed(2)} rad/s`;
  if (turboEl) {
    turboEl.textContent = turboHeld ? '1.5× BOOST' : 'OFF';
    turboEl.style.color = turboHeld ? '#fbbf24' : '#94a3b8';
  }

  const isMovingCmd = (Math.abs(lastCmdLinear) > 1e-4 || Math.abs(lastCmdAngular) > 1e-4);
  if (driveStateEl) {
    driveStateEl.textContent = isMovingCmd ? 'DRIVING' : 'STOPPED';
    driveStateEl.className = isMovingCmd ? 'state-badge driving' : 'state-badge idle';
  }

  // Track kinematics matching MDD10 (L_eff = 0.380 m, max_linear_speed = 1.00 m/s)
  const vLeft = lastCmdLinear - (lastCmdAngular * 0.19);
  const vRight = lastCmdLinear + (lastCmdAngular * 0.19);
  renderTrackMeter('bar-track-left', 'val-track-left', vLeft);
  renderTrackMeter('bar-track-right', 'val-track-right', vRight);

  // Selected robot telemetry
  const r = activeRobotId ? realTelemetry[activeRobotId] : null;
  const wsOpen = Boolean(activeRobotId && activeWebSockets[activeRobotId] && activeWebSockets[activeRobotId].readyState === WebSocket.OPEN);
  const isOnline = Boolean(r && r.online && wsOpen);

  const hdrDot = document.getElementById('teleop-header-dot');
  const hdrRob = document.getElementById('teleop-header-robot');
  const connBadge = document.getElementById('teleop-conn-badge');
  const motBadge = document.getElementById('teleop-motion-badge');

  if (hdrDot) hdrDot.className = isOnline ? 'stat-dot green' : 'stat-dot red';
  if (hdrRob) hdrRob.textContent = activeRobotId ? `ConeRobot ${String(activeRobotId).padStart(2, '0')}` : 'None';

  if (connBadge) {
    connBadge.textContent = isOnline ? `ONLINE (${r.ip})` : 'OFFLINE';
    connBadge.className = isOnline ? 'state-badge idle' : 'state-badge error';
  }
  if (motBadge) {
    motBadge.textContent = isOnline ? (isMovingCmd ? 'MANUAL' : 'STANDBY') : 'OFFLINE';
    motBadge.className = isOnline ? (isMovingCmd ? 'state-badge driving' : 'state-badge idle') : 'state-badge error';
  }

  const setText = (id, txt, color = '') => {
    const el = document.getElementById(id);
    if (!el) return;
    if (el.textContent !== txt) el.textContent = txt;
    if (el.style.color !== color) el.style.color = color;
  };

  if (!isOnline) {
    setText('tel-ip', '--');
    setText('tel-wifi', '--');
    setText('tel-heading', '--°');
    setText('tel-gps', '--');
    setText('tel-obstacle', '--');
    setText('tel-battery', '--');
    setText('tel-sys', '--');
    return;
  }

  setText('tel-ip', r.ip || '--');

  // Wi-Fi Signal
  const w = r.wifi;
  if (w && w.rssi_dbm !== null && !isNaN(w.rssi_dbm)) {
    const rssi = Number(w.rssi_dbm);
    const col = rssi >= -65 ? '#10b981' : (rssi >= -75 ? '#d97706' : '#ef4444');
    const icon = rssi >= -65 ? '🟢' : (rssi >= -75 ? '🟡' : '🔴');
    const qStr = (w.quality_pct !== null && w.quality_pct !== undefined) ? ` • ${w.quality_pct}%` : '';
    setText('tel-wifi', `${icon} ${Math.round(rssi)} dBm (${w.iface || 'wifi'}${qStr})`, col);
  } else if (w && w.iface) {
    setText('tel-wifi', `🌐 ${w.iface}`, '#0284c7');
  } else {
    setText('tel-wifi', 'Waiting...');
  }

  // Heading
  const hdg = (r.imu && r.imu.heading !== null && !isNaN(r.imu.heading)) ? `${Number(r.imu.heading).toFixed(1)}°` : '--°';
  setText('tel-heading', hdg);

  // GPS
  if (r.gps && r.gps.present) {
    const col = r.gps.mode === 'RTK FIX' ? '#10b981' : (r.gps.mode === 'FLOAT' ? '#d97706' : '#ef4444');
    setText('tel-gps', `${r.gps.mode} (${r.gps.sats || 0} Sats)`, col);
  } else {
    setText('tel-gps', 'Not Equipped', '#94a3b8');
  }

  // Nearest LiDAR Obstacle
  if (r.lidar && r.lidar.present && r.lidar.min_dist !== null) {
    const d = r.lidar.min_dist;
    const col = d < 0.5 ? '#ef4444' : (d < 1.2 ? '#d97706' : '#10b981');
    const angStr = r.lidar.min_angle_deg !== null ? ` @ ${r.lidar.min_angle_deg}°` : '';
    setText('tel-obstacle', `${d.toFixed(2)} m${angStr}`, col);
  } else if (r.lidar && r.lidar.present) {
    setText('tel-obstacle', 'Scanning...');
  } else {
    setText('tel-obstacle', 'No LiDAR', '#94a3b8');
  }

  // Battery
  if (r.health && r.health.battery_v && r.health.battery_v !== '--') {
    setText('tel-battery', r.health.battery_v, '#10b981');
  } else {
    setText('tel-battery', 'No Sensor', '#94a3b8');
  }

  // Pi 5 CPU / Temp
  const cpuStr = (r.health && r.health.cpu_percent && r.health.cpu_percent !== '--') ? r.health.cpu_percent : null;
  const tempStr = (r.health && r.health.cpu_temp && r.health.cpu_temp !== '--') ? r.health.cpu_temp : null;
  setText('tel-sys', [cpuStr, tempStr].filter(Boolean).join(' • ') || '--');

  // Optional 2D LiDAR Radar preview
  if (lidarRadarOpen && typeof renderLidarRadarCanvas === 'function') {
    renderLidarRadarCanvas(r);
  }
}
