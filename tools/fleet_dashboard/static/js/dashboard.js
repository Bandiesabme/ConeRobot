/* ==========================================================================
   ConeRobot Fleet Dashboard - Module 6: UI Orchestration, Cards & Detail Modal
   ========================================================================== */

document.addEventListener('DOMContentLoaded', () => {
  initMap();
  initCardsGridDOM();
  initEventListeners();
  startDiscoveryLoop();

  // Smooth, throttled UI update loop (10 FPS)
  setInterval(renderUIUpdates, 100);
});

// --- 1. Card Grid DOM Initialization (Built Once) ---
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
          <span class="metric-key">GNSS Signal (Used)</span>
          <span class="metric-val" id="r${i}-snr">--</span>
        </div>
        <div class="metric-row">
          <span class="metric-key">Wi-Fi Signal</span>
          <span class="metric-val" id="r${i}-wifi">--</span>
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
    realTelemetry[i] = createEmptyRobotTelemetry(i);
  }
}

// --- 2. High Performance UI Update Engine (10 FPS) ---
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
    const snrEl = document.getElementById(`r${i}-snr`);
    const wifiEl = document.getElementById(`r${i}-wifi`);
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

      // LiDAR & Odom Metric
      if (r.lidar.present) {
        const odomSnippet = (r.lidar.odom_x !== null && r.lidar.odom_y !== null)
          ? ` [${r.lidar.odom_x.toFixed(2)}m, ${r.lidar.odom_y.toFixed(2)}m]`
          : '';
        if (r.lidar.freq_hz > 0) {
          lidarEl.innerText = `🟢 Active (${r.lidar.freq_hz} Hz)${odomSnippet}`;
          lidarEl.className = 'metric-val gps-rtk';
        } else if (r.lidar.odom_present) {
          lidarEl.innerText = `🟢 Odom Active${odomSnippet}`;
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
        let gpsTxt = '';
        const hasValidCoords = (r.gps.lat !== null && r.gps.lon !== null && !isNaN(r.gps.lat) && !isNaN(r.gps.lon) && (Math.abs(r.gps.lat) > 0.0001 || Math.abs(r.gps.lon) > 0.0001));
        const coordSnippet = hasValidCoords ? ` [${r.gps.lat.toFixed(5)}, ${r.gps.lon.toFixed(5)}]` : '';
        if (r.gps.mode === 'RTK FIX') {
          gpsTxt = `🟢 RTK FIX (${r.gps.sats} Sats)${coordSnippet}`;
          gpsEl.className = 'metric-val gps-rtk';
        } else if (r.gps.mode === 'FLOAT') {
          gpsTxt = `🟡 RTK FLOAT (${r.gps.sats} Sats)${coordSnippet}`;
          gpsEl.className = 'metric-val gps-float';
        } else if (r.gps.mode === '3D FIX' || (r.gps.mode && r.gps.mode.includes('3D'))) {
          const isBaseConnected = (r.gps.ntrip && r.gps.ntrip.toLowerCase().includes('connected'));
          gpsTxt = isBaseConnected
            ? `3D FIX (${r.gps.sats} Sats)${coordSnippet}`
            : `3D FIX [${r.gps.sats} Sats]${coordSnippet}`;
          gpsEl.className = 'metric-val gps-none';
        } else {
          gpsTxt = `${r.gps.mode} (${r.gps.sats} Sats)${coordSnippet}`;
          gpsEl.className = 'metric-val gps-none';
        }
        gpsEl.innerText = gpsTxt;
        if (snrEl) {
          if (r.gps.avg_snr !== null && !isNaN(r.gps.avg_snr)) {
            const avg = r.gps.avg_snr;
            const totalSats = (r.gps.total_signals > 0)
              ? r.gps.total_signals
              : ((r.gps.snr_sats && r.gps.snr_sats.length > 0) ? r.gps.snr_sats.length : (r.gps.sats || 0));
            const usedCnt = (r.gps.avg_snr_count && r.gps.avg_snr_count > 0) ? r.gps.avg_snr_count : totalSats;
            const icon = avg >= 38 ? '🟢' : (avg >= 28 ? '🟡' : '🔴');
            const satLabel = (r.gps.avg_snr_mode === 'used')
              ? `${usedCnt}/${totalSats} Used`
              : (r.gps.avg_snr_mode === 'top' ? `Top ${usedCnt}/${totalSats}` : `${totalSats} Sats`);
            snrEl.innerText = `${icon} ${avg.toFixed(1)} dB-Hz (${satLabel})`;
            snrEl.className = avg >= 38 ? 'metric-val gps-rtk' : (avg >= 28 ? 'metric-val gps-float' : 'metric-val gps-none');
            snrEl.style.color = '';
          } else {
            snrEl.innerText = 'Waiting GSV...';
            snrEl.className = 'metric-val';
            snrEl.style.color = 'var(--text-muted)';
          }
        }
      } else {
        gpsEl.innerText = 'Not Equipped';
        gpsEl.className = 'metric-val';
        gpsEl.style.color = 'var(--text-muted)';
        if (snrEl) {
          snrEl.innerText = 'Not Equipped';
          snrEl.className = 'metric-val';
          snrEl.style.color = 'var(--text-muted)';
        }
      }

      // Wi-Fi Signal Metric (from /robot/diagnostics /proc/net/wireless, 0 MB extra RAM)
      if (wifiEl) {
        const w = r.wifi;
        if (w && w.rssi_dbm !== null && !isNaN(w.rssi_dbm)) {
          const rssi = w.rssi_dbm;
          const icon = rssi >= -65 ? '🟢' : (rssi >= -75 ? '🟡' : '🔴');
          const ifLabel = w.iface || 'wifi';
          wifiEl.innerText = `${icon} ${Math.round(rssi)} dBm (${ifLabel})`;
          wifiEl.className = rssi >= -65 ? 'metric-val gps-rtk' : (rssi >= -75 ? 'metric-val gps-float' : 'metric-val gps-none');
          wifiEl.style.color = '';
        } else if (w && w.iface) {
          wifiEl.innerText = `🌐 ${w.iface}`;
          wifiEl.className = 'metric-val';
          wifiEl.style.color = '#38bdf8';
        } else {
          wifiEl.innerText = 'Waiting...';
          wifiEl.className = 'metric-val';
          wifiEl.style.color = 'var(--text-muted)';
        }
      }

      headEl.innerText = (r.imu.heading !== null) ? `${Number(r.imu.heading).toFixed(1)}°` : 'Waiting for IMU...';
      stepEl.innerText = r.motion.step_progress;
      pingEl.innerText = '🟢 Connected';

      const battEl = document.getElementById(`r${i}-battery`);
      if (battEl) {
        if (r.health.battery_v && r.health.battery_v !== '--') {
          battEl.innerText = r.health.battery_v;
          battEl.style.color = '#10b981';
        } else if (r.health.cpu_temp !== '--' && r.health.cpu_load && r.health.cpu_load !== '--') {
          battEl.innerText = `${r.health.cpu_temp} (${r.health.cpu_load})`;
          battEl.style.color = 'var(--text-muted)';
        } else if (r.health.cpu_temp !== '--') {
          battEl.innerText = `${r.health.cpu_temp}`;
          battEl.style.color = 'var(--text-muted)';
        } else {
          battEl.innerText = 'No Sensor';
          battEl.style.color = 'var(--text-muted)';
        }
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
      if (snrEl) {
        snrEl.innerText = '--';
        snrEl.className = 'metric-val';
      }
      if (wifiEl) {
        wifiEl.innerText = '--';
        wifiEl.className = 'metric-val';
        wifiEl.style.color = '';
      }
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

// --- 3. Detail Modal & Collapsible Signal Flaps ---
let wifiPingTimer = null;
let wifiPingInFlight = false;
let lastWifiBarsRenderKey = '';

function syncModalFlapVisibility() {
  const snrBody = document.getElementById('modal-snr-body');
  const snrBtn = document.getElementById('btn-toggle-snr-panel');
  if (snrBody) snrBody.style.display = snrPanelOpen ? 'block' : 'none';
  if (snrBtn) snrBtn.innerText = snrPanelOpen ? '▲ Hide' : '▼ Open';

  const wifiBody = document.getElementById('modal-wifi-body');
  const wifiBtn = document.getElementById('btn-toggle-wifi-panel');
  if (wifiBody) wifiBody.style.display = wifiPanelOpen ? 'block' : 'none';
  if (wifiBtn) wifiBtn.innerText = wifiPanelOpen ? '▲ Hide' : '▼ Open';
}

function toggleSnrPanel() {
  snrPanelOpen = !snrPanelOpen;
  syncModalFlapVisibility();
  if (selectedRobotId && realTelemetry[selectedRobotId]) {
    updateModalContent(realTelemetry[selectedRobotId]);
  }
}

function toggleWifiPanel() {
  wifiPanelOpen = !wifiPanelOpen;
  lastWifiBarsRenderKey = '';
  syncModalFlapVisibility();
  if (wifiPanelOpen && selectedRobotId) {
    startWifiPingLoop();
  } else {
    stopWifiPingLoop();
  }
  if (selectedRobotId && realTelemetry[selectedRobotId]) {
    updateModalContent(realTelemetry[selectedRobotId]);
  }
}

function recomputeWifiPingMetrics(w) {
  if (!w || !Array.isArray(w.ping_samples) || w.ping_samples.length === 0) {
    if (w) {
      w.ping_ms = null;
      w.ping_avg_ms = null;
      w.ping_jitter_ms = null;
      w.ping_loss_pct = null;
    }
    return;
  }
  const total = w.ping_samples.length;
  const okSamples = w.ping_samples.filter(s => s.ok && s.ms !== null && !isNaN(s.ms)).map(s => s.ms);
  const lost = total - okSamples.length;
  w.ping_loss_pct = Number(((lost / total) * 100.0).toFixed(1));

  if (okSamples.length > 0) {
    const sum = okSamples.reduce((a, b) => a + b, 0);
    w.ping_avg_ms = Number((sum / okSamples.length).toFixed(1));
    if (okSamples.length >= 2) {
      let diffSum = 0;
      for (let i = 1; i < okSamples.length; i++) {
        diffSum += Math.abs(okSamples[i] - okSamples[i - 1]);
      }
      w.ping_jitter_ms = Number((diffSum / (okSamples.length - 1)).toFixed(1));
    } else {
      w.ping_jitter_ms = 0.0;
    }
  } else {
    w.ping_avg_ms = null;
    w.ping_jitter_ms = null;
  }
}

async function pollSingleWifiPing(robotId) {
  if (wifiPingInFlight) return;
  const r = realTelemetry[robotId];
  if (!r || !r.online || !r.ip || !r.wifi) return;

  wifiPingInFlight = true;
  try {
    const res = await fetch(`/api/ping?ip=${encodeURIComponent(r.ip)}&count=1`);
    if (res.ok) {
      const data = await res.json();
      if (realTelemetry[robotId] && realTelemetry[robotId].wifi) {
        const w = realTelemetry[robotId].wifi;
        const ok = Boolean(data.ok && data.rtt_ms !== null && !isNaN(data.rtt_ms));
        const ms = ok ? Number(data.rtt_ms) : null;
        w.ping_ms = ms;
        if (!Array.isArray(w.ping_samples)) w.ping_samples = [];
        w.ping_samples.push({ ok, ms, ts: Date.now() });
        if (w.ping_samples.length > 30) w.ping_samples.shift();
        recomputeWifiPingMetrics(w);
      }
    }
  } catch (e) {
    // Ignore transient dashboard server errors
  } finally {
    wifiPingInFlight = false;
  }
}

function startWifiPingLoop() {
  stopWifiPingLoop();
  if (!selectedRobotId) return;
  pollSingleWifiPing(selectedRobotId);
  wifiPingTimer = setInterval(() => {
    if (selectedRobotId && wifiPanelOpen) {
      pollSingleWifiPing(selectedRobotId);
    }
  }, 2000);
}

function stopWifiPingLoop() {
  if (wifiPingTimer) {
    clearInterval(wifiPingTimer);
    wifiPingTimer = null;
  }
}

async function runWifiBurstTest() {
  if (!selectedRobotId || !realTelemetry[selectedRobotId]) return;
  const r = realTelemetry[selectedRobotId];
  if (!r.online || !r.ip) return;

  const btn = document.getElementById('btn-wifi-burst');
  const statusEl = document.getElementById('modal-wifi-burst-status');
  if (btn) {
    btn.disabled = true;
    btn.innerText = '⚡ Pinging (10x)...';
  }
  if (statusEl) {
    statusEl.innerText = 'Running 10-packet laptop → Pi ICMP burst...';
    statusEl.style.color = '#38bdf8';
  }

  try {
    const res = await fetch(`/api/ping?ip=${encodeURIComponent(r.ip)}&count=10`);
    if (res.ok) {
      const data = await res.json();
      const w = r.wifi;
      if (w) {
        if (Array.isArray(data.samples)) {
          if (!Array.isArray(w.ping_samples)) w.ping_samples = [];
          data.samples.forEach(val => {
            const ok = val !== null && !isNaN(val);
            w.ping_samples.push({ ok, ms: ok ? Number(val) : null, ts: Date.now() });
            if (w.ping_samples.length > 30) w.ping_samples.shift();
          });
          if (data.rtt_ms !== null && data.rtt_ms !== undefined) {
            w.ping_ms = Number(data.rtt_ms);
          }
          recomputeWifiPingMetrics(w);
        }
        w.burst_result = data;
      }
      if (statusEl) {
        if (data.ok) {
          statusEl.innerText = `Burst (${data.received}/${data.sent}): min ${data.min_ms} / avg ${data.avg_ms} / max ${data.max_ms} ms • ±${data.jitter_ms} ms • ${data.loss_pct}% loss`;
          statusEl.style.color = data.loss_pct > 0 ? '#fbbf24' : '#34d399';
        } else {
          statusEl.innerText = `Burst failed: 0/${data.sent || 10} replies (100% loss)`;
          statusEl.style.color = '#f87171';
        }
      }
      if (selectedRobotId === r.id) {
        updateModalContent(r);
      }
    }
  } catch (e) {
    if (statusEl) {
      statusEl.innerText = 'Burst error — check dashboard server';
      statusEl.style.color = '#f87171';
    }
  } finally {
    if (btn) {
      btn.disabled = false;
      btn.innerText = '⚡ 10-Ping Burst';
    }
  }
}

function pinWifiBaseline() {
  if (!selectedRobotId || !realTelemetry[selectedRobotId]) return;
  const r = realTelemetry[selectedRobotId];
  const w = r.wifi;
  if (!w) return;

  const recentRssi = (w.rssi_history || []).slice(-10).map(s => s.rssi);
  const avgRssi = recentRssi.length > 0
    ? Number((recentRssi.reduce((a, b) => a + b, 0) / recentRssi.length).toFixed(1))
    : w.rssi_dbm;

  wifiAbBaselines[selectedRobotId] = {
    iface: w.iface || 'unknown',
    is_usb: w.is_usb,
    avg_rssi: avgRssi,
    avg_ping: w.ping_avg_ms !== null ? w.ping_avg_ms : w.ping_ms,
    loss_pct: w.ping_loss_pct !== null ? w.ping_loss_pct : 0.0,
    ts: Date.now()
  };

  updateModalContent(r);
}

function resetWifiHistory() {
  if (!selectedRobotId || !realTelemetry[selectedRobotId]) return;
  const r = realTelemetry[selectedRobotId];
  if (r.wifi) {
    r.wifi.rssi_history = [];
    r.wifi.ping_samples = [];
    r.wifi.ping_avg_ms = null;
    r.wifi.ping_jitter_ms = null;
    r.wifi.ping_loss_pct = null;
    r.wifi.burst_result = null;
  }
  delete wifiAbBaselines[selectedRobotId];
  lastWifiBarsRenderKey = '';

  const statusEl = document.getElementById('modal-wifi-burst-status');
  if (statusEl) {
    statusEl.innerText = 'Source: /robot/system_info (0.5 Hz)';
    statusEl.style.color = '#94a3b8';
  }
  updateModalContent(r);
}

function openRobotDetailModal(robotId) {
  selectedRobotId = robotId;
  lastWifiBarsRenderKey = '';
  const modal = document.getElementById('robot-detail-modal');
  modal.classList.remove('hidden');
  syncModalFlapVisibility();

  const statusEl = document.getElementById('modal-wifi-burst-status');
  const r = realTelemetry[robotId];
  if (statusEl) {
    const br = r && r.wifi ? r.wifi.burst_result : null;
    if (br && br.ok) {
      statusEl.innerText = `Burst (${br.received}/${br.sent}): min ${br.min_ms} / avg ${br.avg_ms} / max ${br.max_ms} ms • ±${br.jitter_ms} ms • ${br.loss_pct}% loss`;
      statusEl.style.color = br.loss_pct > 0 ? '#fbbf24' : '#34d399';
    } else {
      statusEl.innerText = 'Source: /robot/system_info (0.5 Hz)';
      statusEl.style.color = '#94a3b8';
    }
  }

  if (r) {
    updateModalContent(r);
    if (wifiPanelOpen) {
      startWifiPingLoop();
    } else if (r.online && r.ip) {
      // Single lightweight ping on modal open so collapsed flap header shows live RTT
      pollSingleWifiPing(robotId);
    }
  }
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

  const topPingMs = (r.wifi && r.wifi.ping_ms !== null && !isNaN(r.wifi.ping_ms))
    ? ` (${r.wifi.ping_ms} ms)`
    : '';
  document.getElementById('modal-ping').innerText = r.online ? `Connected${topPingMs}` : 'Offline';

  // LiDAR & Odometry Widget in Modal
  const odomPosEl = document.getElementById('modal-odom-pos');
  const odomDistEl = document.getElementById('modal-odom-dist');
  if (r.lidar.present) {
    const isScanActive = (r.lidar.freq_hz > 0);
    document.getElementById('modal-lidar-status').innerText = isScanActive ? '🟢 Active & Streaming' : '🟡 Topic Present';
    document.getElementById('modal-lidar-status').style.color = isScanActive ? '#10b981' : '#f59e0b';
    document.getElementById('modal-lidar-freq').innerText = r.lidar.freq_hz > 0 ? `${r.lidar.freq_hz} Hz` : 'Waiting stream...';
    if (r.lidar.odom_x !== null && r.lidar.odom_y !== null) {
      if (odomPosEl) odomPosEl.innerText = `X: ${r.lidar.odom_x.toFixed(3)} m, Y: ${r.lidar.odom_y.toFixed(3)} m`;
      const distOrig = Math.hypot(r.lidar.odom_x, r.lidar.odom_y);
      if (odomDistEl) odomDistEl.innerText = `${distOrig.toFixed(3)} m (${(distOrig * 100).toFixed(1)} cm)`;
    } else {
      if (odomPosEl) odomPosEl.innerText = r.lidar.odom_present ? 'Waiting pose...' : 'No /odom stream';
      if (odomDistEl) odomDistEl.innerText = '--';
    }
  } else {
    document.getElementById('modal-lidar-status').innerText = '⚪ Not Equipped';
    document.getElementById('modal-lidar-status').style.color = 'var(--text-muted)';
    document.getElementById('modal-lidar-freq').innerText = 'N/A';
    if (odomPosEl) odomPosEl.innerText = 'N/A';
    if (odomDistEl) odomDistEl.innerText = 'N/A';
  }

  // Render 2D LiDAR Radar Point Cloud ONLY if user has opened the collapsible panel
  if (lidarRadarOpen) {
    renderLidarRadarCanvas(r);
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

  const pitchEl = document.getElementById('modal-pitch');
  if (pitchEl) pitchEl.innerText = (r.imu.pitch !== null && r.imu.pitch !== undefined) ? `${r.imu.pitch}°` : '--';

  const rollEl = document.getElementById('modal-roll');
  if (rollEl) rollEl.innerText = (r.imu.roll !== null && r.imu.roll !== undefined) ? `${r.imu.roll}°` : '--';

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

  const battModalEl = document.getElementById('modal-battery');
  if (battModalEl) {
    battModalEl.innerText = (r.health.battery_v && r.health.battery_v !== '--') ? r.health.battery_v : 'No Sensor';
    battModalEl.style.color = (r.health.battery_v && r.health.battery_v !== '--') ? '#10b981' : 'var(--text-muted)';
  }

  const hasLat = (r.gps.lat !== null && r.gps.lat !== undefined && !isNaN(r.gps.lat));
  const hasLon = (r.gps.lon !== null && r.gps.lon !== undefined && !isNaN(r.gps.lon));
  document.getElementById('modal-lat').innerText = hasLat ? r.gps.lat.toFixed(7) : (r.gps.present ? 'NO FIX' : 'Not Equipped');
  document.getElementById('modal-lon').innerText = hasLon ? r.gps.lon.toFixed(7) : (r.gps.present ? 'NO FIX' : 'Not Equipped');
  document.getElementById('modal-alt').innerText = (r.gps.alt !== null && r.gps.alt !== undefined && !isNaN(r.gps.alt) && r.gps.alt !== '--') ? `${r.gps.alt} m` : '--';
  document.getElementById('modal-hdop').innerText = r.gps.hdop || '--';

  const diffAgeEl = document.getElementById('modal-diff-age');
  if (diffAgeEl) {
    diffAgeEl.innerText = r.gps.diff_age || '--';
  }

  const ntripEl = document.getElementById('modal-ntrip');
  if (ntripEl) {
    if (r.gps.mode === 'RTK FIX' || r.gps.mode === 'FLOAT' || (r.gps.ntrip && r.gps.ntrip.toLowerCase().includes('connected'))) {
      ntripEl.innerText = `🟢 Connected (${r.gps.rtcm || 'Active'})`;
      ntripEl.style.color = '#10b981';
    } else {
      ntripEl.innerText = '🔴 Disconnected (No Base Station Connected)';
      ntripEl.style.color = '#ef4444';
    }
  }

  // Render Rover GNSS Satellite SNR (C/N0) Panel
  const snrCard = document.getElementById('modal-snr-card');
  if (snrCard) {
    snrCard.style.display = r.gps.present ? 'block' : 'none';
    const avg = r.gps.avg_snr;
    const top16 = r.gps.top16_snr;
    const sats = r.gps.snr_sats || [];
    const totalVisible = (r.gps.total_signals > 0)
      ? r.gps.total_signals
      : (sats.length > 0 ? sats.length : (r.gps.sats || 0));
    const usedCount = (r.gps.avg_snr_count && r.gps.avg_snr_count > 0) ? r.gps.avg_snr_count : null;
    const top16Count = (r.gps.top16_count && r.gps.top16_count > 0) ? r.gps.top16_count : Math.min(16, sats.length);

    const badgeEl = document.getElementById('modal-snr-badge');
    const sumEl = document.getElementById('modal-snr-summary');
    const avgLabelEl = document.getElementById('modal-snr-avg-label');
    const avgEl = document.getElementById('modal-snr-avg');
    const top16LabelEl = document.getElementById('modal-snr-top16-label');
    const top16El = document.getElementById('modal-snr-top16');
    const cntEl = document.getElementById('modal-snr-count');
    const cntUnitEl = document.getElementById('modal-snr-count-unit');
    const topEl = document.getElementById('modal-snr-top');
    const constsEl = document.getElementById('modal-snr-consts');
    const barsEl = document.getElementById('modal-snr-bars');

    if (avg !== null && !isNaN(avg)) {
      if (avgLabelEl) {
        if (r.gps.avg_snr_mode === 'used' && usedCount !== null) {
          avgLabelEl.innerText = `Used Avg (GGA ${usedCount})`;
        } else if (r.gps.avg_snr_mode === 'top' && usedCount !== null) {
          avgLabelEl.innerText = `Usable Avg (Top ${usedCount})`;
        } else {
          avgLabelEl.innerText = 'Used Avg (GGA)';
        }
      }
      if (avgEl) avgEl.innerText = avg.toFixed(1);

      const hasTop16 = top16 !== null && !isNaN(top16);
      if (top16LabelEl) top16LabelEl.innerText = `Top ${top16Count || 16} Avg`;
      if (top16El) {
        top16El.innerText = hasTop16 ? top16.toFixed(1) : '--';
        if (top16El.parentElement) {
          top16El.parentElement.style.color = !hasTop16 ? '#94a3b8' : (top16 >= 38 ? '#34d399' : (top16 >= 28 ? '#fbbf24' : '#f87171'));
        }
      }

      if (cntEl) {
        if (r.gps.avg_snr_mode === 'used' && usedCount !== null) {
          cntEl.innerText = `${usedCount} / ${totalVisible}`;
          if (cntUnitEl) cntUnitEl.innerText = 'used';
        } else {
          cntEl.innerText = totalVisible;
          if (cntUnitEl) cntUnitEl.innerText = 'sats';
        }
      }

      if (sumEl) {
        const top16Part = hasTop16 ? ` • Top ${top16Count || 16}: ${top16.toFixed(1)} dB-Hz` : '';
        if (r.gps.avg_snr_mode === 'used' && usedCount !== null) {
          sumEl.innerText = `${usedCount}/${totalVisible} Used • Avg ${avg.toFixed(1)}${top16Part}`;
        } else {
          sumEl.innerText = `${totalVisible} Sats • Avg ${avg.toFixed(1)}${top16Part}`;
        }
      }

      if (badgeEl) {
        if (avg >= 38) {
          badgeEl.innerText = `🟢 EXCELLENT (${avg.toFixed(1)} dB-Hz)`;
          badgeEl.style.background = 'rgba(16, 185, 129, 0.15)';
          badgeEl.style.color = '#34d399';
          badgeEl.style.borderColor = 'rgba(16, 185, 129, 0.3)';
        } else if (avg >= 28) {
          badgeEl.innerText = `🟡 GOOD (${avg.toFixed(1)} dB-Hz)`;
          badgeEl.style.background = 'rgba(245, 158, 11, 0.15)';
          badgeEl.style.color = '#fbbf24';
          badgeEl.style.borderColor = 'rgba(245, 158, 11, 0.3)';
        } else {
          badgeEl.innerText = `🔴 LOW SIGNAL (${avg.toFixed(1)} dB-Hz)`;
          badgeEl.style.background = 'rgba(239, 68, 68, 0.15)';
          badgeEl.style.color = '#f87171';
          badgeEl.style.borderColor = 'rgba(239, 68, 68, 0.3)';
        }
      }
    } else {
      if (avgLabelEl) avgLabelEl.innerText = 'Used Avg (GGA)';
      if (avgEl) avgEl.innerText = '--';
      if (top16LabelEl) top16LabelEl.innerText = 'Top 16 Avg';
      if (top16El) {
        top16El.innerText = '--';
        if (top16El.parentElement) top16El.parentElement.style.color = '#94a3b8';
      }
      if (cntEl) cntEl.innerText = r.gps.sats || 0;
      if (cntUnitEl) cntUnitEl.innerText = 'sats';
      if (sumEl) sumEl.innerText = `${r.gps.sats || 0} Sats • Waiting C/N0...`;
    }

    if (sats.length > 0) {
      const topSat = sats[0];
      if (topEl) topEl.innerText = `${topSat.id} (${topSat.snr} dB)`;
      const constCounts = {};
      sats.forEach(s => {
        constCounts[s.constellation] = (constCounts[s.constellation] || 0) + 1;
      });
      if (constsEl) {
        constsEl.innerText = Object.keys(constCounts).map(k => `${k}:${constCounts[k]}`).join(' ');
      }
      if (barsEl && snrPanelOpen) {
        const constColors = {
          GPS: '#38bdf8',
          Galileo: '#34d399',
          BeiDou: '#c084fc',
          GLONASS: '#fb7185',
          QZSS: '#fbbf24'
        };
        barsEl.innerHTML = sats.map(sat => {
          const snr = sat.snr || 0;
          const heightPct = Math.min(100, Math.max(8, (snr / 50.0) * 100));
          let barGrad = 'linear-gradient(to top, #059669, #10b981)';
          let valColor = '#34d399';
          if (snr < 28) {
            barGrad = 'linear-gradient(to top, #dc2626, #ef4444)';
            valColor = '#ef4444';
          } else if (snr < 38) {
            barGrad = 'linear-gradient(to top, #d97706, #f59e0b)';
            valColor = '#fbbf24';
          }
          const tagColor = constColors[sat.constellation] || '#94a3b8';
          const elevStr = (sat.elev !== null && sat.elev >= 0) ? `${sat.elev}°` : '--';
          const title = `${sat.constellation} ${sat.id} | SNR: ${snr} dB-Hz | Elev: ${elevStr}`;
          return `
            <div class="fleet-snr-col" title="${title}">
              <div class="fleet-snr-val" style="color: ${valColor};">${Math.round(snr)}</div>
              <div class="fleet-snr-track">
                <div class="fleet-snr-fill" style="height: ${heightPct.toFixed(0)}%; background: ${barGrad};"></div>
              </div>
              <div class="fleet-snr-tag" style="color: ${tagColor}; border-color: ${tagColor}40;">${sat.id}</div>
            </div>
          `;
        }).join('');
      }
    } else {
      if (topEl) topEl.innerText = '--';
      if (constsEl) constsEl.innerText = 'GPS/GAL/BDS/GLO';
      if (barsEl && snrPanelOpen) {
        barsEl.innerHTML = `<div style="width: 100%; text-align: center; padding: 18px; color: #94a3b8; font-size: 0.78rem;">Waiting for per-satellite GSV C/N0 stream from Rover...</div>`;
      }
    }
  }

  // Render Wi-Fi Signal Strength & Network Link Flap (always visible under GNSS SNR)
  updateWifiModalSection(r);

  document.getElementById('modal-motion-state').innerText = r.motion.state;
  document.getElementById('modal-speed').innerText = (r.lidar.odom_vx !== null && !isNaN(r.lidar.odom_vx)) ? `${r.lidar.odom_vx.toFixed(2)} m/s` : '--';
  document.getElementById('modal-dist-progress').innerText = r.motion.step_progress;
  document.getElementById('modal-turn-progress').innerText = '--';
}

function updateWifiModalSection(r) {
  const w = r.wifi || {};
  const badgeEl = document.getElementById('modal-wifi-badge');
  const sumEl = document.getElementById('modal-wifi-summary');

  const hasRssi = (w.rssi_dbm !== null && w.rssi_dbm !== undefined && !isNaN(w.rssi_dbm));
  const rssi = hasRssi ? Number(w.rssi_dbm) : null;
  const iface = w.iface || null;
  const qual = (w.quality_pct !== null && w.quality_pct !== undefined && !isNaN(w.quality_pct)) ? Number(w.quality_pct) : null;
  const pingMs = (w.ping_ms !== null && w.ping_ms !== undefined && !isNaN(w.ping_ms)) ? Number(w.ping_ms) : null;

  // Compact Flap Header (always updated so signal is readable even when collapsed)
  if (badgeEl) {
    if (!r.online) {
      badgeEl.innerText = 'OFFLINE';
      badgeEl.style.background = 'rgba(239, 68, 68, 0.15)';
      badgeEl.style.color = '#f87171';
      badgeEl.style.borderColor = 'rgba(239, 68, 68, 0.3)';
    } else if (hasRssi) {
      const adapterTag = iface ? `${iface}` : 'Wi-Fi';
      if (rssi >= -65) {
        badgeEl.innerText = `🟢 STRONG (${adapterTag})`;
        badgeEl.style.background = 'rgba(16, 185, 129, 0.15)';
        badgeEl.style.color = '#34d399';
        badgeEl.style.borderColor = 'rgba(16, 185, 129, 0.3)';
      } else if (rssi >= -75) {
        badgeEl.innerText = `🟡 FAIR (${adapterTag})`;
        badgeEl.style.background = 'rgba(245, 158, 11, 0.15)';
        badgeEl.style.color = '#fbbf24';
        badgeEl.style.borderColor = 'rgba(245, 158, 11, 0.3)';
      } else {
        badgeEl.innerText = `🔴 WEAK (${adapterTag})`;
        badgeEl.style.background = 'rgba(239, 68, 68, 0.15)';
        badgeEl.style.color = '#f87171';
        badgeEl.style.borderColor = 'rgba(239, 68, 68, 0.3)';
      }
    } else if (iface) {
      badgeEl.innerText = `🌐 ${iface}`;
      badgeEl.style.background = 'rgba(56, 189, 248, 0.15)';
      badgeEl.style.color = '#38bdf8';
      badgeEl.style.borderColor = 'rgba(56, 189, 248, 0.3)';
    } else {
      badgeEl.innerText = 'WAITING...';
      badgeEl.style.background = 'rgba(148, 163, 184, 0.15)';
      badgeEl.style.color = '#94a3b8';
      badgeEl.style.borderColor = 'rgba(148, 163, 184, 0.3)';
    }
  }

  if (sumEl) {
    const rssiPart = hasRssi ? `${Math.round(rssi)} dBm` : (iface ? iface : '-- dBm');
    const qualPart = qual !== null ? ` (${qual}%)` : '';
    const pingPart = pingMs !== null ? ` • ${pingMs} ms` : '';
    sumEl.innerText = `${rssiPart}${qualPart}${pingPart}`;
    sumEl.style.color = !hasRssi ? '#94a3b8' : (rssi >= -65 ? '#34d399' : (rssi >= -75 ? '#fbbf24' : '#f87171'));
  }

  // Skip detailed DOM updates when flap is collapsed
  if (!wifiPanelOpen) return;

  const ifaceEl = document.getElementById('modal-wifi-iface');
  const adapterTypeEl = document.getElementById('modal-wifi-adapter-type');
  const rssiEl = document.getElementById('modal-wifi-rssi');
  const rssiAvgEl = document.getElementById('modal-wifi-rssi-avg');
  const qualEl = document.getElementById('modal-wifi-quality');
  const qualBarEl = document.getElementById('modal-wifi-qual-bar');
  const rttEl = document.getElementById('modal-wifi-rtt');
  const rttAvgEl = document.getElementById('modal-wifi-rtt-avg');
  const jitterEl = document.getElementById('modal-wifi-jitter');
  const lossEl = document.getElementById('modal-wifi-loss');
  const retriesEl = document.getElementById('modal-wifi-retries');
  const dropsEl = document.getElementById('modal-wifi-drops');
  const barsEl = document.getElementById('modal-wifi-bars');
  const pinBtn = document.getElementById('btn-wifi-pin-a');
  const abBanner = document.getElementById('modal-wifi-ab-compare');

  if (ifaceEl) {
    ifaceEl.innerText = iface || '--';
    ifaceEl.style.color = w.is_usb ? '#38bdf8' : (iface ? '#c084fc' : '#94a3b8');
  }
  if (adapterTypeEl) {
    if (iface) {
      const busLabel = w.is_usb === true ? 'USB Adapter' : (w.is_usb === false ? 'Onboard Pi 5' : 'Network');
      const drvLabel = w.driver ? ` (${w.driver})` : '';
      adapterTypeEl.innerText = `${busLabel}${drvLabel}`;
    } else {
      adapterTypeEl.innerText = 'Waiting /robot/system_info...';
    }
  }

  // Rolling recent RSSI avg (for current interface)
  const hist = Array.isArray(w.rssi_history) ? w.rssi_history : [];
  const sameIfaceHist = iface ? hist.filter(h => h.iface === iface) : hist;
  const recentSlice = sameIfaceHist.slice(-10);
  const avgRssi = recentSlice.length > 0
    ? Number((recentSlice.reduce((acc, s) => acc + s.rssi, 0) / recentSlice.length).toFixed(1))
    : null;

  if (rssiEl) {
    rssiEl.innerText = hasRssi ? rssi.toFixed(0) : '--';
    if (rssiEl.parentElement) {
      rssiEl.parentElement.style.color = !hasRssi ? '#fff' : (rssi >= -65 ? '#34d399' : (rssi >= -75 ? '#fbbf24' : '#f87171'));
    }
  }
  if (rssiAvgEl) {
    rssiAvgEl.innerText = avgRssi !== null ? `Avg (${recentSlice.length}): ${avgRssi} dBm` : 'Avg: -- dBm';
  }

  if (qualEl) {
    qualEl.innerText = qual !== null ? String(qual) : '--';
    if (qualEl.parentElement) {
      qualEl.parentElement.style.color = qual === null ? '#94a3b8' : (qual >= 65 ? '#34d399' : (qual >= 40 ? '#fbbf24' : '#f87171'));
    }
  }
  if (qualBarEl) {
    const pct = qual !== null ? Math.max(0, Math.min(100, qual)) : 0;
    qualBarEl.style.width = `${pct}%`;
    qualBarEl.style.background = pct >= 65 ? '#10b981' : (pct >= 40 ? '#f59e0b' : '#ef4444');
  }

  if (rttEl) {
    rttEl.innerText = pingMs !== null ? pingMs.toFixed(1) : '--';
    if (rttEl.parentElement) {
      rttEl.parentElement.style.color = pingMs === null ? '#fff' : (pingMs <= 20 ? '#34d399' : (pingMs <= 50 ? '#fbbf24' : '#f87171'));
    }
  }
  if (rttAvgEl) {
    rttAvgEl.innerText = (w.ping_avg_ms !== null && w.ping_avg_ms !== undefined)
      ? `Avg: ${w.ping_avg_ms.toFixed(1)} ms (${(w.ping_samples || []).length}x)`
      : 'Avg: -- ms';
  }

  if (jitterEl) {
    const jit = w.ping_jitter_ms;
    jitterEl.innerText = (jit !== null && jit !== undefined) ? `±${jit.toFixed(1)} ms` : '-- ms';
    jitterEl.style.color = (jit === null || jit === undefined) ? '#94a3b8' : (jit <= 5 ? '#34d399' : (jit <= 15 ? '#fbbf24' : '#f87171'));
  }
  if (lossEl) {
    const loss = w.ping_loss_pct;
    lossEl.innerText = (loss !== null && loss !== undefined) ? `Packet loss: ${loss.toFixed(1)}%` : 'Packet loss: --%';
    lossEl.style.color = (loss !== null && loss > 0) ? '#f87171' : '#94a3b8';
  }

  if (retriesEl) {
    retriesEl.innerText = (w.retries !== null && w.retries !== undefined) ? `${w.retries} ret` : '--';
  }
  if (dropsEl) {
    const rxD = (w.rx_drop !== null && w.rx_drop !== undefined) ? w.rx_drop : '--';
    const txD = (w.tx_drop !== null && w.tx_drop !== undefined) ? w.tx_drop : '--';
    dropsEl.innerText = `RX/TX drop: ${rxD} / ${txD}`;
  }

  // Render Rolling RSSI History Bar Chart (only when data actually changes)
  if (barsEl) {
    const renderKey = `${r.id}:${hist.length}:${hist.length > 0 ? hist[hist.length - 1].ts : 0}`;
    if (renderKey !== lastWifiBarsRenderKey) {
      lastWifiBarsRenderKey = renderKey;
      if (hist.length > 0) {
        barsEl.innerHTML = hist.map(sample => {
          const val = sample.rssi;
          const heightPct = Math.min(100, Math.max(8, ((val + 95.0) / 60.0) * 100.0));
          let barGrad = 'linear-gradient(to top, #059669, #10b981)';
          let valColor = '#34d399';
          if (val < -75) {
            barGrad = 'linear-gradient(to top, #dc2626, #ef4444)';
            valColor = '#f87171';
          } else if (val < -65) {
            barGrad = 'linear-gradient(to top, #d97706, #f59e0b)';
            valColor = '#fbbf24';
          }
          const tag = sample.is_usb ? 'U' : 'O';
          const tagColor = sample.is_usb ? '#38bdf8' : '#c084fc';
          const tStr = new Date(sample.ts).toLocaleTimeString();
          const title = `${sample.iface} (${sample.is_usb ? 'USB' : 'Onboard'}) | ${val} dBm | ${tStr}`;
          return `
            <div class="fleet-snr-col" style="min-width: 22px;" title="${title}">
              <div class="fleet-snr-val" style="color: ${valColor}; font-size: 9px;">${Math.round(val)}</div>
              <div class="fleet-snr-track" style="height: 76px; width: 13px;">
                <div class="fleet-snr-fill" style="height: ${heightPct.toFixed(0)}%; background: ${barGrad};"></div>
              </div>
              <div class="fleet-snr-tag" style="color: ${tagColor}; border-color: ${tagColor}40;">${tag}</div>
            </div>
          `;
        }).join('');
        barsEl.scrollLeft = barsEl.scrollWidth;
      } else {
        barsEl.innerHTML = `<div style="width: 100%; text-align: center; padding: 18px; color: #94a3b8; font-size: 0.78rem;">Waiting for Wi-Fi RSSI samples from /robot/system_info (2s interval)...</div>`;
      }
    }
  }

  // A/B Extender Comparison Banner
  const baseA = wifiAbBaselines[r.id];
  if (pinBtn) {
    if (baseA) {
      pinBtn.className = 'wifi-action-btn pinned';
      pinBtn.innerText = `📌 Baseline A: ${baseA.iface}`;
    } else {
      pinBtn.className = 'wifi-action-btn';
      pinBtn.innerText = '📌 Pin Baseline (A)';
    }
  }
  if (abBanner) {
    if (baseA) {
      abBanner.style.display = 'block';
      const curRssi = avgRssi !== null ? avgRssi : rssi;
      const curPing = w.ping_avg_ms !== null ? w.ping_avg_ms : pingMs;
      const curIface = iface || 'current';

      let rssiDeltaHtml = 'RSSI: --';
      if (baseA.avg_rssi !== null && curRssi !== null) {
        const dDb = Number((curRssi - baseA.avg_rssi).toFixed(1));
        const sign = dDb > 0 ? '+' : '';
        const col = dDb >= 2 ? '#34d399' : (dDb <= -2 ? '#f87171' : '#fbbf24');
        const verdict = dDb >= 2 ? 'stronger' : (dDb <= -2 ? 'weaker' : 'similar');
        rssiDeltaHtml = `RSSI: <strong style="color:${col};">${sign}${dDb} dB</strong> (${verdict})`;
      }

      let pingDeltaHtml = 'Ping: --';
      if (baseA.avg_ping !== null && curPing !== null) {
        const dMs = Number((curPing - baseA.avg_ping).toFixed(1));
        const sign = dMs > 0 ? '+' : '';
        const col = dMs <= -1 ? '#34d399' : (dMs >= 3 ? '#f87171' : '#fbbf24');
        pingDeltaHtml = `Ping: <strong style="color:${col};">${sign}${dMs} ms</strong>`;
      }

      const baseRssiStr = baseA.avg_rssi !== null ? `${baseA.avg_rssi} dBm` : '-- dBm';
      const basePingStr = baseA.avg_ping !== null ? `${baseA.avg_ping} ms` : '-- ms';
      abBanner.innerHTML =
        `<div><strong>A (${baseA.iface}):</strong> ${baseRssiStr} • ${basePingStr} (${baseA.loss_pct}% loss)</div>` +
        `<div><strong>Now (${curIface}) vs A:</strong> ${rssiDeltaHtml} &nbsp;|&nbsp; ${pingDeltaHtml}</div>`;
    } else {
      abBanner.style.display = 'none';
    }
  }
}

function initEventListeners() {
  document.getElementById('btn-close-modal').onclick = () => {
    document.getElementById('robot-detail-modal').classList.add('hidden');
    stopWifiPingLoop();
    selectedRobotId = null;
  };

  const snrHeader = document.getElementById('modal-snr-header');
  if (snrHeader) {
    snrHeader.onclick = () => toggleSnrPanel();
  }

  const wifiHeader = document.getElementById('modal-wifi-header');
  if (wifiHeader) {
    wifiHeader.onclick = () => toggleWifiPanel();
  }

  const btnWifiBurst = document.getElementById('btn-wifi-burst');
  if (btnWifiBurst) {
    btnWifiBurst.onclick = (e) => {
      e.stopPropagation();
      runWifiBurstTest();
    };
  }

  const btnWifiPinA = document.getElementById('btn-wifi-pin-a');
  if (btnWifiPinA) {
    btnWifiPinA.onclick = (e) => {
      e.stopPropagation();
      pinWifiBaseline();
    };
  }

  const btnWifiReset = document.getElementById('btn-wifi-reset');
  if (btnWifiReset) {
    btnWifiReset.onclick = (e) => {
      e.stopPropagation();
      resetWifiHistory();
    };
  }

  const btnToggleRadar = document.getElementById('btn-toggle-lidar-radar');
  if (btnToggleRadar) {
    btnToggleRadar.onclick = () => toggleLidarRadarPanel();
  }

  document.querySelectorAll('.lidar-range-btn').forEach(btn => {
    btn.onclick = () => {
      const rVal = Number(btn.getAttribute('data-range')) || 4.0;
      setLidarRadarRange(rVal);
    };
  });

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
    } catch (e) {}
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
    } catch (e) {
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
