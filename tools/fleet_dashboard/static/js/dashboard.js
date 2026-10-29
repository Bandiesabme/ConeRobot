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
          <span class="metric-key">GNSS Signal (C/N0)</span>
          <span class="metric-val" id="r${i}-snr">--</span>
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
            const satCnt = (r.gps.snr_sats && r.gps.snr_sats.length > 0) ? r.gps.snr_sats.length : (r.gps.total_signals || r.gps.sats || 0);
            const icon = avg >= 38 ? '🟢' : (avg >= 28 ? '🟡' : '🔴');
            snrEl.innerText = `${icon} ${avg.toFixed(1)} dB-Hz (${satCnt} Sats)`;
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

      headEl.innerText = (r.imu.heading !== null) ? `${Number(r.imu.heading).toFixed(1)}°` : 'Waiting for IMU...';
      stepEl.innerText = r.motion.step_progress;
      pingEl.innerText = '🟢 Connected';

      const battEl = document.getElementById(`r${i}-battery`);
      if (battEl) {
        if (r.health.battery_v && r.health.battery_v !== '--') {
          battEl.innerText = r.health.battery_v;
          battEl.style.color = '#10b981';
        } else if (r.health.cpu_temp !== '--' && r.health.cpu_load) {
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

// --- 3. Detail Modal ---
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
    const sats = r.gps.snr_sats || [];
    const satCount = sats.length > 0 ? sats.length : (r.gps.total_signals || r.gps.sats || 0);
    const badgeEl = document.getElementById('modal-snr-badge');
    const sumEl = document.getElementById('modal-snr-summary');
    const avgEl = document.getElementById('modal-snr-avg');
    const cntEl = document.getElementById('modal-snr-count');
    const topEl = document.getElementById('modal-snr-top');
    const constsEl = document.getElementById('modal-snr-consts');
    const barsEl = document.getElementById('modal-snr-bars');

    if (avg !== null && !isNaN(avg)) {
      if (avgEl) avgEl.innerText = avg.toFixed(1);
      if (cntEl) cntEl.innerText = satCount;
      if (sumEl) sumEl.innerText = `${satCount} Sats • Avg ${avg.toFixed(1)} dB-Hz`;
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
      if (avgEl) avgEl.innerText = '--';
      if (cntEl) cntEl.innerText = r.gps.sats || 0;
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
      if (barsEl) {
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
      if (barsEl) {
        barsEl.innerHTML = `<div style="width: 100%; text-align: center; padding: 18px; color: #94a3b8; font-size: 0.78rem;">Waiting for per-satellite GSV C/N0 stream from Rover...</div>`;
      }
    }
  }

  document.getElementById('modal-motion-state').innerText = r.motion.state;
  document.getElementById('modal-speed').innerText = (r.lidar.odom_vx !== null && !isNaN(r.lidar.odom_vx)) ? `${r.lidar.odom_vx.toFixed(2)} m/s` : '--';
  document.getElementById('modal-dist-progress').innerText = r.motion.step_progress;
  document.getElementById('modal-turn-progress').innerText = '--';
}

function initEventListeners() {
  document.getElementById('btn-close-modal').onclick = () => {
    document.getElementById('robot-detail-modal').classList.add('hidden');
    selectedRobotId = null;
  };

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
