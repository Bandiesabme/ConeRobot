/* ==========================================================================
   ConeRobot Fleet Dashboard - Module 3: 2D LiDAR Point Cloud Radar Canvas
   Hidden by default; opened on demand via toggle button inside the Robot Modal.
   Zero Pi 5 CPU/RAM overhead (renders existing /scan CDR stream in browser).
   ========================================================================== */

function toggleLidarRadarPanel() {
  lidarRadarOpen = !lidarRadarOpen;
  const wrap = document.getElementById('modal-lidar-radar-wrap');
  const btn = document.getElementById('btn-toggle-lidar-radar');

  if (wrap) {
    wrap.style.display = lidarRadarOpen ? 'block' : 'none';
  }
  if (btn) {
    if (lidarRadarOpen) {
      btn.innerHTML = '🔼 Hide 2D LiDAR Map';
      btn.style.background = 'rgba(56, 189, 248, 0.22)';
      btn.style.borderColor = '#38bdf8';
      btn.style.color = '#ffffff';
    } else {
      btn.innerHTML = '📡 Show 2D LiDAR Map';
      btn.style.background = 'rgba(56, 189, 248, 0.12)';
      btn.style.borderColor = 'rgba(56, 189, 248, 0.35)';
      btn.style.color = '#38bdf8';
    }
  }

  if (lidarRadarOpen && selectedRobotId && realTelemetry[selectedRobotId]) {
    renderLidarRadarCanvas(realTelemetry[selectedRobotId]);
  }
}

function setLidarRadarRange(rangeM) {
  lidarRadarMaxRangeM = Number(rangeM) || 4.0;
  const btns = document.querySelectorAll('.lidar-range-btn');
  btns.forEach(b => {
    const val = Number(b.getAttribute('data-range'));
    if (Math.abs(val - lidarRadarMaxRangeM) < 0.01) {
      b.style.background = '#38bdf8';
      b.style.color = '#0b0f19';
      b.style.fontWeight = '700';
    } else {
      b.style.background = 'rgba(255,255,255,0.06)';
      b.style.color = '#94a3b8';
      b.style.fontWeight = '500';
    }
  });

  const scaleEl = document.getElementById('lidar-radar-scale');
  if (scaleEl) {
    scaleEl.innerText = `±${lidarRadarMaxRangeM} m`;
  }

  if (lidarRadarOpen && selectedRobotId && realTelemetry[selectedRobotId]) {
    renderLidarRadarCanvas(realTelemetry[selectedRobotId]);
  }
}

function renderLidarRadarCanvas(r) {
  if (!lidarRadarOpen || !r) return;

  const canvas = document.getElementById('modal-lidar-canvas');
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const cx = w / 2;
  const cy = h / 2;
  const maxRadiusPx = Math.min(cx, cy) - 22;
  const maxRangeM = lidarRadarMaxRangeM || 4.0;
  const pxPerMeter = maxRadiusPx / maxRangeM;

  // 1. Clear & Dark Radar Background
  ctx.fillStyle = '#070b14';
  ctx.fillRect(0, 0, w, h);

  // 2. Concentric Range Rings (25%, 50%, 75%, 100% of maxRangeM)
  ctx.lineWidth = 1;
  ctx.font = '10px monospace';
  for (let step = 1; step <= 4; step++) {
    const frac = step / 4.0;
    const radPx = maxRadiusPx * frac;
    const ringMeters = (maxRangeM * frac);

    ctx.strokeStyle = step === 4 ? 'rgba(56, 189, 248, 0.28)' : 'rgba(148, 163, 184, 0.14)';
    ctx.beginPath();
    ctx.arc(cx, cy, radPx, 0, Math.PI * 2);
    ctx.stroke();

    // Label each ring along top-right diagonal
    const labelStr = ringMeters >= 1.0 ? `${ringMeters.toFixed(ringMeters % 1 === 0 ? 0 : 1)}m` : `${(ringMeters * 100).toFixed(0)}cm`;
    ctx.fillStyle = 'rgba(148, 163, 184, 0.55)';
    ctx.fillText(labelStr, cx + 5, cy - radPx + 11);
  }

  // 3. Crosshair Axes (+X Forward UP, +Y Left LEFT)
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.15)';
  ctx.beginPath();
  ctx.moveTo(cx, cy - maxRadiusPx);
  ctx.lineTo(cx, cy + maxRadiusPx);
  ctx.moveTo(cx - maxRadiusPx, cy);
  ctx.lineTo(cx + maxRadiusPx, cy);
  ctx.stroke();

  // Forward (+X) Axis Highlight Line
  ctx.strokeStyle = 'rgba(56, 189, 248, 0.40)';
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx, cy - maxRadiusPx);
  ctx.stroke();

  // Direction Labels
  ctx.fillStyle = '#38bdf8';
  ctx.font = 'bold 11px sans-serif';
  ctx.textAlign = 'center';
  ctx.fillText('▲ FRONT (+X)', cx, 14);
  ctx.fillStyle = '#64748b';
  ctx.font = '10px sans-serif';
  ctx.fillText('REAR (-X)', cx, h - 6);
  ctx.textAlign = 'left';
  ctx.fillText('LEFT (+Y)', 8, cy - 5);
  ctx.textAlign = 'right';
  ctx.fillText('RIGHT (-Y)', w - 8, cy - 5);
  ctx.textAlign = 'left';

  // 4. Draw Point Cloud
  const pts = (r.lidar && Array.isArray(r.lidar.points)) ? r.lidar.points : [];
  let minPt = null;

  if (pts.length > 0) {
    for (let i = 0; i < pts.length; i++) {
      const x = pts[i][0]; // Forward (+X) in meters
      const y = pts[i][1]; // Left (+Y) in meters
      const d = pts[i][2]; // Distance in meters

      if (d > maxRangeM * 1.15) continue;

      // Screen mapping:
      // +X (Forward) -> -Y screen (Up)
      // +Y (Left)    -> -X screen (Left)
      const sx = cx - y * pxPerMeter;
      const sy = cy - x * pxPerMeter;

      if (sx < 2 || sx > w - 2 || sy < 2 || sy > h - 2) continue;

      if (!minPt || d < minPt[2]) {
        minPt = [sx, sy, d];
      }

      // Color by obstacle distance
      if (d < 0.50) {
        ctx.fillStyle = '#ef4444'; // Red (< 50 cm)
      } else if (d < 1.20) {
        ctx.fillStyle = '#f59e0b'; // Amber (0.5 m - 1.2 m)
      } else {
        ctx.fillStyle = '#38bdf8'; // Cyan (> 1.2 m)
      }

      ctx.fillRect(sx - 1.5, sy - 1.5, 3, 3);
    }

    // Highlight closest obstacle point with a dashed line & target circle
    if (minPt) {
      ctx.save();
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.55)';
      ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(minPt[0], minPt[1]);
      ctx.stroke();

      ctx.setLineDash([]);
      ctx.strokeStyle = '#ef4444';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(minPt[0], minPt[1], 5, 0, Math.PI * 2);
      ctx.stroke();
      ctx.restore();
    }
  } else {
    // Waiting or no LiDAR message overlay
    ctx.fillStyle = '#94a3b8';
    ctx.font = '12px sans-serif';
    ctx.textAlign = 'center';
    const msg = r.lidar?.present
      ? 'Waiting for /scan point cloud packets...'
      : 'No LiDAR (/scan) detected on this robot';
    ctx.fillText(msg, cx, cy + 42);
    ctx.textAlign = 'left';
  }

  // 5. Draw Robot Chassis Icon at Center (pointing UP along +X)
  ctx.save();
  ctx.translate(cx, cy);
  ctx.fillStyle = '#10b981';
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(0, -10);
  ctx.lineTo(8, 8);
  ctx.lineTo(0, 4);
  ctx.lineTo(-8, 8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();

  // 6. Update Radar Stats Bar
  const ptsEl = document.getElementById('lidar-radar-pts');
  if (ptsEl) {
    ptsEl.innerText = pts.length > 0 ? `${pts.length} pts (${r.lidar.sample_count || pts.length} raw)` : (r.lidar?.sample_count ? `${r.lidar.sample_count} rays` : '0 pts');
  }

  const minEl = document.getElementById('lidar-radar-min');
  if (minEl) {
    if (r.lidar?.min_dist !== null && r.lidar?.min_dist !== undefined) {
      const angTxt = (r.lidar.min_angle_deg !== null && r.lidar.min_angle_deg !== undefined) ? ` @ ${r.lidar.min_angle_deg}°` : '';
      minEl.innerText = `${r.lidar.min_dist.toFixed(2)} m${angTxt}`;
      minEl.style.color = r.lidar.min_dist < 0.50 ? '#ef4444' : (r.lidar.min_dist < 1.20 ? '#f59e0b' : '#10b981');
    } else {
      minEl.innerText = '--';
      minEl.style.color = '#94a3b8';
    }
  }
}
