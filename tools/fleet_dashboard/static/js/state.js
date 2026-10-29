/* ==========================================================================
   ConeRobot Fleet Dashboard - Module 1: Shared State & Debug Logger
   ========================================================================== */

let map = null;
let robotMarkers = {};
let selectedRobotId = null;
let activeWebSockets = {}; // { 1: WebSocket }
let reconnectTimers = {};  // { 1: timerId }
let realTelemetry = {};    // { 1: { ... } }
let hasAutoCentered = false;

// 2D LiDAR Point Cloud Radar View State (Hidden by default to save CPU)
let lidarRadarOpen = false;
let lidarRadarMaxRangeM = 4.0; // Default zoom radius in meters (1m, 2m, 4m, 8m, 12m)

let configuredTopics = {
  scan: '/scan',
  odom: '/odom',
  gps_fix: '/fix',
  gps_status: '/gps/status',
  heading: '/imu/heading',
  imu_data: '/imu/data',
  step_status: '/step_status',
  battery: '/battery_state'
};

function createEmptyRobotTelemetry(id) {
  return {
    id: id,
    ip: '',
    online: false,
    lidar: {
      present: false,
      freq_hz: 0,
      sample_count: 0,
      min_dist: null,
      min_angle_deg: null,
      last_scan_time: 0,
      points: [], // Array of [x_forward_m, y_left_m, dist_m] populated when radar is open
      odom_present: false,
      odom_x: null,
      odom_y: null,
      odom_vx: null
    },
    gps: {
      present: false,
      mode: 'NO FIX',
      lat: null,
      lon: null,
      alt: null,
      sats: 0,
      hdop: null,
      diff_age: null,
      avg_snr: null,
      strong_signals: 0,
      total_signals: 0,
      snr_sats: []
    },
    imu: { heading: null, pitch: null, roll: null, yaw: null, last_heading_topic_time: 0 },
    motion: { state: 'Offline', step_progress: '--' },
    health: { cpu_temp: '--', cpu_load: '--', battery_v: '--' }
  };
}

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

  while (container.children.length > 60) {
    container.removeChild(container.firstChild);
  }
  container.scrollTop = container.scrollHeight;
}
