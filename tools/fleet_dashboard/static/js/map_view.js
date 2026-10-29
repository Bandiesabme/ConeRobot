/* ==========================================================================
   ConeRobot Fleet Dashboard - Module 4: Leaflet Map & Robot Pin Markers
   ========================================================================== */

function initMap() {
  map = L.map('fleet-map', {
    center: [47.4979, 19.0402],
    zoom: 16,
    zoomControl: true
  });

  // 1. Standard OpenStreetMap (100% Free, No API Key needed)
  const osm = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
  });

  // 2. High-Resolution Satellite Imagery (Esri World Imagery, 100% Free)
  const satellite = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    maxZoom: 19,
    attribution: 'Tiles &copy; Esri'
  });

  osm.addTo(map);

  const baseMaps = {
    "Utcatérkép (OSM)": osm,
    "Műholdkép (Satellite)": satellite
  };
  L.control.layers(baseMaps, null, { position: 'topright' }).addTo(map);

  const btnFit = document.getElementById('btn-fit-map');
  if (btnFit) {
    btnFit.addEventListener('click', fitAllRobotsInMap);
  }
  setTimeout(() => { if (map) map.invalidateSize(); }, 300);
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
  let primaryCoordText = null;

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
      const rNum = String(id).padStart(2, '0');

      if (!primaryCoordText) {
        primaryCoordText = `R${rNum}: ${lat.toFixed(7)}° N, ${lon.toFixed(7)}° E (${gpsMode})`;
      }

      const tooltipContent = `<b>ConeRobot ${rNum}</b> [${gpsMode}]<br>📍 ${lat.toFixed(7)}, ${lon.toFixed(7)}<br>Alt: ${r.gps?.alt ? r.gps.alt + ' m' : '--'} | Sats: ${r.gps?.sats || 0}`;

      if (robotMarkers[id]) {
        const prev = robotMarkers[id]._prevData;
        const posChanged = !prev || Math.abs(prev.lat - lat) > 1e-7 || Math.abs(prev.lon - lon) > 1e-7;
        const rotChanged = !prev || Math.abs((prev.heading || 0) - (heading || 0)) > 0.4 || prev.gpsMode !== gpsMode;

        if (posChanged) {
          robotMarkers[id].setLatLng([lat, lon]);
        }
        if (rotChanged) {
          robotMarkers[id].setIcon(icon);
          if (robotMarkers[id].getTooltip()) {
            robotMarkers[id].setTooltipContent(tooltipContent);
          }
        }
        robotMarkers[id]._prevData = { lat, lon, heading, gpsMode };
      } else {
        const marker = L.marker([lat, lon], { icon: icon }).addTo(map);
        marker.bindTooltip(tooltipContent, { direction: 'top', offset: [0, -18], className: 'robot-map-tooltip' });
        marker.on('click', () => openRobotDetailModal(id));
        marker._prevData = { lat, lon, heading, gpsMode };
        robotMarkers[id] = marker;
      }

      if (!hasAutoCentered) {
        map.setView([lat, lon], 19);
        hasAutoCentered = true;
      }
    }
  });

  const coordsTextEl = document.getElementById('map-coords-text');
  if (coordsTextEl) {
    coordsTextEl.innerText = primaryCoordText ? primaryCoordText : 'Searching GPS...';
  }

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
  if (coords.length === 1) {
    map.setView(coords[0], 19);
  } else if (coords.length > 1) {
    const bounds = L.latLngBounds(coords);
    map.fitBounds(bounds, { padding: [40, 40], maxZoom: 19 });
  }
}
