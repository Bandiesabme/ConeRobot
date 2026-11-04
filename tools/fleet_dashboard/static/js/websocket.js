/* ==========================================================================
   ConeRobot Fleet Dashboard - Module 5: Fleet Discovery & Foxglove WebSocket
   ========================================================================== */

async function startDiscoveryLoop() {
  logDebug('Discovery engine started. Fetching discovered fleet...', 'info');

  async function pollFleet() {
    try {
      const res = await fetch('/api/fleet');
      if (res.ok) {
        const data = await res.json();
        configuredTopics = data.topics || configuredTopics;
        const robots = data.robots || {};

        if (data.base_station && typeof updateBaseStationLink === 'function') {
          updateBaseStationLink(data.base_station);
        }

        for (let i = 1; i <= 13; i++) {
          const rob = robots[i];
          if (rob && rob.online && rob.ip) {
            realTelemetry[i].ip = rob.ip;

            if (!activeWebSockets[i] && !reconnectTimers[i]) {
              connectToRobotWebSocket(i, rob.ip, rob.port || 8765);
            }
          } else {
            if (reconnectTimers[i]) {
              clearTimeout(reconnectTimers[i]);
              delete reconnectTimers[i];
            }
            if (activeWebSockets[i]) {
              try { activeWebSockets[i].close(); } catch (e) {}
              delete activeWebSockets[i];
            }
            if (realTelemetry[i]) {
              realTelemetry[i].online = false;
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
      return;
    }
  }

  const wsUrl = `ws://${ip}:${port}`;
  logDebug(`[Robot ${robotId}] Connecting to ${wsUrl}...`, 'info');

  let ws;
  try {
    ws = new WebSocket(wsUrl, ["foxglove.sdk.v1", "foxglove.websocket.v1"]);
  } catch (e) {
    try { ws = new WebSocket(wsUrl); } catch (err) {
      logDebug(`[Robot ${robotId}] WebSocket error: ${err}`, 'error');
      scheduleReconnect(robotId, ip, port);
      return;
    }
  }

  activeWebSockets[robotId] = ws;
  ws.binaryType = "arraybuffer";

  const subIdToTopic = {};
  const channelIdToTopic = {};
  let subIdCounter = 1;

  ws.onopen = () => {
    logDebug(`🟢 [Robot ${robotId}] Connected to ${wsUrl}!`, 'success');
    realTelemetry[robotId].online = true;

    // Reset sensor telemetry to clean state upon new connection
    if (typeof resetRobotSensors === 'function') {
      resetRobotSensors(robotId);
    }

    try {
      ws.send(JSON.stringify({
        op: "clientInfo",
        name: "ConeRobotFleetDashboard"
      }));
    } catch (e) {}
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
            '/odom',
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
            const rawTopic = ch.topic || '';
            const topic = rawTopic.startsWith('/') ? rawTopic : '/' + rawTopic;
            const schema = ch.schemaName || '';

            const isScan = topic === configuredTopics.scan || topic === '/scan' || topic.endsWith('/scan');
            const isOdom = topic === configuredTopics.odom || topic === '/odom' || topic.endsWith('/odom') || schema.includes('Odometry');
            const isGpsFix = topic === configuredTopics.gps_fix || topic === '/fix' || topic.endsWith('/fix');
            const isGpsStatus = topic === configuredTopics.gps_status || topic === '/gps/status' || topic.endsWith('/gps/status');
            const isHeading = topic === configuredTopics.heading || topic === '/imu/heading' || topic.endsWith('/heading');
            const isImuData = topic === configuredTopics.imu_data || topic === '/imu/data' || topic.endsWith('/imu/data');
            const isStep = topic === configuredTopics.step_status || topic === '/step_status' || topic.endsWith('/step_status');
            const isDiag = topic === '/robot/diagnostics' || topic.endsWith('/robot/diagnostics');
            const isSysinfo = topic === '/foxglove_bridge/sysinfo' || topic.includes('sysinfo');
            const isBattery = topic === configuredTopics.battery ||
                              topic === '/battery_state' ||
                              topic.includes('battery') ||
                              schema.includes('BatteryState');

            if (isScan || isOdom) hasLidarTopic = true;
            if (isGpsFix) hasGpsTopic = true;

            const isTarget = isScan || isOdom || isGpsFix || isGpsStatus || isHeading || isImuData || isStep || isDiag || isSysinfo || isBattery || targetTopics.includes(topic) || Object.values(configuredTopics).includes(topic);

            if (isTarget && !seen.has(rawTopic)) {
              seen.add(rawTopic);
              const subId = subIdCounter++;
              subIdToTopic[subId] = rawTopic;
              channelIdToTopic[ch.id] = rawTopic;
              subscriptions.push({
                id: subId,
                channelId: ch.id
              });
            }
          });

          if (hasLidarTopic) {
            realTelemetry[robotId].lidar.present = true;
          } else if (typeof resetRobotLidar === 'function') {
            resetRobotLidar(robotId);
          }

          if (hasGpsTopic) {
            realTelemetry[robotId].gps.present = true;
          } else if (typeof resetRobotGps === 'function') {
            resetRobotGps(robotId);
          }

          const topicNames = subscriptions.map(s => subIdToTopic[s.id]).join(', ');
          logDebug(`🛰️ [Robot ${robotId}] Subscribed to ${subscriptions.length} topics: ${topicNames}`, 'topic');

          if (subscriptions.length > 0) {
            ws.send(JSON.stringify({ op: "subscribe", subscriptions: subscriptions }));
          }
        }
        // 2. Channel Unadvertisement
        else if (msg.op === "unadvertise" && Array.isArray(msg.channelIds)) {
          msg.channelIds.forEach(chId => {
            const rawTopic = channelIdToTopic[chId] || '';
            const topic = rawTopic.startsWith('/') ? rawTopic : '/' + rawTopic;
            if (topic === configuredTopics.gps_fix || topic === '/fix' || topic.endsWith('/fix')) {
              if (typeof resetRobotGps === 'function') resetRobotGps(robotId);
            }
            if (topic === configuredTopics.scan || topic === '/scan' || topic.endsWith('/scan')) {
              if (typeof resetRobotLidar === 'function') resetRobotLidar(robotId);
            }
            delete channelIdToTopic[chId];
          });
        }
        // 3. Message Data
        else if (msg.op === "message" && msg.data) {
          const topic = subIdToTopic[msg.subscriptionId] || channelIdToTopic[msg.channelId] || subIdToTopic[msg.id] || channelIdToTopic[msg.id];
          handleParsedRobotMessage(robotId, topic, msg.data);
        }
      } catch (e) {}
    } else if (event.data instanceof ArrayBuffer) {
      handleBinaryFoxgloveMessage(robotId, event.data, subIdToTopic, channelIdToTopic);
    }
  };

  ws.onerror = () => {};

  ws.onclose = (event) => {
    logDebug(`🔴 [Robot ${robotId}] WebSocket Disconnected (Code: ${event.code}).`, 'warn');
    delete activeWebSockets[robotId];
    if (realTelemetry[robotId]) {
      realTelemetry[robotId].online = false;
      if (typeof resetRobotSensors === 'function') {
        resetRobotSensors(robotId);
      }
    }
    scheduleReconnect(robotId, ip, port);
  };
}

function scheduleReconnect(robotId, ip, port) {
  if (reconnectTimers[robotId]) return;
  if (realTelemetry[robotId] && realTelemetry[robotId].online === false) {
    return;
  }
  reconnectTimers[robotId] = setTimeout(() => {
    delete reconnectTimers[robotId];
    if (realTelemetry[robotId] && realTelemetry[robotId].online !== false) {
      connectToRobotWebSocket(robotId, ip, port);
    }
  }, 4000);
}
