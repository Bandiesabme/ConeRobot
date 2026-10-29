/* ==========================================================================
   ConeRobot Fleet Dashboard - Module 2: ROS 2 Binary CDR & JSON Message Decoder
   Decodes /scan (including 2D point cloud), /odom, /fix, /gps/status,
   /imu/heading, /imu/data, /step_status, /battery_state, /foxglove_bridge/sysinfo
   ========================================================================== */

function cdrAlign(ptr, alignment) {
  // Foxglove binary message has a 13-byte envelope (op:1, subId:4, stamp:8).
  // ROS 2 CDR encapsulation header is 4 bytes (offset 13..16).
  // The CDR serialized data payload starts at byte 17.
  const dataStart = 17;
  const offset = Math.max(0, ptr - dataStart);
  const aligned = (offset + (alignment - 1)) & ~(alignment - 1);
  return dataStart + aligned;
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

function parseSnrAndCorrFromStatus(r, str) {
  if (!r || !str) return;
  const ageMatch = str.match(/Corr\.\s*age:\s*([\d.]+s|-)/i);
  if (ageMatch) {
    r.gps.diff_age = ageMatch[1];
  }

  const cn0Match = str.match(/C\/N0:\s*avg\s*([\d.]+)\s*dB-Hz(?:,\s*(\d+)\/(\d+))?/i);
  if (cn0Match) {
    r.gps.avg_snr = parseFloat(cn0Match[1]);
    if (cn0Match[2] !== undefined) r.gps.strong_signals = parseInt(cn0Match[2], 10);
    if (cn0Match[3] !== undefined) r.gps.total_signals = parseInt(cn0Match[3], 10);
  }

  const snrListMatch = str.match(/SNR:\[([^\]]*)\]/i);
  if (snrListMatch && snrListMatch[1].trim()) {
    const constMap = { G: 'GPS', R: 'GLONASS', E: 'Galileo', B: 'BeiDou', Q: 'QZSS', S: 'GNSS' };
    const sats = [];
    snrListMatch[1].split(',').forEach(tok => {
      const m = tok.trim().match(/^([A-Z]\d{2}):(\d+)(?:@(-?\d+))?$/i);
      if (m) {
        const sid = m[1].toUpperCase();
        const snr = parseFloat(m[2]);
        const elev = m[3] !== undefined ? parseInt(m[3], 10) : null;
        sats.push({
          id: sid,
          constellation: constMap[sid[0]] || 'GNSS',
          snr: snr,
          elev: elev
        });
      }
    });
    if (sats.length > 0) {
      r.gps.snr_sats = sats;
    }
  }
}

function handleBinaryFoxgloveMessage(robotId, buffer, subIdToTopic, channelIdToTopic) {
  const view = new DataView(buffer);
  if (buffer.byteLength < 17) return;
  const op = view.getUint8(0);

  // Opcode 1 = Message Data
  if (op === 1) {
    const subId = view.getUint32(1, true);
    const topic = (subIdToTopic && subIdToTopic[subId]) || (channelIdToTopic && channelIdToTopic[subId]);
    if (!topic) return;

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
          } catch (e) {}
        }
        return;
      }

      let ptr = 17;
      const normTopic = topic.startsWith('/') ? topic : '/' + topic;

      // 1. sensor_msgs/msg/LaserScan (/scan)
      if (normTopic === configuredTopics.scan || normTopic === '/scan' || normTopic.endsWith('/scan')) {
        r.lidar.present = true;
        const now = Date.now();
        if (r.lidar.last_scan_time > 0) {
          const dt = (now - r.lidar.last_scan_time) / 1000;
          if (dt > 0.01) r.lidar.freq_hz = Number((1.0 / dt).toFixed(1));
        }
        r.lidar.last_scan_time = now;

        // Decode CDR LaserScan ranges
        // Only compute 2D cartesian [x, y, dist] points when the modal & 2D LiDAR panel are open for this robot
        const shouldDecodePoints = (Number(selectedRobotId) === Number(robotId) && lidarRadarOpen);
        const isLE = buffer.byteLength > 14 ? (view.getUint8(14) === 1) : true;
        let sPtr = ptr + 8; // Skip stamp (sec:4 + nsec:4)
        if (sPtr + 4 <= buffer.byteLength) {
          const frameLen = view.getUint32(sPtr, isLE);
          if (frameLen < 128 && sPtr + 4 + frameLen <= buffer.byteLength) {
            sPtr += 4 + frameLen;
            sPtr = cdrAlign(sPtr, 4);
            // 7 float32 fields (28 bytes) + uint32 ranges_len (4 bytes) = 32 bytes
            if (sPtr + 32 <= buffer.byteLength) {
              const angleMin = view.getFloat32(sPtr, isLE);
              const angleInc = view.getFloat32(sPtr + 8, isLE);
              const rangeMin = Math.max(0.05, view.getFloat32(sPtr + 20, isLE) || 0.05);
              const rangeMax = Math.min(25.0, view.getFloat32(sPtr + 24, isLE) || 12.0);
              const count = view.getUint32(sPtr + 28, isLE);
              const rangesOffset = sPtr + 32;

              if (count > 0 && count <= 4096 && rangesOffset + count * 4 <= buffer.byteLength) {
                r.lidar.sample_count = count;
                let minD = Infinity;
                let minAngDeg = null;
                const pts = shouldDecodePoints ? [] : null;

                for (let i = 0; i < count; i++) {
                  const d = view.getFloat32(rangesOffset + i * 4, isLE);
                  if (!isNaN(d) && isFinite(d) && d >= rangeMin && d <= rangeMax) {
                    const angleRad = angleMin + i * angleInc;
                    if (d < minD) {
                      minD = d;
                      minAngDeg = (angleRad * 180.0) / Math.PI;
                    }
                    if (shouldDecodePoints) {
                      // Standard ROS LaserFrame: +X forward, +Y left
                      const x = d * Math.cos(angleRad);
                      const y = d * Math.sin(angleRad);
                      pts.push([x, y, d]);
                    }
                  }
                }

                r.lidar.min_dist = minD !== Infinity ? Number(minD.toFixed(2)) : null;
                r.lidar.min_angle_deg = minAngDeg !== null ? Number(minAngDeg.toFixed(0)) : null;
                if (shouldDecodePoints) {
                  r.lidar.points = pts;
                }
              }
            }
          }
        }
      }

      // 1b. nav_msgs/msg/Odometry (/odom)
      else if (normTopic === configuredTopics.odom || normTopic === '/odom' || normTopic.endsWith('/odom')) {
        r.lidar.present = true;
        r.lidar.odom_present = true;
        const isLE = buffer.byteLength > 14 ? (view.getUint8(14) === 1) : true;
        let oPtr = ptr + 8; // Skip stamp (sec:4 + nsec:4)
        if (oPtr + 4 <= buffer.byteLength) {
          const frameLen = view.getUint32(oPtr, isLE);
          if (frameLen < 128 && oPtr + 4 + frameLen <= buffer.byteLength) {
            oPtr += 4 + frameLen;
            oPtr = cdrAlign(oPtr, 4);
            if (oPtr + 4 <= buffer.byteLength) {
              const childLen = view.getUint32(oPtr, isLE);
              if (childLen < 128 && oPtr + 4 + childLen <= buffer.byteLength) {
                oPtr += 4 + childLen;
                oPtr = cdrAlign(oPtr, 8);
                if (oPtr + 24 <= buffer.byteLength) {
                  const ox = view.getFloat64(oPtr, isLE);
                  const oy = view.getFloat64(oPtr + 8, isLE);
                  if (!isNaN(ox) && !isNaN(oy) && isFinite(ox) && isFinite(oy) && Math.abs(ox) < 10000 && Math.abs(oy) < 10000) {
                    r.lidar.odom_x = Number(ox.toFixed(3));
                    r.lidar.odom_y = Number(oy.toFixed(3));
                  }
                  if (oPtr + 352 <= buffer.byteLength) {
                    const vx = view.getFloat64(oPtr + 344, isLE);
                    if (!isNaN(vx) && isFinite(vx) && Math.abs(vx) < 20.0) {
                      r.lidar.odom_vx = Number(vx.toFixed(2));
                    }
                  }
                }
              }
            }
          }
        }
      }

      // 2. sensor_msgs/msg/NavSatFix (/fix)
      else if (normTopic === configuredTopics.gps_fix || normTopic === '/fix' || normTopic.endsWith('/fix')) {
        r.gps.present = true;
        const isLE = buffer.byteLength > 14 ? (view.getUint8(14) === 1) : true;
        let lat = null, lon = null, alt = null;

        let fPtr = ptr + 8;
        if (fPtr + 4 <= buffer.byteLength) {
          const frameLen = view.getUint32(fPtr, isLE);
          if (frameLen < 256 && fPtr + 4 + frameLen <= buffer.byteLength) {
            fPtr += 4 + frameLen;
            const statusInt = view.getInt8(fPtr); fPtr += 1;
            fPtr = cdrAlign(fPtr, 2);
            fPtr += 2; // service (uint16)
            fPtr = cdrAlign(fPtr, 8);

            if (fPtr + 24 <= buffer.byteLength) {
              const cLat = view.getFloat64(fPtr, isLE);
              const cLon = view.getFloat64(fPtr + 8, isLE);
              const cAlt = view.getFloat64(fPtr + 16, isLE);
              if (!isNaN(cLat) && !isNaN(cLon) && isFinite(cLat) && isFinite(cLon) &&
                  Math.abs(cLat) <= 90.0 && Math.abs(cLon) <= 180.0 &&
                  (Math.abs(cLat) > 0.0001 || Math.abs(cLon) > 0.0001)) {
                lat = cLat;
                lon = cLon;
                if (!isNaN(cAlt) && isFinite(cAlt) && Math.abs(cAlt) < 100000.0) alt = cAlt;
              }
            }

            if (statusInt === 2) {
              if (r.gps.mode === 'NO FIX' || r.gps.mode === '3D FIX') r.gps.mode = 'RTK FIX';
            } else if (statusInt === 1) {
              if (r.gps.mode === 'NO FIX' || r.gps.mode === '3D FIX') r.gps.mode = 'DGPS';
            } else if (statusInt === 0) {
              if (r.gps.mode === 'NO FIX') r.gps.mode = '3D FIX';
            }
          }
        }

        if (lat === null) {
          for (let offset = 25; offset <= buffer.byteLength - 16; offset++) {
            let sLat = view.getFloat64(offset, true);
            let sLon = view.getFloat64(offset + 8, true);
            if (!isNaN(sLat) && !isNaN(sLon) && isFinite(sLat) && isFinite(sLon) &&
                Math.abs(sLat) <= 90.0 && Math.abs(sLon) <= 180.0 &&
                (Math.abs(sLat) > 0.0001 || Math.abs(sLon) > 0.0001) &&
                (Math.abs(sLat) >= 0.01 && Math.abs(sLon) >= 0.01)) {
              lat = sLat;
              lon = sLon;
              if (offset + 24 <= buffer.byteLength) {
                const sAlt = view.getFloat64(offset + 16, true);
                if (!isNaN(sAlt) && isFinite(sAlt) && Math.abs(sAlt) < 100000.0) alt = sAlt;
              }
              break;
            }
            sLat = view.getFloat64(offset, false);
            sLon = view.getFloat64(offset + 8, false);
            if (!isNaN(sLat) && !isNaN(sLon) && isFinite(sLat) && isFinite(sLon) &&
                Math.abs(sLat) <= 90.0 && Math.abs(sLon) <= 180.0 &&
                (Math.abs(sLat) > 0.0001 || Math.abs(sLon) > 0.0001) &&
                (Math.abs(sLat) >= 0.01 && Math.abs(sLon) >= 0.01)) {
              lat = sLat;
              lon = sLon;
              if (offset + 24 <= buffer.byteLength) {
                const sAlt = view.getFloat64(offset + 16, false);
                if (!isNaN(sAlt) && isFinite(sAlt) && Math.abs(sAlt) < 100000.0) alt = sAlt;
              }
              break;
            }
          }
        }

        if (lat !== null && lon !== null) {
          r.gps.lat = Number(lat.toFixed(7));
          r.gps.lon = Number(lon.toFixed(7));
          if (alt !== null && !isNaN(alt) && isFinite(alt)) r.gps.alt = Number(alt.toFixed(1));
          if (r.gps.mode === 'NO FIX') r.gps.mode = '3D FIX';
          logDebug(`📍 [Robot ${robotId}] GPS Fix: ${r.gps.lat}, ${r.gps.lon} (Alt: ${r.gps.alt !== null ? r.gps.alt + 'm' : '--'})`, 'topic');
        }
      }

      // 3. std_msgs/msg/Float32 (/imu/heading)
      else if (normTopic === configuredTopics.heading || normTopic === '/imu/heading' || normTopic.endsWith('/heading')) {
        if (ptr + 4 <= buffer.byteLength) {
          let val = view.getFloat32(ptr, true);
          if (!isNaN(val) && isFinite(val)) {
            val = ((val % 360) + 360) % 360;
            r.imu.heading = Number(val.toFixed(1));
            r.imu.last_heading_topic_time = Date.now();
          }
        }
      }

      // 4. std_msgs/msg/String (/gps/status)
      else if (normTopic === configuredTopics.gps_status || normTopic === '/gps/status' || normTopic.endsWith('/gps/status')) {
        r.gps.present = true;
        let str = '';

        if (ptr + 4 <= buffer.byteLength) {
          const strLen = view.getUint32(ptr, true);
          if (strLen > 0 && strLen < 10000 && ptr + 4 < buffer.byteLength) {
            const actualLen = Math.min(strLen, buffer.byteLength - (ptr + 4));
            const strBytes = new Uint8Array(buffer, ptr + 4, actualLen);
            str = new TextDecoder('utf-8', { fatal: false }).decode(strBytes).replace(/\0/g, '').trim();
          }
        }

        if (!str || !str.includes('Fix:')) {
          try {
            const payloadBytes = new Uint8Array(buffer, 13);
            const rawPayload = new TextDecoder('utf-8', { fatal: false }).decode(payloadBytes);
            const fixIdx = rawPayload.indexOf('Fix:');
            if (fixIdx !== -1) {
              str = rawPayload.substring(fixIdx).replace(/\0/g, '').trim();
            }
          } catch (e) {}
        }

        if (str) {
          const fixMatch = str.match(/Fix:\s*([^|]+)/i);
          const fixStr = (fixMatch ? fixMatch[1] : str).trim().toUpperCase();
          if (fixStr.includes('RTK FIX') || fixStr === '4') r.gps.mode = 'RTK FIX';
          else if (fixStr.includes('FLOAT') || fixStr === '5') r.gps.mode = 'FLOAT';
          else if (fixStr.includes('DGPS') || fixStr === '2') r.gps.mode = 'DGPS';
          else if (fixStr.includes('3D') || fixStr === '1') r.gps.mode = '3D FIX';
          else if (fixStr.includes('NO FIX') || fixStr === '0') r.gps.mode = 'NO FIX';
          else r.gps.mode = fixStr || '3D FIX';

          const satMatch = str.match(/Sats:\s*(\d+)/i) || str.match(/(\d+)\s*sats/i);
          if (satMatch) r.gps.sats = parseInt(satMatch[1]);

          const hdopMatch = str.match(/HDOP:\s*([\d.]+)/i);
          if (hdopMatch) r.gps.hdop = hdopMatch[1];

          const ntripMatch = str.match(/NTRIP:\s*([^(|]+)(?:\(([^)]+)\))?/i);
          if (ntripMatch) {
            r.gps.ntrip = ntripMatch[1].trim();
            r.gps.rtcm = ntripMatch[2] ? ntripMatch[2].trim() : '';
          }

          parseSnrAndCorrFromStatus(r, str);

          let pMatch = str.match(/Pos:\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)(?:\s*,\s*([-\d.]+)m?)?\s*\)/i);
          if (!pMatch) {
            const latM = str.match(/Lat(?:itude)?:\s*([-\d.]+)/i);
            const lonM = str.match(/Lon(?:gitude)?:\s*([-\d.]+)/i);
            if (latM && lonM) {
              const altM = str.match(/Alt(?:itude)?:\s*([-\d.]+)/i);
              pMatch = [null, latM[1], lonM[1], altM ? altM[1] : null];
            }
          }
          if (!pMatch) {
            pMatch = str.match(/Pos:\s*([-\d.]+)\s*,\s*([-\d.]+)(?:\s*,\s*([-\d.]+)m?)?/i);
          }
          if (!pMatch) {
            const decM = str.match(/\b([0-8]?\d\.\d{4,9})\s*,\s*([0-1]?\d{1,2}\.\d{4,9})\b/);
            if (decM) {
              pMatch = [null, decM[1], decM[2], null];
            }
          }
          if (pMatch) {
            const pLat = parseFloat(pMatch[1]);
            const pLon = parseFloat(pMatch[2]);
            if (!isNaN(pLat) && !isNaN(pLon) && isFinite(pLat) && isFinite(pLon) &&
                Math.abs(pLat) <= 90.0 && Math.abs(pLon) <= 180.0 &&
                (Math.abs(pLat) > 0.0001 || Math.abs(pLon) > 0.0001)) {
              r.gps.lat = Number(pLat.toFixed(7));
              r.gps.lon = Number(pLon.toFixed(7));
              if (pMatch[3]) {
                const pAlt = parseFloat(pMatch[3]);
                if (!isNaN(pAlt) && isFinite(pAlt)) r.gps.alt = Number(pAlt.toFixed(1));
              }
            }
          }

          logDebug(`🛰️ [Robot ${robotId}] GPS Status: ${r.gps.mode} (${r.gps.sats} Sats, HDOP ${r.gps.hdop}) | Base: ${r.gps.ntrip}`, 'topic');
        }
      }

      // 5. std_msgs/msg/String (/step_status)
      else if (normTopic === configuredTopics.step_status || normTopic === '/step_status' || normTopic.endsWith('/step_status')) {
        let str = '';
        if (ptr + 4 <= buffer.byteLength) {
          const strLen = view.getUint32(ptr, true);
          if (strLen > 0 && strLen < 10000 && ptr + 4 < buffer.byteLength) {
            const actualLen = Math.min(strLen, buffer.byteLength - (ptr + 4));
            const strBytes = new Uint8Array(buffer, ptr + 4, actualLen);
            str = new TextDecoder('utf-8', { fatal: false }).decode(strBytes).replace(/\0/g, '').trim();
          }
        }
        if (!str) {
          try {
            const payloadBytes = new Uint8Array(buffer, 13);
            str = new TextDecoder('utf-8', { fatal: false }).decode(payloadBytes).replace(/\0/g, '').trim();
          } catch (e) {}
        }
        if (str) {
          r.motion.step_progress = str;
          r.motion.state = str.includes('Driving') || str.includes('Turning') || str.includes('ARC') || str.includes('MOVE') || str.includes('TURN') ? 'Moving' : 'Standby';
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
          } catch (e) {
            if (str.includes('°C') || str.includes('C')) r.health.cpu_temp = str;
          }
        }
      }

      // 7. sensor_msgs/msg/Imu (/imu/data)
      else if (normTopic === configuredTopics.imu_data || normTopic === '/imu/data' || normTopic.endsWith('/imu/data')) {
        const isLE = buffer.byteLength > 14 ? (view.getUint8(14) === 1) : true;
        ptr += 8;
        if (ptr + 4 <= buffer.byteLength) {
          const frameLen = view.getUint32(ptr, isLE);
          if (frameLen < 256 && ptr + 4 + frameLen <= buffer.byteLength) {
            ptr += 4 + frameLen;
            ptr = cdrAlign(ptr, 8);

            if (ptr + 32 <= buffer.byteLength) {
              const x = view.getFloat64(ptr, isLE); ptr += 8;
              const y = view.getFloat64(ptr, isLE); ptr += 8;
              const z = view.getFloat64(ptr, isLE); ptr += 8;
              const w = view.getFloat64(ptr, isLE); ptr += 8;

              const normSq = x * x + y * y + z * z + w * w;
              if (!isNaN(normSq) && normSq > 0.0001 && normSq < 4.0) {
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

                const hasFreshHeadingTopic = r.imu.last_heading_topic_time && (Date.now() - r.imu.last_heading_topic_time < 2000);
                if (!hasFreshHeadingTopic) {
                  r.imu.heading = Number(yaw.toFixed(1));
                }
              }
            }
          }
        }
      }

      // 8. sensor_msgs/msg/BatteryState (/battery_state)
      else if (topic === configuredTopics.battery || topic === '/battery_state' || (topic && (topic.includes('battery') || topic.endsWith('/battery_state')))) {
        let volt = null;
        let pct = null;

        let bPtr = ptr + 8;
        if (bPtr + 4 <= buffer.byteLength) {
          const frameLen = view.getUint32(bPtr, true);
          if (frameLen < 128 && bPtr + 4 + frameLen <= buffer.byteLength) {
            bPtr += 4 + frameLen;
            bPtr = cdrAlign(bPtr, 4);

            if (bPtr + 28 <= buffer.byteLength) {
              const vCand = view.getFloat32(bPtr, true);
              if (!isNaN(vCand) && vCand >= 3.0 && vCand <= 36.0) {
                volt = vCand;
                const pCand = view.getFloat32(bPtr + 24, true);
                if (!isNaN(pCand) && pCand >= 0.0 && pCand <= 100.0) {
                  pct = pCand;
                }
              }
            }
          }
        }

        if (volt === null) {
          for (let offset = 17; offset <= buffer.byteLength - 28; offset += 4) {
            const vCand = view.getFloat32(offset, true);
            if (!isNaN(vCand) && vCand >= 5.0 && vCand <= 30.0) {
              volt = vCand;
              const pCand = view.getFloat32(offset + 24, true);
              if (!isNaN(pCand) && pCand >= 0.0 && pCand <= 100.0) {
                pct = pCand;
              }
              break;
            }
          }
        }

        if (volt === null) {
          for (let offset = 17; offset <= buffer.byteLength - 4; offset += 2) {
            const vCand = view.getFloat32(offset, true);
            if (!isNaN(vCand) && vCand >= 5.0 && vCand <= 30.0) {
              volt = vCand;
              break;
            }
          }
        }

        if (volt !== null && volt > 0.0) {
          const vFormatted = Number(volt).toFixed(1);
          let pctVal = null;
          if (pct !== null && !isNaN(pct) && pct >= 0) {
            pctVal = Math.round(pct <= 1.0 ? pct * 100 : pct);
          }
          const prevBatt = r.health.battery_v;
          r.health.battery_v = pctVal !== null ? `${vFormatted}V (${pctVal}%)` : `${vFormatted}V`;
          if (prevBatt !== r.health.battery_v) {
            logDebug(`🔋 [Robot ${robotId}] Battery: ${r.health.battery_v}`, 'topic');
          }
        }
      }

    } catch (e) {}
  }
}

function handleParsedRobotMessage(robotId, topic, data) {
  const r = realTelemetry[robotId];
  if (!r) return;

  const normTopic = topic ? (topic.startsWith('/') ? topic : '/' + topic) : '';

  // 1. LiDAR LaserScan (/scan)
  if (normTopic === configuredTopics.scan || normTopic === '/scan' || normTopic.endsWith('/scan')) {
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
      const shouldDecodePoints = (Number(selectedRobotId) === Number(robotId) && lidarRadarOpen);
      const angleMin = (data.angle_min !== undefined) ? Number(data.angle_min) : -Math.PI;
      const angleInc = (data.angle_increment !== undefined)
        ? Number(data.angle_increment)
        : (2 * Math.PI / Math.max(1, data.ranges.length));
      const rangeMin = Math.max(0.05, Number(data.range_min) || 0.05);
      const rangeMax = Math.min(25.0, Number(data.range_max) || 12.0);

      r.lidar.sample_count = data.ranges.length;
      let minD = Infinity;
      let minAngDeg = null;
      const pts = shouldDecodePoints ? [] : null;

      for (let i = 0; i < data.ranges.length; i++) {
        const d = data.ranges[i];
        if (d !== null && !isNaN(d) && isFinite(d) && d >= rangeMin && d <= rangeMax) {
          const angleRad = angleMin + i * angleInc;
          if (d < minD) {
            minD = d;
            minAngDeg = (angleRad * 180.0) / Math.PI;
          }
          if (shouldDecodePoints) {
            pts.push([d * Math.cos(angleRad), d * Math.sin(angleRad), d]);
          }
        }
      }
      r.lidar.min_dist = minD !== Infinity ? Number(minD.toFixed(2)) : null;
      r.lidar.min_angle_deg = minAngDeg !== null ? Number(minAngDeg.toFixed(0)) : null;
      if (shouldDecodePoints) {
        r.lidar.points = pts;
      }
    }
  }

  // 1b. LiDAR Laser Odometry (/odom)
  if (normTopic === configuredTopics.odom || normTopic === '/odom' || normTopic.endsWith('/odom')) {
    r.lidar.present = true;
    r.lidar.odom_present = true;
    const pos = data.pose?.pose?.position || data.pose?.position || data.position;
    if (pos && pos.x !== undefined && pos.y !== undefined) {
      r.lidar.odom_x = Number(Number(pos.x).toFixed(3));
      r.lidar.odom_y = Number(Number(pos.y).toFixed(3));
    }
    const lin = data.twist?.twist?.linear || data.twist?.linear;
    if (lin && lin.x !== undefined) {
      r.lidar.odom_vx = Number(Number(lin.x).toFixed(2));
    }
  }

  // 2. GPS Fix (/fix)
  if (normTopic === configuredTopics.gps_fix || normTopic === '/fix' || normTopic.endsWith('/fix')) {
    r.gps.present = true;
    if (data.latitude !== undefined && data.longitude !== undefined) {
      r.gps.lat = Number(data.latitude);
      r.gps.lon = Number(data.longitude);
      r.gps.alt = data.altitude ? Number(data.altitude).toFixed(1) : '--';
      const statusInt = data.status?.status ?? data.status;
      if (statusInt === 2) {
        if (r.gps.mode === 'NO FIX' || r.gps.mode === '3D FIX') r.gps.mode = 'RTK FIX';
      } else if (statusInt === 1) {
        if (r.gps.mode === 'NO FIX' || r.gps.mode === '3D FIX') r.gps.mode = 'DGPS';
      } else if (statusInt >= 0 && r.gps.mode === 'NO FIX') {
        r.gps.mode = '3D FIX';
      }
    }
  }

  // 3. GPS Status (/gps/status)
  if (normTopic === configuredTopics.gps_status || normTopic === '/gps/status' || normTopic.endsWith('/gps/status')) {
    r.gps.present = true;
    const str = typeof data === 'string' ? data : (data.data || '');
    if (str) {
      const fixMatch = str.match(/Fix:\s*([^|]+)/i);
      const fixStr = (fixMatch ? fixMatch[1] : str).trim().toUpperCase();
      if (fixStr.includes('RTK FIX') || fixStr === '4') r.gps.mode = 'RTK FIX';
      else if (fixStr.includes('FLOAT') || fixStr === '5') r.gps.mode = 'FLOAT';
      else if (fixStr.includes('DGPS') || fixStr === '2') r.gps.mode = 'DGPS';
      else if (fixStr.includes('3D') || fixStr === '1') r.gps.mode = '3D FIX';
      else if (fixStr.includes('NO FIX') || fixStr === '0') r.gps.mode = 'NO FIX';
      else r.gps.mode = fixStr || '3D FIX';

      const satMatch = str.match(/Sats:\s*(\d+)/i) || str.match(/(\d+)\s*sats/i);
      if (satMatch) r.gps.sats = parseInt(satMatch[1]);

      const hdopMatch = str.match(/HDOP:\s*([\d.]+)/i);
      if (hdopMatch) r.gps.hdop = hdopMatch[1];

      const ntripMatch = str.match(/NTRIP:\s*([^(|]+)(?:\(([^)]+)\))?/i);
      if (ntripMatch) {
        r.gps.ntrip = ntripMatch[1].trim();
        r.gps.rtcm = ntripMatch[2] ? ntripMatch[2].trim() : '';
      }

      parseSnrAndCorrFromStatus(r, str);

      let pMatch = str.match(/Pos:\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)(?:\s*,\s*([-\d.]+)m?)?\s*\)/i);
      if (!pMatch) {
        const latM = str.match(/Lat(?:itude)?:\s*([-\d.]+)/i);
        const lonM = str.match(/Lon(?:gitude)?:\s*([-\d.]+)/i);
        if (latM && lonM) {
          const altM = str.match(/Alt(?:itude)?:\s*([-\d.]+)/i);
          pMatch = [null, latM[1], lonM[1], altM ? altM[1] : null];
        }
      }
      if (!pMatch) {
        pMatch = str.match(/Pos:\s*([-\d.]+)\s*,\s*([-\d.]+)(?:\s*,\s*([-\d.]+)m?)?/i);
      }
      if (!pMatch) {
        const decM = str.match(/\b([0-8]?\d\.\d{4,9})\s*,\s*([0-1]?\d{1,2}\.\d{4,9})\b/);
        if (decM) {
          pMatch = [null, decM[1], decM[2], null];
        }
      }
      if (pMatch) {
        const pLat = parseFloat(pMatch[1]);
        const pLon = parseFloat(pMatch[2]);
        if (!isNaN(pLat) && !isNaN(pLon) && isFinite(pLat) && isFinite(pLon) &&
            Math.abs(pLat) <= 90.0 && Math.abs(pLon) <= 180.0 &&
            (Math.abs(pLat) > 0.0001 || Math.abs(pLon) > 0.0001)) {
          r.gps.lat = Number(pLat.toFixed(7));
          r.gps.lon = Number(pLon.toFixed(7));
          if (pMatch[3]) {
            const pAlt = parseFloat(pMatch[3]);
            if (!isNaN(pAlt) && isFinite(pAlt)) r.gps.alt = Number(pAlt.toFixed(1));
          }
        }
      }
    }
  }

  // 4. IMU Heading (/imu/heading)
  if (topic === configuredTopics.heading || topic === '/imu/heading' || (topic && topic.endsWith('/heading'))) {
    const val = typeof data === 'number' ? data : (data.data !== undefined ? Number(data.data) : null);
    if (val !== null && !isNaN(val) && isFinite(val)) {
      r.imu.heading = Number((((val % 360) + 360) % 360).toFixed(1));
      r.imu.last_heading_topic_time = Date.now();
    }
  }

  // 5. IMU Data Orientation (/imu/data)
  if (topic === configuredTopics.imu_data || topic === '/imu/data' || (topic && topic.endsWith('/imu/data'))) {
    if (data.orientation) {
      const { x, y, z, w } = data.orientation;
      const normSq = x * x + y * y + z * z + w * w;
      if (!isNaN(normSq) && normSq > 0.0001 && normSq < 4.0) {
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

        const hasFreshHeadingTopic = r.imu.last_heading_topic_time && (Date.now() - r.imu.last_heading_topic_time < 2000);
        if (!hasFreshHeadingTopic) {
          r.imu.heading = Number(yaw.toFixed(1));
        }
      }
    }
  }

  // 6. Step Motion Status (/step_status)
  if (topic === configuredTopics.step_status || topic === '/step_status') {
    const stepStr = typeof data === 'string' ? data : (data.data || '--');
    r.motion.step_progress = stepStr;
    r.motion.state = stepStr.includes('Driving') || stepStr.includes('Turning') || stepStr.includes('ARC') || stepStr.includes('MOVE') || stepStr.includes('TURN') ? 'Moving' : 'Standby';
  }

  // 7. Battery State (/battery_state)
  if (topic === configuredTopics.battery || topic === '/battery_state' || (topic && (topic.includes('battery') || topic.endsWith('/battery_state')))) {
    if (data.voltage !== undefined) {
      const v = Number(data.voltage).toFixed(1);
      const pct = (data.percentage !== undefined && data.percentage !== null && !isNaN(data.percentage))
        ? Math.round(Number(data.percentage) * (data.percentage <= 1.0 ? 100 : 1))
        : null;
      const prevBatt = r.health.battery_v;
      r.health.battery_v = pct !== null ? `${v}V (${pct}%)` : `${v}V`;
      if (prevBatt !== r.health.battery_v) {
        logDebug(`🔋 [Robot ${robotId}] Battery (JSON): ${r.health.battery_v}`, 'topic');
      }
    }
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
      } catch (e) {
        if (data.includes('°C') || data.includes('C')) r.health.cpu_temp = data;
      }
    }
  }
}
