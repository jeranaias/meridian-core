/* ============================================================
   map.js — Leaflet map with vehicle icon, trail, home,
   waypoints, center/follow toggles.
   ============================================================ */

'use strict';

window.FlyMap = (function () {

    let map;
    let vehicleMarker, homeMarker, trailLine, trajectoryLine, homeGuideLine, flyMissionLine;
    let uncertaintyCircle = null;
    let crosstrackLine = null;
    let secondaryVehicleMarkers = {};
    let followMode = true;
    let waypointMarkers = [];
    let missionPolyline = null;
    let planModeActive = false;
    let initialized = false;

    // SVG vehicle icon (drone shape pointing up)
    // Vehicle icons by type
    const VEHICLE_SVGS = {
        quad: `<svg viewBox="0 0 36 36" xmlns="http://www.w3.org/2000/svg">
            <g fill="none" stroke="#0891b2" stroke-width="1.8" stroke-linecap="round">
                <line x1="18" y1="4" x2="18" y2="14"/><line x1="18" y1="22" x2="18" y2="32"/>
                <line x1="4" y1="18" x2="14" y2="18"/><line x1="22" y1="18" x2="32" y2="18"/>
                <circle cx="18" cy="18" r="4" fill="rgba(0,229,255,0.2)"/>
                <circle cx="7" cy="7" r="3" stroke-opacity="0.4"/><circle cx="29" cy="7" r="3" stroke-opacity="0.4"/>
                <circle cx="7" cy="29" r="3" stroke-opacity="0.4"/><circle cx="29" cy="29" r="3" stroke-opacity="0.4"/>
            </g>
            <polygon points="18,6 15,12 21,12" fill="#0891b2" opacity="0.8"/>
        </svg>`,
        boat: `<svg viewBox="0 0 36 36" xmlns="http://www.w3.org/2000/svg">
            <g fill="none" stroke="#0891b2" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
                <path d="M8,24 L6,20 L18,8 L30,20 L28,24 Z" fill="rgba(0,229,255,0.15)"/>
                <line x1="18" y1="8" x2="18" y2="18" stroke-width="1.5"/>
                <path d="M6,27 Q12,30 18,27 Q24,24 30,27" stroke-opacity="0.4"/>
            </g>
            <polygon points="18,6 16,10 20,10" fill="#0891b2" opacity="0.9"/>
        </svg>`,
        plane: `<svg viewBox="0 0 36 36" xmlns="http://www.w3.org/2000/svg">
            <g fill="none" stroke="#0891b2" stroke-width="1.8" stroke-linecap="round">
                <line x1="18" y1="4" x2="18" y2="32"/>
                <line x1="6" y1="16" x2="30" y2="16"/>
                <line x1="12" y1="28" x2="24" y2="28"/>
            </g>
            <polygon points="18,4 15,10 21,10" fill="#0891b2" opacity="0.8"/>
        </svg>`,
        rover: `<svg viewBox="0 0 36 36" xmlns="http://www.w3.org/2000/svg">
            <g fill="none" stroke="#0891b2" stroke-width="2" stroke-linecap="round">
                <rect x="10" y="10" width="16" height="20" rx="3" fill="rgba(0,229,255,0.15)"/>
                <circle cx="12" cy="12" r="2.5"/><circle cx="24" cy="12" r="2.5"/>
                <circle cx="12" cy="28" r="2.5"/><circle cx="24" cy="28" r="2.5"/>
            </g>
            <polygon points="18,6 16,10 20,10" fill="#0891b2" opacity="0.8"/>
        </svg>`,
    };

    let currentVehicleType = 'quad';

    function getVehicleSvg() {
        return VEHICLE_SVGS[currentVehicleType] || VEHICLE_SVGS.quad;
    }

    function setVehicleType(type) {
        if (VEHICLE_SVGS[type] && type !== currentVehicleType) {
            currentVehicleType = type;
            if (vehicleMarker && map) {
                var newIcon = L.divIcon({
                    className: 'vehicle-icon',
                    html: getVehicleSvg(),
                    iconSize: [36, 36],
                });
                vehicleMarker.setIcon(newIcon);
            }
        }
    }

    const vehicleSvg = VEHICLE_SVGS.quad; // default

    // SVG home icon
    const homeSvg = `<svg viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
        <polygon points="12,2 2,12 6,12 6,22 18,22 18,12 22,12"
            fill="rgba(224,64,251,0.3)" stroke="#e040fb" stroke-width="1.5"/>
        <rect x="9" y="14" width="6" height="8" fill="rgba(224,64,251,0.5)"/>
    </svg>`;

    function init() {
        if (initialized) return;
        initialized = true;

        // Light tile layer (CartoDB Voyager — clean, modern)
        const tileUrl = 'https://{s}.basemaps.cartocdn.com/rastertiles/voyager/{z}/{x}/{y}{r}.png';
        const tileAttr = '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a> &copy; <a href="https://carto.com/">CARTO</a>';

        map = L.map('map', {
            center: [35.7516, -120.7710],
            zoom: 17,
            zoomControl: false,  // We use our own toolbar
            attributionControl: true,
        });

        L.tileLayer(tileUrl, {
            attribution: tileAttr,
            subdomains: 'abcd',
            maxZoom: 19,
        }).addTo(map);

        // Vehicle marker
        const vehicleIcon = L.divIcon({
            className: 'vehicle-icon',
            html: vehicleSvg,
            iconSize: [36, 36],
        });
        vehicleMarker = L.marker([0, 0], { icon: vehicleIcon, zIndexOffset: 1000, draggable: true }).addTo(map);

        // Home marker
        const homeIcon = L.divIcon({
            className: 'home-icon',
            html: homeSvg,
            iconSize: [24, 24],
        });
        homeMarker = L.marker([0, 0], { icon: homeIcon }).addTo(map);
        homeMarker.setOpacity(0);

        // Trail polyline — subtle, fading
        trailLine = L.polyline([], {
            color: '#0891b2',
            weight: 2,
            opacity: 0.5,
            smoothFactor: 1,
        }).addTo(map);

        // Short velocity vector (2 second projection)
        trajectoryLine = L.polyline([], {
            color: '#0891b2',
            weight: 2,
            opacity: 0.4,
            smoothFactor: 1,
        }).addTo(map);

        // Home guide line and fly mission line are opt-in, not created by default
        homeGuideLine = null;
        flyMissionLine = null;

        // Position uncertainty ellipse (Victor: HDOP * EKF pos variance * scale)
        uncertaintyCircle = L.circle([0, 0], {
            radius: 1,
            color: '#00e5ff',
            weight: 1,
            fillColor: '#00e5ff',
            fillOpacity: 0.08,
            opacity: 0.3,
            interactive: false,
        }).addTo(map);
        uncertaintyCircle.setStyle({ opacity: 0 });

        // Crosstrack deviation line (Victor: perpendicular from planned path to vehicle)
        crosstrackLine = L.polyline([], {
            color: '#dc2626',
            weight: 2.5,
            opacity: 0.8,
            smoothFactor: 1,
        }).addTo(map);

        // Vehicle switch: recenter map and reset trail
        meridian.events.on('vehicle_switch', function () {
            var v = meridian.v;
            if (v && v.lat && v.lon) {
                map.setView([v.lat, v.lon], map.getZoom(), { animate: true });
                trailLine.setLatLngs(v.trail || []);
            }
        });

        // Drag-to-fly: drag vehicle icon to set guided target (Victor: direct manipulation)
        let dragTarget = null;
        let dragLine = null;

        vehicleMarker.on('dragstart', function () {
            dragLine = L.polyline([], {
                color: '#9333ea',
                weight: 2,
                dashArray: '4,4',
                opacity: 0.8,
            }).addTo(map);
        });

        vehicleMarker.on('drag', function (e) {
            const v = meridian.v;
            if (v && dragLine) {
                dragLine.setLatLngs([[v.lat, v.lon], [e.latlng.lat, e.latlng.lng]]);
            }
            dragTarget = e.latlng;
        });

        vehicleMarker.on('dragend', async function () {
            if (dragLine) { map.removeLayer(dragLine); dragLine = null; }
            if (!dragTarget) return;

            const v = meridian.v;
            if (!v || !v.armed) {
                meridian.log('Cannot fly: not armed', 'warn');
                // Snap marker back
                if (v) vehicleMarker.setLatLng([v.lat, v.lon]);
                return;
            }

            const alt = v.relativeAlt > 0 ? v.relativeAlt : 10;
            const dist = map.distance([v.lat, v.lon], dragTarget);

            // T2-6: Block if no home or target exceeds guidedDistMax
            if (v.homeLat === null || v.homeLon === null) {
                meridian.log('Cannot fly: no home position set', 'warn');
                vehicleMarker.setLatLng([v.lat, v.lon]);
                await Modal.confirm('No Home Position',
                    'Home position is not set. Cannot validate distance limit. Set home first.',
                    'OK');
                dragTarget = null;
                return;
            }

            const maxDist = (meridian.settings && meridian.settings.guidedDistMax) || 1000;
            const homeDist = map.distance([v.homeLat, v.homeLon], dragTarget);
            if (homeDist > maxDist) {
                meridian.log('Drag-to-fly target exceeds distance limit: ' + Math.round(homeDist) + 'm (max ' + maxDist + 'm)', 'warn');
                vehicleMarker.setLatLng([v.lat, v.lon]);
                await Modal.confirm('Distance Limit Exceeded',
                    'Target is ' + Math.round(homeDist) + 'm from home. Maximum guided distance is ' + maxDist + 'm.\n' +
                    'Adjust the limit in Settings → Guided Dist Max to proceed.',
                    'OK');
                dragTarget = null;
                return;
            }

            const ok = await Modal.confirm('Fly Here',
                'Fly to dropped position? Distance: ' + Math.round(dist) + 'm',
                'Go');
            if (ok) {
                if (v) v._userModeChange = true;
                Connection.sendSetMode('GUIDED');
                const onAck = () => {
                    Connection.sendGoto(dragTarget.lat, dragTarget.lng, alt);
                    meridian.events.off('command_ack', onAck);
                };
                meridian.events.on('command_ack', onAck);
                setTimeout(() => {
                    meridian.events.off('command_ack', onAck);
                    Connection.sendGoto(dragTarget.lat, dragTarget.lng, alt);
                }, 2000);
                meridian.log('Flying to ' + dragTarget.lat.toFixed(6) + ', ' + dragTarget.lng.toFixed(6), 'info');
            } else {
                // Snap back
                if (v) vehicleMarker.setLatLng([v.lat, v.lon]);
            }
            dragTarget = null;
        });

        // Right-click context menu
        map.on('contextmenu', function (e) {
            e.originalEvent.preventDefault();
            ContextMenu.show(e.originalEvent.clientX, e.originalEvent.clientY, e.latlng.lat, e.latlng.lng);
        });

        // Click to close context menu
        map.on('click', function () {
            ContextMenu.hide();
        });

        // Disable follow on user drag
        map.on('dragstart', function () {
            followMode = false;
            updateFollowBtn();
        });

        // Listen for events
        meridian.events.on('position', updateVehicle);
        meridian.events.on('home_changed', updateHome);

        // Initial invalidation (Leaflet needs this after DOM mount)
        setTimeout(() => map.invalidateSize(), 100);
    }

    function updateVehicle(v) {
        if (!map || !v || v.lat === 0 && v.lon === 0) return;

        // Update vehicle position and heading
        vehicleMarker.setLatLng([v.lat, v.lon]);
        // Set rotation directly on SVG, not accumulating on container (Meier fix)
        const iconEl = vehicleMarker.getElement();
        if (iconEl) {
            const svg = iconEl.querySelector('svg');
            if (svg) svg.style.transform = 'rotate(' + v.heading + 'deg)';
        }

        // Update trail
        trailLine.setLatLngs(v.trail);

        // Commanded trajectory: project TRUE velocity vector 5s ahead (Victor R2 fix)
        // Uses vx/vy (actual velocity) not heading (which lags in wind)
        if (trajectoryLine && v.groundspeed > 0.3) {
            const points = [[v.lat, v.lon]];
            // vx/vy are in m/s NED frame
            const vxMs = v.vx * 100; // stored as cm/s in some sources
            const vyMs = v.vy * 100;
            const speed = Math.sqrt(vxMs * vxMs + vyMs * vyMs);
            if (speed > 0.3) {
                for (let t = 1; t <= 2; t++) {
                    const dlat = (vxMs * t) / 111320;
                    const dlon = (vyMs * t) / (111320 * Math.cos(v.lat * Math.PI / 180));
                    points.push([v.lat + dlat, v.lon + dlon]);
                }
            } else {
                // Fallback to heading if vx/vy unavailable
                const hdgRad = v.heading * Math.PI / 180;
                for (let t = 1; t <= 2; t++) {
                    const dist = v.groundspeed * t;
                    const dlat = (dist * Math.cos(hdgRad)) / 111320;
                    const dlon = (dist * Math.sin(hdgRad)) / (111320 * Math.cos(v.lat * Math.PI / 180));
                    points.push([v.lat + dlat, v.lon + dlon]);
                }
            }
            trajectoryLine.setLatLngs(points);
        } else if (trajectoryLine) {
            trajectoryLine.setLatLngs([]);
        }

        // Position uncertainty ellipse (Victor)
        if (uncertaintyCircle) {
            if (meridian.settings.showUncertainty && v.hdop < 50 && v.ekfPosVar > 0) {
                var SCALE = 5.0;
                var radius = v.hdop * v.ekfPosVar * SCALE;
                radius = Math.max(1, Math.min(radius, 200)); // clamp 1-200m
                uncertaintyCircle.setLatLng([v.lat, v.lon]);
                uncertaintyCircle.setRadius(radius);
                uncertaintyCircle.setStyle({
                    opacity: 0.3,
                    fillOpacity: 0.08,
                });
            } else {
                uncertaintyCircle.setStyle({ opacity: 0, fillOpacity: 0 });
            }
        }

        // Crosstrack deviation (Victor): show when AUTO mode + mission loaded
        if (crosstrackLine) {
            if (v.modeName === 'AUTO' && missionPolyline) {
                var missionCoords = missionPolyline.getLatLngs();
                if (missionCoords && missionCoords.length >= 2) {
                    var closest = findClosestPointOnPath(v.lat, v.lon, missionCoords);
                    if (closest) {
                        crosstrackLine.setLatLngs([[v.lat, v.lon], closest]);
                    } else {
                        crosstrackLine.setLatLngs([]);
                    }
                } else {
                    crosstrackLine.setLatLngs([]);
                }
            } else {
                crosstrackLine.setLatLngs([]);
            }
        }

        // Home guide and mission path lines disabled for now (too cluttered)

        // Follow vehicle — no animation to prevent jitter at 10Hz
        if (followMode) {
            map.setView([v.lat, v.lon], map.getZoom(), { animate: false });
        }

        // First position: center map
        if (v.trail.length === 1) {
            map.setView([v.lat, v.lon], 17);
        }
    }

    function updateHome(v) {
        if (!v || !v.homeLat) return;
        homeMarker.setLatLng([v.homeLat, v.homeLon]);
        homeMarker.setOpacity(1);
    }

    function toggleFollow() {
        followMode = !followMode;
        updateFollowBtn();
        if (followMode) {
            const v = meridian.v;
            if (v && v.lat !== 0) {
                map.setView([v.lat, v.lon], map.getZoom());
            }
        }
    }

    function updateFollowBtn() {
        const btn = document.getElementById('btn-follow');
        if (btn) btn.classList.toggle('active', followMode);
    }

    function centerOnVehicle() {
        const v = meridian.v;
        if (v && v.lat !== 0) {
            map.setView([v.lat, v.lon], 16);
            followMode = true;
            updateFollowBtn();
        }
    }

    function getMap() { return map; }

    // --- Plan Mode: click-to-add, waypoint markers, mission polyline ---

    function enablePlanMode(enable) {
        planModeActive = enable;
        if (map) {
            if (enable) {
                map.on('click', onPlanClick);
                map.getContainer().style.cursor = 'crosshair';
            } else {
                map.off('click', onPlanClick);
                map.getContainer().style.cursor = '';
            }
        }
    }

    function onPlanClick(e) {
        if (!planModeActive) return;
        // Don't add waypoint if clicking on an existing marker
        if (e.originalEvent && e.originalEvent._wpMarkerClick) return;
        // Don't add waypoint if geofence drawing is active
        if (window.Geofence && Geofence.isDrawing()) return;
        Mission.addWaypoint(e.latlng.lat, e.latlng.lng, 10);
    }

    function updateMissionMarkers(items, selectedIdx) {
        if (!map) return;

        // Remove old markers
        for (var i = 0; i < waypointMarkers.length; i++) {
            map.removeLayer(waypointMarkers[i]);
        }
        waypointMarkers = [];

        // Remove old polyline
        if (missionPolyline) {
            map.removeLayer(missionPolyline);
            missionPolyline = null;
        }

        if (!items || items.length === 0) return;

        var pathCoords = [];

        for (var i = 0; i < items.length; i++) {
            var it = items[i];
            if (!it.lat && !it.lon) continue;

            var latlng = [it.lat, it.lon];
            pathCoords.push(latlng);

            // Determine marker CSS class based on command
            var seqClass = '';
            if (window.Mission) seqClass = Mission.cmdClass(it.command);

            var isSelected = (i === selectedIdx);
            var selClass = isSelected ? ' selected' : '';

            var icon = L.divIcon({
                className: 'wp-marker-wrapper',
                html: '<div class="wp-marker ' + seqClass + selClass + '">' + (i + 1) + '</div>',
                iconSize: [24, 24],
                iconAnchor: [12, 12],
            });

            var marker = L.marker(latlng, {
                icon: icon,
                draggable: planModeActive,
                zIndexOffset: isSelected ? 500 : 100,
            }).addTo(map);

            // Store index for event handlers
            marker._wpIdx = i;

            // Click to select
            marker.on('click', function (e) {
                if (e.originalEvent) e.originalEvent._wpMarkerClick = true;
                Mission.select(this._wpIdx);
            });

            // Drag to update position
            if (planModeActive) {
                marker.on('dragend', function (e) {
                    var ll = e.target.getLatLng();
                    Mission.updateItem(this._wpIdx, { lat: ll.lat, lon: ll.lng });
                });
            }

            waypointMarkers.push(marker);
        }

        // Draw mission path polyline (blue dashed)
        if (pathCoords.length >= 2) {
            missionPolyline = L.polyline(pathCoords, {
                color: '#448aff',
                weight: 2.5,
                opacity: 0.7,
                dashArray: '8,6',
                smoothFactor: 1,
            }).addTo(map);
        }
    }

    function clearMissionMarkers() {
        updateMissionMarkers([], -1);
    }

    function fitMission(items) {
        if (!map || !items || items.length === 0) return;
        var bounds = [];
        for (var i = 0; i < items.length; i++) {
            if (items[i].lat && items[i].lon) {
                bounds.push([items[i].lat, items[i].lon]);
            }
        }
        if (bounds.length > 0) {
            map.fitBounds(bounds, { padding: [40, 40], maxZoom: 18 });
        }
    }

    // --- Crosstrack geometry helper ---
    // Find the closest point on a polyline path to a given position
    function findClosestPointOnPath(lat, lon, pathLatLngs) {
        var minDist = Infinity;
        var closestPt = null;

        for (var i = 0; i < pathLatLngs.length - 1; i++) {
            var a = pathLatLngs[i];
            var b = pathLatLngs[i + 1];

            var aLat = a.lat !== undefined ? a.lat : a[0];
            var aLon = a.lng !== undefined ? a.lng : a[1];
            var bLat = b.lat !== undefined ? b.lat : b[0];
            var bLon = b.lng !== undefined ? b.lng : b[1];

            // Project point onto segment
            var dx = bLat - aLat;
            var dy = bLon - aLon;
            var segLenSq = dx * dx + dy * dy;

            if (segLenSq < 1e-14) continue;

            var t = ((lat - aLat) * dx + (lon - aLon) * dy) / segLenSq;
            t = Math.max(0, Math.min(1, t));

            var projLat = aLat + t * dx;
            var projLon = aLon + t * dy;

            var dLat = lat - projLat;
            var dLon = lon - projLon;
            var dist = dLat * dLat + dLon * dLon;

            if (dist < minDist) {
                minDist = dist;
                closestPt = [projLat, projLon];
            }
        }

        return closestPt;
    }

    // --- Multi-vehicle map display ---
    // Show secondary vehicle markers (non-active vehicles) with dimmed style
    function updateSecondaryVehicles() {
        if (!map) return;

        var activeId = meridian.activeVehicleId;
        var vehicles = meridian.vehicles;

        // Remove markers for vehicles that no longer exist
        for (var id in secondaryVehicleMarkers) {
            if (!vehicles[id] || parseInt(id) === activeId) {
                map.removeLayer(secondaryVehicleMarkers[id]);
                delete secondaryVehicleMarkers[id];
            }
        }

        // Add/update markers for non-active vehicles
        for (var sysid in vehicles) {
            var sid = parseInt(sysid);
            if (sid === activeId) continue;

            var sv = vehicles[sid];
            if (!sv || (sv.lat === 0 && sv.lon === 0)) continue;

            if (!secondaryVehicleMarkers[sid]) {
                var dimIcon = L.divIcon({
                    className: 'vehicle-icon secondary',
                    html: '<svg viewBox="0 0 36 36" xmlns="http://www.w3.org/2000/svg">' +
                          '<g fill="none" stroke="#94a3b8" stroke-width="1.5" stroke-linecap="round" opacity="0.6">' +
                          '<circle cx="18" cy="18" r="4" fill="rgba(148,163,184,0.15)"/>' +
                          '<polygon points="18,6 15,12 21,12" fill="#94a3b8" opacity="0.5"/>' +
                          '</g></svg>' +
                          '<div class="secondary-label">V' + sid + '</div>',
                    iconSize: [36, 36],
                });
                secondaryVehicleMarkers[sid] = L.marker([sv.lat, sv.lon], {
                    icon: dimIcon,
                    zIndexOffset: 500,
                    interactive: true,
                }).addTo(map);

                // Click to switch vehicle
                (function (targetId) {
                    secondaryVehicleMarkers[targetId].on('click', function () {
                        if (window.MultiVehicle) MultiVehicle.switchVehicle(targetId);
                    });
                })(sid);
            } else {
                secondaryVehicleMarkers[sid].setLatLng([sv.lat, sv.lon]);
            }

            // Update heading
            var el = secondaryVehicleMarkers[sid].getElement();
            if (el) {
                var svg = el.querySelector('svg');
                if (svg) svg.style.transform = 'rotate(' + sv.heading + 'deg)';
            }
        }
    }

    // --- T1-2: Geofence display in Fly view ---
    let geofencePolygon = null;

    function updateGeofence(points) {
        if (!map) return;
        if (geofencePolygon) {
            map.removeLayer(geofencePolygon);
            geofencePolygon = null;
        }
        if (!points || points.length < 3) return;
        var latlngs = points.map(function (p) { return [p.lat, p.lon]; });
        geofencePolygon = L.polygon(latlngs, {
            color: '#f44336',
            weight: 2,
            opacity: 0.6,
            fillColor: '#f44336',
            fillOpacity: 0.08,
            dashArray: '8,4',
        }).addTo(map);
    }

    // --- T1-5: ADSB traffic display ---
    let adsbMarkers = {};  // ICAO -> L.marker

    function updateAdsbTraffic() {
        if (!map || !meridian.settings.showAdsb) {
            // Hide all ADSB markers
            for (var icao in adsbMarkers) {
                map.removeLayer(adsbMarkers[icao]);
                delete adsbMarkers[icao];
            }
            return;
        }

        var now = Date.now();
        var v = meridian.v;
        var vehicleAlt = v ? v.relativeAlt : 0;

        for (var icao in meridian.adsb) {
            var contact = meridian.adsb[icao];

            // Expire after 60s
            if (now - contact.lastSeen > 60000) {
                if (adsbMarkers[icao]) {
                    map.removeLayer(adsbMarkers[icao]);
                    delete adsbMarkers[icao];
                }
                delete meridian.adsb[icao];
                continue;
            }

            if (contact.lat === 0 && contact.lon === 0) continue;

            // Color by relative altitude
            var altDiff = contact.alt - vehicleAlt;
            var color = '#94a3b8'; // neutral gray
            if (altDiff > 50) color = '#f59e0b';   // above = amber
            else if (altDiff < -50) color = '#22c55e'; // below = green
            else color = '#ef4444';                     // same level = red

            var label = contact.callsign || ('ICAO:' + icao);
            var altStr = contact.alt ? Math.round(contact.alt) + 'm' : '--';

            if (!adsbMarkers[icao]) {
                var icon = L.divIcon({
                    className: 'adsb-icon',
                    html: '<div class="adsb-marker" style="color:' + color + '">' +
                          '<svg viewBox="0 0 24 24" width="20" height="20" fill="' + color + '" style="transform:rotate(' + (contact.heading || 0) + 'deg)">' +
                          '<path d="M12 2L4 14h3v8h10v-8h3L12 2z"/></svg>' +
                          '<span class="adsb-label">' + label + ' ' + altStr + '</span></div>',
                    iconSize: [20, 20],
                    iconAnchor: [10, 10],
                });
                adsbMarkers[icao] = L.marker([contact.lat, contact.lon], {
                    icon: icon,
                    zIndexOffset: 200,
                    interactive: false,
                }).addTo(map);
            } else {
                adsbMarkers[icao].setLatLng([contact.lat, contact.lon]);
                var el = adsbMarkers[icao].getElement();
                if (el) {
                    var svg = el.querySelector('svg');
                    if (svg) svg.style.transform = 'rotate(' + (contact.heading || 0) + 'deg)';
                    var lbl = el.querySelector('.adsb-label');
                    if (lbl) lbl.textContent = label + ' ' + altStr;
                    var markerDiv = el.querySelector('.adsb-marker');
                    if (markerDiv) markerDiv.style.color = color;
                    if (svg) svg.setAttribute('fill', color);
                }
            }
        }
    }

    return {
        init, toggleFollow, centerOnVehicle, getMap,
        enablePlanMode, updateMissionMarkers, clearMissionMarkers, fitMission,
        updateSecondaryVehicles, updateGeofence, updateAdsbTraffic,
        setVehicleType
    };

})();
