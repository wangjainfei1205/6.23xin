import streamlit as st
import streamlit.components.v1 as components
import math
import json
import os

from route_planner import RoutePlanner, Point

NANJING_LAT = 32.234104
NANJING_LNG = 118.749421

x_pi = 3.14159265358979324 * 3000.0 / 180.0
pi = 3.1415926535897932384626
a = 6378245.0
ee = 0.00669342162296594323

def out_of_china(lng, lat):
    if lng < 72.004 or lng > 137.8347:
        return True
    if lat < 0.8293 or lat > 55.8271:
        return True
    return False

def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
    return ret

def wgs84_to_gcj02(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    return lng + dlng, lat + dlat

def gcj02_to_wgs84(lng, lat):
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lng(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    return lng * 2 - (lng + dlng), lat * 2 - (lat + dlat)

def render_route_planning_page():
    st.header("🗺️ 航线规划")
    
    if 'route_planner' not in st.session_state:
        st.session_state.route_planner = RoutePlanner()
        st.session_state.route_planner.load_route()
    
    planner = st.session_state.route_planner
    
    tab1, tab2, tab3 = st.tabs(["🛤️ 航点管理", "🚧 障碍物管理", "🔄 坐标转换"])
    
    with tab1:
        col1, col2 = st.columns([1, 2])
        
        with col1:
            st.subheader("📍 添加航点")
            
            wp_lat = st.number_input("纬度", min_value=-90.0, max_value=90.0, value=NANJING_LAT, step=0.0001, format="%.6f")
            wp_lng = st.number_input("经度", min_value=-180.0, max_value=180.0, value=NANJING_LNG, step=0.0001, format="%.6f")
            wp_alt = st.number_input("高度(m)", min_value=10, max_value=500, value=planner.uav_altitude, step=10)
            wp_speed = st.number_input("速度(m/s)", min_value=1, max_value=50, value=15, step=1)
            wp_name = st.text_input("航点名称", placeholder=f"WP{len(planner.waypoints) + 1}")
            
            col_add_btn1, col_add_btn2 = st.columns(2)
            with col_add_btn1:
                if st.button("➕ 添加航点", use_container_width=True):
                    name = wp_name if wp_name else f"WP{len(planner.waypoints) + 1}"
                    planner.add_waypoint(wp_lat, wp_lng, wp_alt, wp_speed, name)
                    planner.save_route()
                    st.success(f"✅ 航点 {name} 添加成功!")
            with col_add_btn2:
                if st.button("📍 从地图添加", use_container_width=True):
                    st.info("点击地图上的位置添加航点")
            
            st.markdown("---")
            st.subheader("✏️ 编辑航点")
            
            if planner.waypoints:
                selected_wp = st.selectbox(
                    "选择航点",
                    [f"{wp.name} ({wp.lat:.6f}, {wp.lng:.6f})" for wp in planner.waypoints],
                    index=0
                )
                wp_index = [wp.name for wp in planner.waypoints].index(selected_wp.split(' ')[0])
                
                col_edit_btn1, col_edit_btn2, col_edit_btn3 = st.columns(3)
                with col_edit_btn1:
                    if st.button("↑ 上移", use_container_width=True):
                        if wp_index > 0:
                            planner.waypoints[wp_index], planner.waypoints[wp_index-1] = planner.waypoints[wp_index-1], planner.waypoints[wp_index]
                            planner.save_route()
                            st.rerun()
                with col_edit_btn2:
                    if st.button("↓ 下移", use_container_width=True):
                        if wp_index < len(planner.waypoints) - 1:
                            planner.waypoints[wp_index], planner.waypoints[wp_index+1] = planner.waypoints[wp_index+1], planner.waypoints[wp_index]
                            planner.save_route()
                            st.rerun()
                with col_edit_btn3:
                    if st.button("🗑️ 删除", use_container_width=True):
                        planner.remove_waypoint(wp_index)
                        planner.save_route()
                        st.success("✅ 航点已删除")
                        st.rerun()
            
            st.markdown("---")
            st.subheader("📋 航点列表")
            
            if planner.waypoints:
                for i, wp in enumerate(planner.waypoints):
                    st.markdown(f"**{i+1}. {wp.name}**")
                    st.markdown(f"   - 坐标: {wp.lat:.6f}, {wp.lng:.6f}")
                    st.markdown(f"   - 高度: {wp.altitude}m")
                    st.markdown(f"   - 速度: {wp.speed}m/s")
                    if i < len(planner.waypoints) - 1:
                        p1 = Point(wp.lat, wp.lng)
                        p2 = Point(planner.waypoints[i+1].lat, planner.waypoints[i+1].lng)
                        st.markdown(f"   - 到下一站: {p1.distance_to(p2):.2f} km")
                    st.markdown("---")
            else:
                st.info("💡 暂无航点，请添加航点开始规划航线")
            
            st.markdown("---")
            st.subheader("⚙️ 航线操作")
            
            col_route_btn1, col_route_btn2, col_route_btn3 = st.columns(3)
            with col_route_btn1:
                if st.button("🔄 清空航线", use_container_width=True):
                    planner.clear_all()
                    planner.save_route()
                    st.success("✅ 航线已清空")
            with col_route_btn2:
                if st.button("📥 加载航线", use_container_width=True):
                    planner.load_route()
                    st.success("✅ 航线已加载")
            with col_route_btn3:
                if st.button("📤 保存航线", use_container_width=True):
                    planner.save_route()
                    st.success("✅ 航线已保存")
            
            st.markdown("---")
            st.subheader("📊 航线统计")
            
            direct_distance = planner.calculate_total_distance()
            flight_time = planner.estimate_flight_time()
            
            col_stat1, col_stat2, col_stat3 = st.columns(3)
            with col_stat1:
                st.metric("航点数量", len(planner.waypoints))
            with col_stat2:
                st.metric("总距离", f"{direct_distance:.2f} km")
            with col_stat3:
                st.metric("预计时间", f"{flight_time/60:.1f} 分钟")
        
        with col2:
            st.subheader("🗺️ 航线预览")
            
            map_type = st.radio("地图类型", ["普通地图", "卫星地图"], horizontal=True)
            route_mode = st.radio("航线模式", ["直接航线", "避障航线"], horizontal=True)
            
            detour_mode = st.selectbox("绕飞模式", ["最优路径", "向左绕飞", "向右绕飞"], key="detour_mode")
            
            tile_url = "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" if map_type == "普通地图" else "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
            tile_attribution = "© OpenStreetMap contributors" if map_type == "普通地图" else "© Esri"
            
            if planner.waypoints:
                if route_mode == "避障航线":
                    route_coords, detour_info = planner.plan_route_with_obstacle_avoidance(detour_mode=detour_mode.lower())
                else:
                    route_coords = planner.get_direct_route()
                    detour_info = ""
                
                detour_distance = planner.calculate_total_distance(route_coords)
                
                center_lat = sum(wp.lat for wp in planner.waypoints) / len(planner.waypoints)
                center_lng = sum(wp.lng for wp in planner.waypoints) / len(planner.waypoints)
                
                obstacles_json = json.dumps([{'name': obs.name, 'coords': obs.coords, 'height': obs.height} for obs in planner.obstacles])
                
                map_html = """
                <!DOCTYPE html>
                <html>
                <head>
                    <meta charset="utf-8">
                    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
                    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
                    <style>
                        html, body, #map { width: 100%; height: 100%; margin: 0; padding: 0; }
                        .info { position: absolute; top: 10px; right: 10px; z-index: 1000; background: rgba(255,255,255,0.9); padding: 10px; border-radius: 5px; font-size: 12px; }
                        .legend { position: absolute; bottom: 10px; left: 10px; z-index: 1000; background: rgba(255,255,255,0.9); padding: 8px; border-radius: 5px; font-size: 11px; }
                    </style>
                </head>
                <body>
                    <div id="map"></div>
                    <div class="info">
                        <div><strong>📍 航线信息</strong></div>
                        <div>航点: """ + str(len(planner.waypoints)) + """ 个</div>
                        <div>距离: """ + f"{detour_distance:.2f}" + """ km</div>
                        <div>模式: """ + route_mode + """</div>
                        """ + ("<div style='color: #FF6B6B;'>" + detour_info + "</div>" if detour_info else "") + """
                    </div>
                    <div class="legend">
                        <div><b>图例</b></div>
                        <div>🟢 起点</div>
                        <div>🔴 终点</div>
                        <div>🔵 航线</div>
                        <div>🟠 障碍物</div>
                        <div>⚪ 航点</div>
                    </div>
                    <script>
                        var map = L.map('map').setView([""" + f"{center_lat}, {center_lng}" + """], 15);
                        
                        L.tileLayer('""" + tile_url + """', {
                            attribution: '""" + tile_attribution + """',
                            maxZoom: 19
                        }).addTo(map);

                        L.control.scale({imperial: false}).addTo(map);

                        var routeCoords = """ + json.dumps(route_coords) + """;
                        L.polyline(routeCoords, {color: '#3366FF', weight: 4, opacity: 0.8}).addTo(map);

                        """ + ("""
                        var obstacles = """ + obstacles_json + """;
                        obstacles.forEach(function(obs, index) {
                            L.polygon(obs.coords, {
                                color: '#FFA500',
                                fillColor: '#FFA500',
                                fillOpacity: 0.3,
                                weight: 2
                            }).addTo(map).bindPopup('障碍物: ' + obs.name + '<br>高度: ' + obs.height + 'm');
                        });
                        """ if planner.obstacles else "") + """

                        routeCoords.forEach(function(coord, index) {
                            var isStart = index === 0;
                            var isEnd = index === routeCoords.length - 1;
                            
                            if (isStart) {
                                L.marker(coord, {icon: L.icon({
                                    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
                                    iconSize: [25, 41],
                                    iconAnchor: [12, 41]
                                })}).addTo(map).bindPopup('起点');
                            } else if (isEnd) {
                                L.marker(coord, {icon: L.icon({
                                    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
                                    iconSize: [25, 41],
                                    iconAnchor: [12, 41]
                                })}).addTo(map).bindPopup('终点');
                            } else {
                                L.circleMarker(coord, {radius: 5, color: '#FFFFFF', fillColor: '#3366FF', fillOpacity: 1, weight: 2}).addTo(map);
                            }
                        });

                        var bounds = L.latLngBounds(routeCoords);
                        map.fitBounds(bounds, {padding: [30, 30]});
                    </script>
                </body>
                </html>
                """
                components.html(map_html, height=600, scrolling=True)
                
                if detour_info:
                    st.info(f"🗺️ {detour_info}")
            else:
                map_html = f"""
                <!DOCTYPE html>
                <html>
                <head>
                    <meta charset="utf-8">
                    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
                    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
                    <style>html, body, #map {{ width: 100%; height: 100%; margin: 0; padding: 0; }}</style>
                </head>
                <body>
                    <div id="map"></div>
                    <script>
                        var map = L.map('map').setView([{NANJING_LAT}, {NANJING_LNG}], 15);
                        L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                            attribution: '© OpenStreetMap', maxZoom: 19
                        }}).addTo(map);
                        L.control.scale({{imperial: false}}).addTo(map);
                    </script>
                </body>
                </html>
                """
                components.html(map_html, height=600, scrolling=True)
                st.info("💡 添加航点后将在地图上显示航线")
    
    with tab2:
        col1, col2 = st.columns([1, 2])
        
        with col1:
            st.subheader("🚧 障碍物管理")
            
            st.markdown("**无人机飞行高度**")
            uav_alt = st.slider("设置无人机飞行高度(m)", min_value=10, max_value=500, value=int(planner.uav_altitude), step=10)
            planner.set_uav_altitude(uav_alt)
            planner.save_route()
            
            st.markdown("---")
            st.markdown("**添加障碍物**")
            obs_name = st.text_input("障碍物名称", placeholder="障碍物1")
            obs_height = st.number_input("障碍物高度(m)", min_value=1, max_value=500, value=50, step=10)
            
            col_obs_btn1, col_obs_btn2 = st.columns(2)
            with col_obs_btn1:
                if st.button("➕ 添加障碍物", key="add_obs", use_container_width=True):
                    if 'drawing_obstacle_coords' in st.session_state and len(st.session_state.drawing_obstacle_coords) >= 3:
                        name = obs_name if obs_name else f"障碍物{len(planner.obstacles) + 1}"
                        planner.add_obstacle(name, st.session_state.drawing_obstacle_coords, obs_height)
                        planner.save_route()
                        st.success(f"✅ 障碍物 {name} 添加成功!")
                        del st.session_state.drawing_obstacle_coords
                    else:
                        st.warning("⚠️ 请先在地图上绘制障碍物（至少3个顶点）")
            
            with col_obs_btn2:
                if st.button("🗑️ 清除全部", use_container_width=True):
                    planner.obstacles = []
                    planner.save_route()
                    st.success("✅ 所有障碍物已清除")
            
            st.markdown("---")
            st.markdown("**已标记障碍物**")
            
            if planner.obstacles:
                for i, obs in enumerate(planner.obstacles):
                    st.markdown(f"**{obs.name}**")
                    st.markdown(f"   - 顶点数: {len(obs.coords)}")
                    st.markdown(f"   - 高度: {obs.height}m")
                    st.markdown(f"   - 需要绕飞: {'是' if obs.height >= planner.uav_altitude else '否'}")
                    
                    col_obs_info, col_obs_del = st.columns([4, 1])
                    with col_obs_info:
                        new_height = st.number_input(f"高度调整(m)", min_value=1, max_value=500, value=obs.height, step=10, key=f"obs_height_{i}")
                        if new_height != obs.height:
                            planner.update_obstacle(i, height=new_height)
                            planner.save_route()
                    with col_obs_del:
                        if st.button("✕", key=f"del_obs_{i}"):
                            planner.remove_obstacle(i)
                            planner.save_route()
                            st.rerun()
                    st.markdown("---")
            else:
                st.info("💡 暂无障碍物")
            
            st.markdown("---")
            st.markdown("**安全距离设置**")
            safety_dist = st.slider("避障安全距离(m)", min_value=10, max_value=200, value=int(planner.safety_distance * 111000), step=10)
            planner.safety_distance = safety_dist / 111000
            planner.save_route()
        
        with col2:
            st.subheader("🗺️ 障碍物绘制")
            
            obstacles_json = json.dumps([{'name': obs.name, 'coords': obs.coords, 'height': obs.height} for obs in planner.obstacles])
            
            map_html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8">
                <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
                <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
                <style>
                    html, body, #map {{ width: 100%; height: 100%; margin: 0; padding: 0; }}
                    .drawing-hint {{ position: absolute; top: 10px; left: 10px; z-index: 1000; background: rgba(255,100,100,0.9); padding: 8px 12px; border-radius: 5px; font-size: 12px; color: white; }}
                    .info {{ position: absolute; top: 10px; right: 10px; z-index: 1000; background: rgba(255,255,255,0.9); padding: 10px; border-radius: 5px; font-size: 12px; }}
                </style>
            </head>
            <body>
                <div id="map"></div>
                <div class="drawing-hint">🖱️ 点击添加顶点，双击完成多边形</div>
                <div class="info">
                    <div><strong>✏️ 绘制提示</strong></div>
                    <div>无人机高度: <span style="color: blue;">{planner.uav_altitude}m</span></div>
                    <div>障碍物: {len(planner.obstacles)} 个</div>
                </div>
                <script>
                    var map = L.map('map').setView([{NANJING_LAT}, {NANJING_LNG}], 15);
                    L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                        attribution: '© OpenStreetMap', maxZoom: 19
                    }}).addTo(map);
                    L.control.scale({{imperial: false}}).addTo(map);

                    var obstacles = {obstacles_json};
                    obstacles.forEach(function(obs, index) {{
                        var color = obs.height >= {planner.uav_altitude} ? '#FFA500' : '#90EE90';
                        L.polygon(obs.coords, {{
                            color: color,
                            fillColor: color,
                            fillOpacity: 0.3,
                            weight: 2
                        }}).addTo(map).bindPopup('障碍物: ' + obs.name + '<br>高度: ' + obs.height + 'm' + '<br>' + (obs.height >= {planner.uav_altitude} ? '<span style="color:red;">需要绕飞</span>' : '<span style="color:green;">无需绕飞</span>'));
                    }});

                    var routeCoords = {json.dumps(planner.get_direct_route())};
                    if (routeCoords.length > 1) {{
                        L.polyline(routeCoords, {{color: '#3366FF', weight: 3, dashArray: '10, 10'}}).addTo(map);
                    }}

                    var drawingPoints = [];
                    var drawingPolyline = null;

                    map.on('click', function(e) {{
                        drawingPoints.push([e.latlng.lat, e.latlng.lng]);
                        if (drawingPolyline) {{
                            map.removeLayer(drawingPolyline);
                        }}
                        if (drawingPoints.length > 1) {{
                            drawingPolyline = L.polyline(drawingPoints, {{
                                color: '#FF6B6B',
                                weight: 2,
                                dashArray: '5, 5'
                            }}).addTo(map);
                        }}
                        L.circleMarker(e.latlng, {{
                            radius: 5,
                            color: '#FF6B6B',
                            fillColor: '#FF6B6B',
                            fillOpacity: 1
                        }}).addTo(map);
                    }});

                    map.on('dblclick', function(e) {{
                        if (drawingPoints.length < 3) return;
                        drawingPoints.push(drawingPoints[0]);
                        var newObstacle = {{
                            name: '新障碍物',
                            coords: drawingPoints.slice(0, -1),
                            height: 50
                        }};
                        L.polygon(newObstacle.coords, {{
                            color: '#FFA500',
                            fillColor: '#FFA500',
                            fillOpacity: 0.3,
                            weight: 2
                        }}).addTo(map).bindPopup('新障碍物');
                        
                        fetch('/save_obstacle', {{
                            method: 'POST',
                            headers: {{'Content-Type': 'application/json'}},
                            body: JSON.stringify(newObstacle)
                        }});
                        
                        drawingPoints = [];
                        if (drawingPolyline) {{
                            map.removeLayer(drawingPolyline);
                            drawingPolyline = null;
                        }}
                    }});
                </script>
            </body>
            </html>
            """
            components.html(map_html, height=500, scrolling=True)
    
    with tab3:
        st.subheader("🔄 坐标转换")
        
        tab_wgs_gcj, tab_gcj_wgs, tab_batch = st.tabs(["WGS→GCJ", "GCJ→WGS", "批量转换"])
        
        with tab_wgs_gcj:
            st.markdown("**WGS-84 转 GCJ-02（高德/百度）**")
            wgs_lat = st.number_input("WGS纬度", value=NANJING_LAT, step=0.0001, format="%.6f")
            wgs_lng = st.number_input("WGS经度", value=NANJING_LNG, step=0.0001, format="%.6f")
            if st.button("转换"):
                gcj_lng, gcj_lat = wgs84_to_gcj02(wgs_lng, wgs_lat)
                st.success(f"GCJ-02 坐标:\n纬度: {gcj_lat:.6f}\n经度: {gcj_lng:.6f}")
        
        with tab_gcj_wgs:
            st.markdown("**GCJ-02 转 WGS-84**")
            gcj_lat = st.number_input("GCJ纬度", value=NANJING_LAT, step=0.0001, format="%.6f")
            gcj_lng = st.number_input("GCJ经度", value=NANJING_LNG, step=0.0001, format="%.6f")
            if st.button("逆向转换"):
                wgs_lng, wgs_lat = gcj02_to_wgs84(gcj_lng, gcj_lat)
                st.success(f"WGS-84 坐标:\n纬度: {wgs_lat:.6f}\n经度: {wgs_lng:.6f}")
        
        with tab_batch:
            st.markdown("**批量转换**")
            batch_input = st.text_area("输入坐标（每行一个，格式：纬度,经度）",
                                     placeholder="32.234104,118.749421\n32.235000,118.750000")
            from_sys = st.selectbox("源坐标系", ["WGS-84", "GCJ-02"])
            to_sys = st.selectbox("目标坐标系", ["WGS-84", "GCJ-02"])
            
            if st.button("批量转换"):
                results = []
                for line in batch_input.strip().split('\n'):
                    line = line.strip()
                    if line:
                        try:
                            lat, lng = map(float, line.split(','))
                            if from_sys == "WGS-84" and to_sys == "GCJ-02":
                                result_lng, result_lat = wgs84_to_gcj02(lng, lat)
                            else:
                                result_lng, result_lat = gcj02_to_wgs84(lng, lat)
                            results.append(f"{result_lat:.6f},{result_lng:.6f}")
                        except ValueError:
                            results.append(f"错误: {line}")
                st.text_area("转换结果", '\n'.join(results), height=200)