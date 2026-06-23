import streamlit as st
import folium
from streamlit_folium import st_folium
import math
import random
import json
import os
import pandas as pd
import plotly.graph_objects as go
from datetime import datetime, timedelta
from route_planner import RoutePlanner, Point
from coordinate_converter import wgs84_to_gcj02, gcj02_to_wgs84, wgs84_to_bd09, bd09_to_wgs84

st.set_page_config(
    page_title="无人机智能化应用系统",
    page_icon="🚁",
    layout="wide",
    initial_sidebar_state="expanded"
)

NANJING_LAT = 32.234104
NANJING_LNG = 118.749421

def create_satellite_map(center_lat, center_lng, zoom=15):
    m = folium.Map(location=[center_lat, center_lng], zoom_start=zoom, tiles=None)
    folium.TileLayer(
        tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        attr='© Esri',
        name='卫星地图'
    ).add_to(m)
    return m

def save_data(data, filename='route.json'):
    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def load_data(filename='route.json'):
    if os.path.exists(filename):
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                return json.load(f)
        except:
            pass
    return {'waypoints': [], 'obstacles': [], 'safety_radius': 30, 'uav_altitude': 100}

def init_session_state():
    if 'data' not in st.session_state:
        st.session_state.data = load_data()
    if 'heartbeat_data' not in st.session_state:
        st.session_state.heartbeat_data = []
    if 'is_simulating' not in st.session_state:
        st.session_state.is_simulating = False
    if 'battery_level' not in st.session_state:
        st.session_state.battery_level = 100.0
    if 'current_position' not in st.session_state:
        st.session_state.current_position = [NANJING_LAT, NANJING_LNG]
    if 'flight_path' not in st.session_state:
        st.session_state.flight_path = []
    if 'temp_obstacle' not in st.session_state:
        st.session_state.temp_obstacle = []
    if 'map_click_mode' not in st.session_state:
        st.session_state.map_click_mode = 'waypoint'
    if 'current_waypoint' not in st.session_state:
        st.session_state.current_waypoint = 0
    if 'start_time' not in st.session_state:
        st.session_state.start_time = None
    if 'total_distance' not in st.session_state:
        st.session_state.total_distance = 0.0
    if 'completed_distance' not in st.session_state:
        st.session_state.completed_distance = 0.0
    if 'flight_speed' not in st.session_state:
        st.session_state.flight_speed = 15.0
    if 'planned_route' not in st.session_state:
        st.session_state.planned_route = []
    if 'heading' not in st.session_state:
        st.session_state.heading = 0.0
    if 'route_planner' not in st.session_state:
        st.session_state.route_planner = RoutePlanner()
        st.session_state.route_planner.load_route()
    # MAVLink通信链路数据
    if 'mavlink_messages' not in st.session_state:
        st.session_state.mavlink_messages = []
    if 'link_status' not in st.session_state:
        st.session_state.link_status = {
            'gcs_obc': {'connected': False, 'rssi': 0, 'latency': 0},
            'obc_fcu': {'connected': False, 'rssi': 0, 'latency': 0},
            'gcs_fcu': {'connected': False, 'rssi': 0, 'latency': 0}
        }
    if 'mavlink_msg_count' not in st.session_state:
        st.session_state.mavlink_msg_count = 0

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def point_to_polygon_distance(point, polygon):
    min_dist = float('inf')
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i+1)%n]
        dist = point_to_segment_distance(point[0], point[1], x1, y1, x2, y2)
        if dist < min_dist:
            min_dist = dist
    return min_dist

def point_to_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.hypot(px - x1, py - y1)
    
    t = ((px - x1)*dx + (py - y1)*dy) / (dx*dx + dy*dy)
    t = max(0, min(1, t))
    
    nx = x1 + t*dx
    ny = y1 + t*dy
    
    return math.hypot(px - nx, py - ny)

def check_route_conflict(waypoints, obstacles, safety_radius, uav_altitude):
    conflicts = []
    needs_flyaround = []
    
    for i in range(len(waypoints)-1):
        wp1 = waypoints[i]
        wp2 = waypoints[i+1]
        
        for j, obs in enumerate(obstacles):
            obs_height = obs.get('height', 50)
            if obs_height >= uav_altitude:
                min_dist = point_to_polygon_distance((wp1['lat'], wp1['lng']), obs['coords'])
                if min_dist < safety_radius / 111000:
                    conflicts.append(f"航段 {wp1['name']}-{wp2['name']} 与障碍物 {obs['name']} 距离过近")
                    needs_flyaround.append((i, j))
                
                min_dist = point_to_polygon_distance((wp2['lat'], wp2['lng']), obs['coords'])
                if min_dist < safety_radius / 111000:
                    conflicts.append(f"航段 {wp1['name']}-{wp2['name']} 与障碍物 {obs['name']} 距离过近")
                    needs_flyaround.append((i, j))
    
    return conflicts, needs_flyaround

def calculate_flyaround(wp1, wp2, obstacle, mode='optimal', safety_radius=30):
    obs_coords = obstacle['coords']
    obs_center_lat = sum(c[0] for c in obs_coords) / len(obs_coords)
    obs_center_lng = sum(c[1] for c in obs_coords) / len(obs_coords)
    
    dx = wp2['lng'] - wp1['lng']
    dy = wp2['lat'] - wp1['lat']
    
    perp_x = -dy
    perp_y = dx
    perp_len = math.sqrt(perp_x**2 + perp_y**2)
    if perp_len > 0:
        perp_x /= perp_len
        perp_y /= perp_len
    
    safety_offset = safety_radius / 111000
    
    if mode == 'left':
        flyaround_points = [
            (wp1['lat'], wp1['lng']),
            (wp1['lat'] + perp_y * safety_offset * 2, wp1['lng'] + perp_x * safety_offset * 2),
            (obs_center_lat + perp_y * safety_offset * 3, obs_center_lng + perp_x * safety_offset * 3),
            (wp2['lat'] + perp_y * safety_offset * 2, wp2['lng'] + perp_x * safety_offset * 2),
            (wp2['lat'], wp2['lng'])
        ]
    elif mode == 'right':
        flyaround_points = [
            (wp1['lat'], wp1['lng']),
            (wp1['lat'] - perp_y * safety_offset * 2, wp1['lng'] - perp_x * safety_offset * 2),
            (obs_center_lat - perp_y * safety_offset * 3, obs_center_lng - perp_x * safety_offset * 3),
            (wp2['lat'] - perp_y * safety_offset * 2, wp2['lng'] - perp_x * safety_offset * 2),
            (wp2['lat'], wp2['lng'])
        ]
    else:
        angle1 = math.atan2(obs_center_lat - wp1['lat'], obs_center_lng - wp1['lng'])
        angle2 = math.atan2(wp2['lat'] - obs_center_lat, wp2['lng'] - obs_center_lng)
        
        radius = safety_offset * 2
        
        mid_angle = (angle1 + angle2) / 2
        arc_center_lat = obs_center_lat - math.sin(mid_angle) * radius * 1.5
        arc_center_lng = obs_center_lng + math.cos(mid_angle) * radius * 1.5
        
        num_points = 10
        flyaround_points = [(wp1['lat'], wp1['lng'])]
        
        for i in range(1, num_points):
            t = i / num_points
            angle = angle1 + (angle2 - angle1) * t
            px = arc_center_lat + math.sin(angle) * radius * 1.5
            py = arc_center_lng - math.cos(angle) * radius * 1.5
            flyaround_points.append((px, py))
        
        flyaround_points.append((wp2['lat'], wp2['lng']))
    
    return flyaround_points

def generate_route_with_flyaround(waypoints, obstacles, safety_radius, uav_altitude, mode='optimal'):
    planned_route = []
    
    for i in range(len(waypoints)-1):
        wp1 = waypoints[i]
        wp2 = waypoints[i+1]
        
        need_flyaround = False
        affected_obs = None
        
        for obs in obstacles:
            obs_height = obs.get('height', 50)
            if obs_height >= uav_altitude:
                min_dist = point_to_polygon_distance((wp1['lat'], wp1['lng']), obs['coords'])
                if min_dist < safety_radius / 111000:
                    need_flyaround = True
                    affected_obs = obs
                    break
        
        if need_flyaround and affected_obs:
            flyaround_points = calculate_flyaround(wp1, wp2, affected_obs, mode, safety_radius)
            planned_route.extend(flyaround_points[:-1])
        else:
            planned_route.append((wp1['lat'], wp1['lng']))
    
    if waypoints:
        planned_route.append((waypoints[-1]['lat'], waypoints[-1]['lng']))
    
    return planned_route

def render_home_page():
    st.markdown('<h1 style="text-align: center; color: #00D4AA; font-size: 2.5rem; font-weight: bold;">🚁 无人机智能化应用系统</h1>', unsafe_allow_html=True)
    st.markdown("---")
    
    col1, col2 = st.columns(2)
    with col1:
        st.markdown("""
        ### 🗺️ 航线规划
        - 卫星地图展示
        - 航点添加（地图点击）
        - 障碍物多边形圈选（含高度设置）
        - 安全半径设置与检测
        - 绕飞路径规划（左/右/最优）
        """)
    with col2:
        st.markdown("""
        ### 🚁 飞行监控
        - 按航点飞行模拟
        - 实时状态监测
        - 电量模拟与预警
        - 飞行数据可视化
        """)
    
    col_f1, col_f2, col_f3, col_f4 = st.columns(4)
    with col_f1:
        st.metric("航点数量", len(st.session_state.data['waypoints']))
    with col_f2:
        st.metric("障碍物数量", len(st.session_state.data['obstacles']))
    with col_f3:
        st.metric("安全半径", f"{st.session_state.data.get('safety_radius', 30)}m")
    with col_f4:
        st.metric("飞行高度", f"{st.session_state.data.get('uav_altitude', 100)}m")

def render_route_planning_page():
    st.header("🗺️ 航线规划")
    data = st.session_state.data
    planner = st.session_state.route_planner

    # 控制面板区域 - 使用expander折叠
    with st.expander("🎮 控制面板（点击展开/折叠）", expanded=True):
        col_ctrl1, col_ctrl2, col_ctrl3 = st.columns(3)
        
        with col_ctrl1:
            st.info(f"🎯 当前模式：{'📍 航点模式' if st.session_state.map_click_mode == 'waypoint' else '🚧 障碍物模式'}")
            
            mode1, mode2 = st.columns(2)
            with mode1:
                if st.button("📍 航点模式", type="primary" if st.session_state.map_click_mode == "waypoint" else "secondary", use_container_width=True):
                    st.session_state.map_click_mode = "waypoint"
                    st.rerun()
            with mode2:
                if st.button("🚧 障碍物模式", type="primary" if st.session_state.map_click_mode == "obstacle" else "secondary", use_container_width=True):
                    st.session_state.map_click_mode = "obstacle"
                    st.rerun()
        
        with col_ctrl2:
            uav_altitude = st.slider("✈️ 飞行高度 (米)", min_value=20, max_value=500, value=data.get('uav_altitude', 100), step=10)
            data['uav_altitude'] = uav_altitude
            planner.set_uav_altitude(uav_altitude)
            
            safety_radius = st.slider("🛡️ 安全半径 (米)", min_value=10, max_value=100, value=data.get('safety_radius', 30), step=5)
            data['safety_radius'] = safety_radius
            planner.safety_distance = safety_radius / 111000
            
            save_data(data)
            planner.save_route()
        
        with col_ctrl3:
            st.subheader("🔄 绕飞路径规划")
            if len(data['waypoints']) >= 2:
                col_f1, col_f2, col_f3 = st.columns(3)
                with col_f1:
                    if st.button("⬅️ 向左绕飞", use_container_width=True, type="primary"):
                        route = generate_route_with_flyaround(data['waypoints'], data['obstacles'], safety_radius, uav_altitude, 'left')
                        st.session_state.planned_route = route
                        st.success("✅ 向左绕飞航线计算完成！")
                        st.rerun()
                with col_f2:
                    if st.button("🎯 最优路径", use_container_width=True, type="primary"):
                        route = generate_route_with_flyaround(data['waypoints'], data['obstacles'], safety_radius, uav_altitude, 'optimal')
                        st.session_state.planned_route = route
                        st.success("✅ 最优绕飞航线计算完成！")
                        st.rerun()
                with col_f3:
                    if st.button("➡️ 向右绕飞", use_container_width=True, type="primary"):
                        route = generate_route_with_flyaround(data['waypoints'], data['obstacles'], safety_radius, uav_altitude, 'right')
                        st.session_state.planned_route = route
                        st.success("✅ 向右绕飞航线计算完成！")
                        st.rerun()
                
                if st.session_state.planned_route:
                    st.info(f"📊 当前规划了 {len(st.session_state.planned_route)} 个航点的绕飞路径")
            else:
                st.warning("⚠️ 请先添加至少2个航点")

    # 地图区域 - 全宽显示
    st.subheader("🗺️ 航线地图")
    
    center_lat, center_lng = NANJING_LAT, NANJING_LNG
    all_points = []
    for wp in data['waypoints']:
        all_points.append((wp['lat'], wp['lng']))
    for obs in data['obstacles']:
        for coord in obs['coords']:
            all_points.append(coord)
    if all_points:
        center_lat = sum(p[0] for p in all_points) / len(all_points)
        center_lng = sum(p[1] for p in all_points) / len(all_points)

    m = create_satellite_map(center_lat, center_lng)

    # 绘制规划航线（青色实线）
    if st.session_state.planned_route:
        folium.PolyLine(st.session_state.planned_route, color='#00D4AA', weight=4, opacity=0.9, popup='规划航线').add_to(m)
    # 绘制直接航线（橙色虚线）
    elif len(data['waypoints']) >= 2:
        route_coords = [(wp['lat'], wp['lng']) for wp in data['waypoints']]
        folium.PolyLine(route_coords, color='#FFA500', weight=3, dash_array='10, 5', opacity=0.8).add_to(m)

    # 航点标记
    if data['waypoints']:
        for i, wp in enumerate(data['waypoints']):
            if i == 0:
                folium.Marker(
                    [wp['lat'], wp['lng']], 
                    popup=f"🟢 起点: {wp['name']}",
                    icon=folium.Icon(color='green', icon='play', prefix='glyphicon')
                ).add_to(m)
            elif i == len(data['waypoints']) - 1:
                folium.Marker(
                    [wp['lat'], wp['lng']], 
                    popup=f"🔴 终点: {wp['name']}",
                    icon=folium.Icon(color='red', icon='stop', prefix='glyphicon')
                ).add_to(m)
            else:
                folium.CircleMarker(
                    [wp['lat'], wp['lng']], 
                    radius=6, 
                    color='#FFA500', 
                    fillColor='#FFA500', 
                    fillOpacity=1,
                    weight=2,
                    popup=f"航点 {i+1}: {wp['name']}"
                ).add_to(m)

    # 障碍物
    for obs in data['obstacles']:
        obs_height = obs.get('height', 50)
        if obs_height >= uav_altitude:
            color = '#FF6B6B'
            fill_color = '#FF6B6B'
            status_text = '<span style="color:red;">⚠️ 需要绕飞</span>'
        else:
            color = '#4CAF50'
            fill_color = '#4CAF50'
            status_text = '<span style="color:green;">✅ 无需绕飞</span>'
        
        folium.Polygon(
            obs['coords'], 
            color=color, 
            fill=True, 
            fillColor=fill_color, 
            fillOpacity=0.3, 
            weight=2,
            popup=folium.Popup(
                f"<b>{obs['name']}</b><br>"
                f"高度: {obs_height}m<br>"
                f"无人机高度: {uav_altitude}m<br>"
                f"{status_text}",
                max_width=200
            )
        ).add_to(m)
        
        obs_center_lat = sum(c[0] for c in obs['coords']) / len(obs['coords'])
        obs_center_lng = sum(c[1] for c in obs['coords']) / len(obs['coords'])
        folium.Circle(
            [obs_center_lat, obs_center_lng], 
            radius=safety_radius, 
            color='orange', 
            fill=False, 
            weight=2, 
            dash_array='5,5',
            popup=f"安全半径: {safety_radius}m"
        ).add_to(m)

    # 临时障碍物绘制
    if len(st.session_state.temp_obstacle) >= 1:
        for coord in st.session_state.temp_obstacle:
            folium.CircleMarker(
                coord, 
                radius=6, 
                color='#FF6B6B', 
                fillColor='#FF6B6B', 
                fillOpacity=1,
                weight=2
            ).add_to(m)
    if len(st.session_state.temp_obstacle) >= 2:
        folium.PolyLine(
            st.session_state.temp_obstacle, 
            color='#FF6B6B', 
            weight=2, 
            dash_array='5,5'
        ).add_to(m)
    if len(st.session_state.temp_obstacle) >= 3:
        folium.Polygon(
            st.session_state.temp_obstacle, 
            color='#FFA500', 
            fill=True, 
            fillColor='#FFA500', 
            fillOpacity=0.3, 
            weight=2
        ).add_to(m)

    map_output = st_folium(m, height=700, key="main_map", use_container_width=True)

    if map_output and map_output.get('last_clicked'):
        lat = map_output['last_clicked'].get('lat')
        lng = map_output['last_clicked'].get('lng')
        if lat and lng:
            if st.session_state.map_click_mode == 'waypoint':
                new_wp_name = f"航点{len(data['waypoints'])+1}"
                data['waypoints'].append({'lat': lat, 'lng': lng, 'name': new_wp_name})
                planner.add_waypoint(lat, lng, uav_altitude, 15.0, new_wp_name)
                st.session_state.planned_route = []
                save_data(data)
                planner.save_route()
                st.success(f"✅ {new_wp_name} 已添加到地图点击位置！")
                st.rerun()
            else:
                st.session_state.temp_obstacle.append((lat, lng))
                st.success(f"✅ 已添加第 {len(st.session_state.temp_obstacle)} 个顶点！")
                st.rerun()

    # 航点和障碍物管理 - 放在地图下方
    col_mgmt1, col_mgmt2 = st.columns(2)
    
    with col_mgmt1:
        with st.expander("📍 航点管理"):
            wp_lat = st.number_input("纬度", value=NANJING_LAT, step=0.0001, format="%.6f", key="wp_lat")
            wp_lng = st.number_input("经度", value=NANJING_LNG, step=0.0001, format="%.6f", key="wp_lng")
            wp_name = st.text_input("名称", placeholder=f"航点{len(data['waypoints'])+1}", key="wp_name")

            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("➕ 添加航点", use_container_width=True):
                    name = wp_name if wp_name else f"航点{len(data['waypoints'])+1}"
                    data['waypoints'].append({'lat': wp_lat, 'lng': wp_lng, 'name': name})
                    planner.add_waypoint(wp_lat, wp_lng, uav_altitude, 15.0, name)
                    save_data(data)
                    planner.save_route()
                    st.success(f"✅ {name} 添加成功！")

            with col_btn2:
                if st.button("🗑️ 清空航点", use_container_width=True):
                    data['waypoints'] = []
                    st.session_state.planned_route = []
                    planner.waypoints = []
                    save_data(data)
                    planner.save_route()
                    st.success("✅ 已清空")

            if data['waypoints']:
                st.subheader("航点列表")
                for i, wp in enumerate(data['waypoints']):
                    col_wp1, col_wp2 = st.columns([4, 1])
                    with col_wp1:
                        status = "🟢 起点" if i == 0 else ("🔴 终点" if i == len(data['waypoints'])-1 else "")
                        st.write(f"{i+1}. **{wp['name']}** {status}: ({wp['lat']:.6f}, {wp['lng']:.6f})")
                    with col_wp2:
                        if st.button("🗑️", key=f"del_wp_{i}"):
                            del data['waypoints'][i]
                            st.session_state.planned_route = []
                            save_data(data)
                            st.rerun()

    with col_mgmt2:
        with st.expander("🚧 障碍物管理"):
            obs_name = st.text_input("障碍物名称", placeholder="障碍物1", key="obs_name")
            obs_height = st.number_input("障碍物高度 (米)", min_value=1, max_value=500, value=50, step=10, key="obs_height")

            st.markdown(f"**已添加 {len(st.session_state.temp_obstacle)} 个顶点** (需要至少3个)")

            col_obs1, col_obs2 = st.columns(2)
            with col_obs1:
                if st.button("➕ 添加障碍物", use_container_width=True):
                    if len(st.session_state.temp_obstacle) >= 3:
                        name = obs_name if obs_name else f"障碍物{len(data['obstacles'])+1}"
                        data['obstacles'].append({'name': name, 'coords': st.session_state.temp_obstacle.copy(), 'height': obs_height})
                        planner.add_obstacle(name, st.session_state.temp_obstacle.copy(), obs_height)
                        st.session_state.planned_route = []
                        save_data(data)
                        planner.save_route()
                        st.success(f"✅ {name} 添加成功！")
                        st.session_state.temp_obstacle = []
                    else:
                        st.warning("⚠️ 需要至少3个顶点")

            with col_obs2:
                if st.button("🔄 重置顶点", use_container_width=True):
                    st.session_state.temp_obstacle = []
                    st.rerun()

            if st.button("🗑️ 清空所有障碍物", use_container_width=True):
                data['obstacles'] = []
                planner.obstacles = []
                st.session_state.planned_route = []
                save_data(data)
                planner.save_route()
                st.success("✅ 已清空")

            if st.session_state.temp_obstacle:
                st.subheader("当前顶点")
                for i, coord in enumerate(st.session_state.temp_obstacle):
                    st.write(f"{i+1}. ({coord[0]:.6f}, {coord[1]:.6f})")

            if data['obstacles']:
                st.subheader("障碍物列表")
                for i, obs in enumerate(data['obstacles']):
                    need_fly = "🔴 需要绕飞" if obs.get('height', 50) >= uav_altitude else "🟢 无需绕飞"
                    col_obs_n, col_obs_d = st.columns([4, 1])
                    with col_obs_n:
                        st.write(f"{i+1}. **{obs['name']}** {need_fly} (高度: {obs.get('height', 50)}m, {len(obs['coords'])}个顶点)")
                    with col_obs_d:
                        if st.button("🗑️", key=f"del_obs_{i}"):
                            del data['obstacles'][i]
                            st.session_state.planned_route = []
                            save_data(data)
                            st.rerun()

    # 坐标转换工具
    st.markdown("---")
    with st.expander("🌐 坐标转换工具（WGS-84 / GCJ-02 / BD-09）"):
        col_conv1, col_conv2, col_conv3 = st.columns(3)
        
        with col_conv1:
            st.subheader("输入坐标")
            conv_lat = st.number_input("纬度", value=NANJING_LAT, step=0.0001, format="%.6f", key="conv_lat")
            conv_lng = st.number_input("经度", value=NANJING_LNG, step=0.0001, format="%.6f", key="conv_lng")
            from_sys = st.selectbox("源坐标系", ["WGS-84", "GCJ-02", "BD-09"], key="from_sys")
            to_sys = st.selectbox("目标坐标系", ["GCJ-02", "WGS-84", "BD-09"], key="to_sys")
        
        with col_conv2:
            st.subheader("转换结果")
            if st.button("🔄 开始转换", use_container_width=True, type="primary"):
                result_lng, result_lat = conv_lat, conv_lng
                
                if from_sys == "WGS-84" and to_sys == "GCJ-02":
                    result_lng, result_lat = wgs84_to_gcj02(conv_lng, conv_lat)
                elif from_sys == "GCJ-02" and to_sys == "WGS-84":
                    result_lng, result_lat = gcj02_to_wgs84(conv_lng, conv_lat)
                elif from_sys == "WGS-84" and to_sys == "BD-09":
                    result_lng, result_lat = wgs84_to_bd09(conv_lng, conv_lat)
                elif from_sys == "BD-09" and to_sys == "WGS-84":
                    result_lng, result_lat = bd09_to_wgs84(conv_lng, conv_lat)
                elif from_sys == to_sys:
                    st.info("⚠️ 源坐标系和目标坐标系相同，无需转换")
                else:
                    st.warning("⚠️ 该转换组合暂不支持")
                
                st.session_state.conv_result = (result_lng, result_lat)
                st.success("✅ 转换完成！")
            
            if 'conv_result' in st.session_state:
                result_lng, result_lat = st.session_state.conv_result
                st.metric("转换后纬度", f"{result_lat:.6f}")
                st.metric("转换后经度", f"{result_lng:.6f}")
        
        with col_conv3:
            st.subheader("快速测试")
            st.markdown("**测试点：南京科技职业学院**")
            st.markdown(f"- WGS-84: `{NANJING_LAT:.6f}, {NANJING_LNG:.6f}`")
            
            # 自动转换显示
            gcj_lng, gcj_lat = wgs84_to_gcj02(NANJING_LNG, NANJING_LAT)
            st.markdown(f"- GCJ-02: `{gcj_lat:.6f}, {gcj_lng:.6f}`")
            
            bd_lng, bd_lat = wgs84_to_bd09(NANJING_LNG, NANJING_LAT)
            st.markdown(f"- BD-09: `{bd_lat:.6f}, {bd_lng:.6f}`")
            
            st.markdown("---")
            st.markdown("**说明：**")
            st.markdown("- WGS-84: GPS原始坐标")
            st.markdown("- GCJ-02: 国测局坐标（火星坐标）")
            st.markdown("- BD-09: 百度坐标")

    # 统计信息
    total_dist = 0
    if len(data['waypoints']) >= 2:
        for i in range(len(data['waypoints']) - 1):
            total_dist += haversine(data['waypoints'][i]['lat'], data['waypoints'][i]['lng'],
                                  data['waypoints'][i+1]['lat'], data['waypoints'][i+1]['lng']) / 1000

    col_stat1, col_stat2, col_stat3, col_stat4 = st.columns(4)
    with col_stat1:
        st.metric("航点数量", len(data['waypoints']))
    with col_stat2:
        st.metric("障碍物数量", len(data['obstacles']))
    with col_stat3:
        st.metric("航线距离", f"{total_dist:.2f} km")
    with col_stat4:
        st.metric("安全半径", f"{safety_radius}m")

    if len(data['waypoints']) >= 2 and len(data['obstacles']) > 0:
        conflicts, _ = check_route_conflict(data['waypoints'], data['obstacles'], safety_radius, uav_altitude)
        if conflicts:
            st.subheader("⚠️ 安全警告")
            for conflict in conflicts:
                st.warning(conflict)
        else:
            st.success("✅ 航线安全检测通过！所有航段与障碍物距离符合安全要求")

def calculate_heading(from_lat, from_lng, to_lat, to_lng):
    """计算从一点到另一点的航向角度（0-360度）"""
    dlat = to_lat - from_lat
    dlng = to_lng - from_lng
    angle = math.degrees(math.atan2(dlng, dlat))
    return (angle + 360) % 360

def get_heading_direction(heading):
    """获取航向方向文本"""
    if heading < 22.5 or heading >= 337.5:
        return "北"
    elif heading < 67.5:
        return "东北"
    elif heading < 112.5:
        return "东"
    elif heading < 157.5:
        return "东南"
    elif heading < 202.5:
        return "南"
    elif heading < 247.5:
        return "西南"
    elif heading < 292.5:
        return "西"
    else:
        return "西北"

def render_flight_monitor_page():
    st.header("🚁 飞行监控")

    data = st.session_state.data

    col1, col2 = st.columns([1, 2])

    with col1:
        st.subheader("🎮 模拟控制")

        col_ctrl1, col_ctrl2 = st.columns(2)
        with col_ctrl1:
            if st.button("▶️ 启动模拟", use_container_width=True):
                if len(data['waypoints']) < 2:
                    st.warning("⚠️ 请先添加至少2个航点！")
                else:
                    st.session_state.is_simulating = True
                    st.session_state.heartbeat_data = []
                    st.session_state.flight_path = []
                    st.session_state.battery_level = 100.0
                    st.session_state.current_position = [data['waypoints'][0]['lat'], data['waypoints'][0]['lng']]
                    st.session_state.current_waypoint = 0
                    st.session_state.start_time = datetime.now()
                    st.session_state.completed_distance = 0.0
                    st.session_state.total_distance = 0.0
                    st.session_state.heading = 0.0
                    for i in range(len(data['waypoints']) - 1):
                        st.session_state.total_distance += haversine(
                            data['waypoints'][i]['lat'], data['waypoints'][i]['lng'],
                            data['waypoints'][i+1]['lat'], data['waypoints'][i+1]['lng']
                        )
                    st.success("✅ 模拟已启动")

        with col_ctrl2:
            if st.button("⏹️ 停止模拟", use_container_width=True):
                st.session_state.is_simulating = False
                st.success("✅ 模拟已停止")

        st.markdown("---")

        st.subheader("📊 飞行状态")

        if st.session_state.is_simulating and data['waypoints']:
            current_lat, current_lng = st.session_state.current_position
            target_wp = data['waypoints'][min(st.session_state.current_waypoint, len(data['waypoints'])-1)]

            st.session_state.heading = calculate_heading(
                current_lat, current_lng, target_wp['lat'], target_wp['lng']
            )

            col_status1, col_status2 = st.columns(2)

            with col_status1:
                wp = data['waypoints'][st.session_state.current_waypoint]
                st.metric("当前航点", f"{wp['name']} ({st.session_state.current_waypoint + 1}/{len(data['waypoints'])})")
                st.metric("飞行速度", f"{st.session_state.flight_speed} m/s")
                st.metric("飞行高度", f"{data.get('uav_altitude', 100)} m")

            with col_status2:
                heading_dir = get_heading_direction(st.session_state.heading)
                st.metric("航向", f"{heading_dir} {st.session_state.heading:.1f}°")
                st.metric("当前坐标", f"{current_lat:.5f}, {current_lng:.5f}")

            st.markdown("---")

            elapsed_time = datetime.now() - st.session_state.start_time
            elapsed_str = str(elapsed_time).split('.')[0]
            st.metric("已用时间", elapsed_str)

            remaining_distance = st.session_state.total_distance - st.session_state.completed_distance
            remaining_time = timedelta(seconds=remaining_distance / st.session_state.flight_speed) if st.session_state.flight_speed > 0 else timedelta(0)
            st.metric("剩余距离", f"{remaining_distance/1000:.2f} km")
            st.metric("预计到达", str(remaining_time).split('.')[0])

            progress = min((st.session_state.completed_distance / max(st.session_state.total_distance, 1)) * 100, 100)
            st.progress(min(progress / 100, 1.0), text=f"航线进度: {progress:.1f}%")
        else:
            col_status1, col_status2 = st.columns(2)
            with col_status1:
                st.metric("当前航点", "未启动")
                st.metric("飞行速度", "-")
                st.metric("飞行高度", "-")
            with col_status2:
                st.metric("航向", "-")
                st.metric("当前坐标", "-")

        st.markdown("---")

        st.subheader("🔋 电量监控")
        battery_color = "green" if st.session_state.battery_level > 50 else ("yellow" if st.session_state.battery_level > 20 else "red")
        st.metric("电池电量", f"{st.session_state.battery_level:.1f}%")
        st.progress(st.session_state.battery_level / 100)
        if st.session_state.battery_level <= 20:
            st.warning("⚠️ 电量不足！请尽快返航")

    with col2:
        st.subheader("🗺️ 飞行轨迹")

        m = create_satellite_map(st.session_state.current_position[0], st.session_state.current_position[1])

        if st.session_state.planned_route:
            folium.PolyLine(st.session_state.planned_route, color='#00D4AA', weight=3, opacity=0.7).add_to(m)
        elif data['waypoints']:
            route_coords = [(wp['lat'], wp['lng']) for wp in data['waypoints']]
            folium.PolyLine(route_coords, color='#FFA500', weight=3, opacity=0.7).add_to(m)

        if data['waypoints']:
            for i, wp in enumerate(data['waypoints']):
                color = 'green' if i == 0 else ('red' if i == len(data['waypoints'])-1 else 'orange')
                icon_color = 'blue' if i == st.session_state.current_waypoint else color
                folium.Marker([wp['lat'], wp['lng']], popup=f"{wp['name']}", 
                            icon=folium.Icon(color=icon_color, icon='info-sign')).add_to(m)

        if len(st.session_state.flight_path) > 1:
            folium.PolyLine(st.session_state.flight_path, color='#3366FF', weight=4, opacity=0.9).add_to(m)

        if st.session_state.flight_path:
            folium.Marker(
                st.session_state.flight_path[-1],
                popup="✈️ 当前位置",
                icon=folium.Icon(color='blue', icon='plane', prefix='glyphicon')
            ).add_to(m)

        for obs in data['obstacles']:
            obs_height = obs.get('height', 50)
            color = '#FF6B6B' if obs_height >= data.get('uav_altitude', 100) else '#4CAF50'
            folium.Polygon(obs['coords'], color=color, fill=True, fillColor=color, fillOpacity=0.3, weight=2).add_to(m)

        st_folium(m, width=700, height=500)

    st.markdown("---")
    
    # 通信链路展示模块
    st.subheader("📡 通信链路状态")
    
    col_link1, col_link2, col_link3 = st.columns(3)
    
    with col_link1:
        st.markdown("**GCS ↔ OBC**")
        gcs_obc = st.session_state.link_status['gcs_obc']
        if st.session_state.is_simulating:
            gcs_obc['connected'] = True
            gcs_obc['rssi'] = random.randint(-85, -45)
            gcs_obc['latency'] = random.randint(20, 80)
        status_color = "🟢" if gcs_obc['connected'] else "🔴"
        st.metric("状态", f"{status_color} {'在线' if gcs_obc['connected'] else '离线'}")
        if gcs_obc['connected']:
            st.metric("信号强度", f"{gcs_obc['rssi']} dBm")
            st.metric("延迟", f"{gcs_obc['latency']} ms")
    
    with col_link2:
        st.markdown("**OBC ↔ FCU**")
        obc_fcu = st.session_state.link_status['obc_fcu']
        if st.session_state.is_simulating:
            obc_fcu['connected'] = True
            obc_fcu['rssi'] = random.randint(-70, -30)
            obc_fcu['latency'] = random.randint(5, 25)
        status_color = "🟢" if obc_fcu['connected'] else "🔴"
        st.metric("状态", f"{status_color} {'在线' if obc_fcu['connected'] else '离线'}")
        if obc_fcu['connected']:
            st.metric("信号强度", f"{obc_fcu['rssi']} dBm")
            st.metric("延迟", f"{obc_fcu['latency']} ms")
    
    with col_link3:
        st.markdown("**GCS ↔ FCU**")
        gcs_fcu = st.session_state.link_status['gcs_fcu']
        if st.session_state.is_simulating:
            gcs_fcu['connected'] = True
            gcs_fcu['rssi'] = random.randint(-90, -50)
            gcs_fcu['latency'] = random.randint(30, 100)
        status_color = "🟢" if gcs_fcu['connected'] else "🔴"
        st.metric("状态", f"{status_color} {'在线' if gcs_fcu['connected'] else '离线'}")
        if gcs_fcu['connected']:
            st.metric("信号强度", f"{gcs_fcu['rssi']} dBm")
            st.metric("延迟", f"{gcs_fcu['latency']} ms")
    
    st.markdown("---")
    
    # GCS-OBC-FCU 拓扑图
    st.subheader("🌐 系统拓扑图 (GCS-OBC-FCU)")
    
    # 使用HTML/CSS绘制拓扑图
    topology_html = """
    <style>
        .topology-container {
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
            background: #f8f9fa;
            border-radius: 10px;
        }
        .node {
            width: 100px;
            height: 100px;
            border-radius: 50%;
            display: flex;
            flex-direction: column;
            justify-content: center;
            align-items: center;
            color: white;
            font-weight: bold;
            font-size: 14px;
            text-align: center;
            margin: 0 30px;
        }
        .node-gcs { background: #007bff; }
        .node-obc { background: #28a745; }
        .node-fcu { background: #dc3545; }
        .connection {
            display: flex;
            flex-direction: column;
            align-items: center;
            margin: 0 10px;
        }
        .line {
            width: 80px;
            height: 3px;
            background: #28a745;
            position: relative;
        }
        .line::before {
            content: '';
            position: absolute;
            right: -8px;
            top: -4px;
            width: 0;
            height: 0;
            border-left: 10px solid #28a745;
            border-top: 5px solid transparent;
            border-bottom: 5px solid transparent;
        }
        .line-bidirectional::after {
            content: '';
            position: absolute;
            left: -8px;
            top: -4px;
            width: 0;
            height: 0;
            border-right: 10px solid #28a745;
            border-top: 5px solid transparent;
            border-bottom: 5px solid transparent;
        }
        .line-offline {
            background: #dc3545;
        }
        .line-offline::before {
            border-left-color: #dc3545;
        }
        .status-text {
            font-size: 12px;
            margin-top: 5px;
            color: #666;
        }
    </style>
    <div class="topology-container">
        <div class="node node-gcs">
            <div>📱</div>
            <div>GCS</div>
            <div style="font-size: 10px;">地面站</div>
        </div>
        <div class="connection">
            <div class="line line-bidirectional" id="gcs-obc-line"></div>
            <div class="status-text" id="gcs-obc-status">Telemetry Link</div>
        </div>
        <div class="node node-obc">
            <div>🖥️</div>
            <div>OBC</div>
            <div style="font-size: 10px;">机载计算机</div>
        </div>
        <div class="connection">
            <div class="line line-bidirectional" id="obc-fcu-line"></div>
            <div class="status-text" id="obc-fcu-status">MAVLink</div>
        </div>
        <div class="node node-fcu">
            <div>🚁</div>
            <div>FCU</div>
            <div style="font-size: 10px;">飞控单元</div>
        </div>
    </div>
    """
    st.components.v1.html(topology_html, height=180)
    
    st.markdown("---")
    
    # MAVLink 数据流与报文显示
    st.subheader("📨 MAVLink 数据流")
    
    col_mav1, col_mav2 = st.columns([1, 2])
    
    with col_mav1:
        st.markdown("**报文统计**")
        if st.session_state.is_simulating:
            st.session_state.mavlink_msg_count += random.randint(5, 15)
        st.metric("总报文数", st.session_state.mavlink_msg_count)
        st.metric("报文速率", f"{random.randint(10, 50) if st.session_state.is_simulating else 0} Hz")
        st.metric("丢包率", f"{random.uniform(0.1, 2.5) if st.session_state.is_simulating else 0:.2f}%")
    
    with col_mav2:
        st.markdown("**实时报文**")
        
        # 生成模拟MAVLink报文
        if st.session_state.is_simulating:
            msg_types = ['HEARTBEAT', 'GLOBAL_POSITION_INT', 'ATTITUDE', 'BATTERY_STATUS', 
                        'GPS_RAW_INT', 'VFR_HUD', 'SYS_STATUS', 'MISSION_CURRENT']
            new_msg = {
                'time': datetime.now().strftime("%H:%M:%S.%f")[:-3],
                'type': random.choice(msg_types),
                'sysid': 1,
                'compid': 1,
                'seq': st.session_state.mavlink_msg_count
            }
            st.session_state.mavlink_messages.insert(0, new_msg)
            if len(st.session_state.mavlink_messages) > 20:
                st.session_state.mavlink_messages = st.session_state.mavlink_messages[:20]
        
        if st.session_state.mavlink_messages:
            # 显示最近10条报文
            display_msgs = st.session_state.mavlink_messages[:10]
            msg_df = pd.DataFrame(display_msgs)
            st.dataframe(msg_df, use_container_width=True, hide_index=True)
        else:
            st.info("💡 启动模拟后显示MAVLink报文")
    
    # MAVLink报文详情
    with st.expander("📋 MAVLink报文详情"):
        if st.session_state.mavlink_messages:
            for i, msg in enumerate(st.session_state.mavlink_messages[:5]):
                with st.container():
                    col_msg1, col_msg2, col_msg3 = st.columns([2, 2, 3])
                    with col_msg1:
                        st.markdown(f"**{msg['type']}**")
                    with col_msg2:
                        st.markdown(f"`sysid:{msg['sysid']}` `compid:{msg['compid']}`")
                    with col_msg3:
                        st.markdown(f"seq:{msg['seq']} | {msg['time']}")
                    st.markdown("---")
        else:
            st.info("暂无报文数据")
    
    st.markdown("---")
    
    st.subheader("📈 数据图表")
    
    col_chart1, col_chart2 = st.columns(2)
    
    with col_chart1:
        if st.session_state.heartbeat_data:
            df = pd.DataFrame(st.session_state.heartbeat_data)
            fig = go.Figure()
            fig.add_trace(go.Scatter(x=df['time'], y=df['seq'], mode='lines+markers', name='心跳序号'))
            fig.update_layout(height=300, title='心跳包时序图')
            st.plotly_chart(fig, use_container_width=True)
        else:
            st.info("💡 启动模拟后显示心跳曲线")

    with col_chart2:
        if st.session_state.heartbeat_data:
            df = pd.DataFrame(st.session_state.heartbeat_data)
            battery_values = [100 - i * 0.05 for i in range(len(df))]
            fig_bat = go.Figure()
            fig_bat.add_trace(go.Scatter(x=df['time'], y=battery_values, mode='lines', name='电量', line=dict(color='#4CAF50')))
            fig_bat.update_layout(height=300, title='电量变化曲线')
            st.plotly_chart(fig_bat, use_container_width=True)
        else:
            st.info("💡 启动模拟后显示电量曲线")

    if st.session_state.is_simulating:
        import time
        time.sleep(1)

        seq = len(st.session_state.heartbeat_data) + 1
        st.session_state.heartbeat_data.append({'seq': seq, 'time': datetime.now().strftime("%H:%M:%S")})

        st.session_state.battery_level = max(0, st.session_state.battery_level - 0.05)

        if data['waypoints'] and st.session_state.current_waypoint < len(data['waypoints']):
            target_wp = data['waypoints'][st.session_state.current_waypoint]
            current_lat, current_lng = st.session_state.current_position

            dlat = target_wp['lat'] - current_lat
            dlng = target_wp['lng'] - current_lng
            dist = math.sqrt(dlat**2 + dlng**2) * 111000

            if dist < 5:
                st.session_state.current_waypoint += 1
                if st.session_state.current_waypoint >= len(data['waypoints']):
                    st.session_state.is_simulating = False
                    st.session_state.completed_distance = st.session_state.total_distance
                    st.success("🎉 已到达终点！")
            else:
                step = 0.0001
                current_lat += (dlat / (dist/111000)) * step
                current_lng += (dlng / (dist/111000)) * step
                current_lat += (random.random() - 0.5) * 0.00002
                current_lng += (random.random() - 0.5) * 0.00002

                st.session_state.completed_distance = min(
                    st.session_state.completed_distance + 11.1,
                    st.session_state.total_distance
                )

            st.session_state.current_position = [current_lat, current_lng]
            st.session_state.flight_path.append((current_lat, current_lng))

        st.rerun()

def main():
    init_session_state()

    st.sidebar.markdown("<h2 style='color: #00D4AA; text-align: center;'>🚁 UAV Monitor</h2>", unsafe_allow_html=True)
    st.sidebar.markdown("---")

    page = st.sidebar.radio("导航", ["🏠 首页", "🗺️ 航线规划", "🚁 飞行监控"])

    st.sidebar.markdown("---")

    with st.sidebar.expander("ℹ️ 系统信息"):
        st.markdown("""
        **版本**: v2.0.0
        
        **功能模块**:
        - 航线规划（含安全半径）
        - 障碍物圈选（高度设置）
        - 绕飞路径规划（左/右/最优）
        - 飞行监控模拟
        
        **技术栈**:
        - Streamlit
        - Folium卫星地图
        - Plotly可视化
        """)

    if page == "🏠 首页":
        render_home_page()
    elif page == "🗺️ 航线规划":
        render_route_planning_page()
    elif page == "🚁 飞行监控":
        render_flight_monitor_page()

if __name__ == "__main__":
    main()
