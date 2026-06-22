import streamlit as st
import streamlit.components.v1 as components
import plotly.graph_objects as go
import pandas as pd
from datetime import datetime
import time
import json
import threading
import random
import math

class HeartbeatPacket:
    def __init__(self, seq, timestamp, battery, altitude, speed, pitch, roll, yaw, latitude, longitude):
        self.seq = seq
        self.timestamp = timestamp
        self.battery = battery
        self.altitude = altitude
        self.speed = speed
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.latitude = latitude
        self.longitude = longitude

class Waypoint:
    def __init__(self, lat, lng, altitude=100.0, speed=15.0, name=""):
        self.lat = lat
        self.lng = lng
        self.altitude = altitude
        self.speed = speed
        self.name = name

class Obstacle:
    def __init__(self, name, coords):
        self.name = name
        self.coords = coords

class UAVSimulator:
    def __init__(self, interval=1.0, offline_threshold=3.0):
        self.interval = interval
        self.offline_threshold = offline_threshold
        self.sequence = 0
        self.is_running = False
        self.heartbeat_history = []
        self.last_heartbeat_time = None
        self._thread = None
        self._lock = threading.Lock()
        self.base_lat = 32.234104
        self.base_lon = 118.749421
        self.waypoints = []
        self.obstacles = []
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = False
        self.safety_distance = 0.0002
        self._generate_default_route()
    
    def _generate_default_route(self):
        self.waypoints = [
            Waypoint(32.234104, 118.749421, 100.0, 15.0, "起点"),
            Waypoint(32.234500, 118.750000, 100.0, 15.0, "航点1"),
            Waypoint(32.235000, 118.749800, 120.0, 18.0, "航点2"),
            Waypoint(32.235200, 118.749000, 120.0, 15.0, "航点3"),
            Waypoint(32.234800, 118.748500, 100.0, 15.0, "航点4"),
            Waypoint(32.234300, 118.748800, 100.0, 15.0, "航点5"),
            Waypoint(32.234104, 118.749421, 100.0, 15.0, "终点"),
        ]
    
    def load_route_from_file(self, filename='route.json'):
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.waypoints = [Waypoint(wp['lat'], wp['lng'], wp.get('altitude', 100.0), wp.get('speed', 15.0), wp.get('name', '')) for wp in data.get('waypoints', [])]
            self.obstacles = [Obstacle(obs['name'], obs['coords']) for obs in data.get('obstacles', [])]
            self.safety_distance = data.get('safety_distance', 0.0002)
            self.current_waypoint_index = 0
            self.progress_to_next_waypoint = 0.0
            self.flying_route = len(self.waypoints) >= 2
            return True
        except Exception as e:
            print(f"Failed to load route: {e}")
            return False
    
    def set_circular_route(self, center_lat, center_lng, radius_km, num_points=8):
        waypoints = []
        for i in range(num_points):
            angle = (i / num_points) * 2 * math.pi
            dx = radius_km * math.cos(angle) / 111.0
            dy = radius_km * math.sin(angle) / 111.0
            waypoints.append(Waypoint(center_lat + dx, center_lng + dy, 100.0, 15.0, f"WP{i+1}"))
        waypoints.append(waypoints[0])
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = True
    
    def set_rectangular_route(self, start_lat, start_lng, width_km, height_km):
        waypoints = [
            Waypoint(start_lat, start_lng, 100.0, 15.0, "起点"),
            Waypoint(start_lat, start_lng + width_km / 111.0, 100.0, 15.0, "WP1"),
            Waypoint(start_lat + height_km / 111.0, start_lng + width_km / 111.0, 100.0, 15.0, "WP2"),
            Waypoint(start_lat + height_km / 111.0, start_lng, 100.0, 15.0, "WP3"),
            Waypoint(start_lat, start_lng, 100.0, 15.0, "终点"),
        ]
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = True
    
    def _point_in_polygon(self, lat, lng, polygon_coords):
        n = len(polygon_coords)
        inside = False
        for i in range(n):
            j = (i + 1) % n
            xi, yi = polygon_coords[i]
            xj, yj = polygon_coords[j]
            if ((yi > lat) != (yj > lat)):
                x_intersect = (lat - yi) * (xj - xi) / (yj - yi) + xi
                if lng < x_intersect + self.safety_distance:
                    inside = not inside
        return inside
    
    def _check_obstacle_collision(self, lat, lng):
        for obs in self.obstacles:
            if self._point_in_polygon(lat, lng, obs.coords):
                return obs
        return None
    
    def _calculate_detour(self, current_lat, current_lng, target_lat, target_lng):
        mid_lat = (current_lat + target_lat) / 2
        mid_lng = (current_lng + target_lng) / 2
        
        perp_lat = -(target_lng - current_lng)
        perp_lng = target_lat - current_lat
        length = math.sqrt(perp_lat**2 + perp_lng**2)
        if length > 0:
            perp_lat /= length
            perp_lng /= length
        
        offset = self.safety_distance * 3
        return mid_lat + perp_lat * offset, mid_lng + perp_lng * offset
    
    def _get_current_position(self):
        if not self.flying_route or len(self.waypoints) < 2:
            return (self.base_lat + random.uniform(-0.001, 0.001),
                    self.base_lon + random.uniform(-0.001, 0.001),
                    100 + random.uniform(-10, 10),
                    15 + random.uniform(-3, 3))
        
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.current_waypoint_index = 0
            self.progress_to_next_waypoint = 0.0
        
        current_wp = self.waypoints[self.current_waypoint_index]
        next_wp = self.waypoints[self.current_waypoint_index + 1]
        
        lat = current_wp.lat + (next_wp.lat - current_wp.lat) * self.progress_to_next_waypoint
        lng = current_wp.lng + (next_wp.lng - current_wp.lng) * self.progress_to_next_waypoint
        alt = current_wp.altitude + (next_wp.altitude - current_wp.altitude) * self.progress_to_next_waypoint
        spd = current_wp.speed + (next_wp.speed - current_wp.speed) * self.progress_to_next_waypoint
        
        obstacle = self._check_obstacle_collision(lat, lng)
        if obstacle:
            detour_lat, detour_lng = self._calculate_detour(lat, lng, next_wp.lat, next_wp.lng)
            lat, lng = detour_lat, detour_lng
            spd *= 0.8
        
        self.progress_to_next_waypoint += 0.02
        
        if self.progress_to_next_waypoint >= 1.0:
            self.progress_to_next_waypoint = 0.0
            self.current_waypoint_index += 1
        
        noise = 0.00005
        return (
            lat + random.uniform(-noise, noise),
            lng + random.uniform(-noise, noise),
            max(50, alt + random.uniform(-2, 2)),
            max(5, spd + random.uniform(-1, 1))
        )
    
    def _calculate_yaw(self, lat1, lng1, lat2, lng2):
        d_lng = lng2 - lng1
        yaw = math.atan2(d_lng, lat2 - lat1) * 180 / math.pi
        return (yaw + 360) % 360
    
    def _generate_packet(self):
        self.sequence += 1
        lat, lng, alt, spd = self._get_current_position()
        
        if self.flying_route and len(self.waypoints) > 1:
            if self.current_waypoint_index < len(self.waypoints) - 1:
                next_wp = self.waypoints[self.current_waypoint_index + 1]
                yaw = self._calculate_yaw(lat, lng, next_wp.lat, next_wp.lng)
            else:
                next_wp = self.waypoints[0]
                yaw = self._calculate_yaw(lat, lng, next_wp.lat, next_wp.lng)
        else:
            yaw = random.uniform(0, 360)
        
        return HeartbeatPacket(
            seq=self.sequence,
            timestamp=datetime.now(),
            battery=max(20, 100 - self.sequence * 0.02 + random.uniform(-0.5, 0.5)),
            altitude=alt,
            speed=spd,
            pitch=random.uniform(-2, 2),
            roll=random.uniform(-2, 2),
            yaw=yaw,
            latitude=lat,
            longitude=lng
        )
    
    def _simulate(self):
        while self.is_running:
            packet = self._generate_packet()
            with self._lock:
                self.heartbeat_history.append(packet)
                self.last_heartbeat_time = packet.timestamp
                if len(self.heartbeat_history) > 1000:
                    self.heartbeat_history = self.heartbeat_history[-500:]
            time.sleep(self.interval)
    
    def start(self):
        if not self.is_running:
            self.is_running = True
            self._thread = threading.Thread(target=self._simulate, daemon=True)
            self._thread.start()
    
    def stop(self):
        self.is_running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
    
    def check_offline(self):
        if self.last_heartbeat_time is None:
            return True
        elapsed = (datetime.now() - self.last_heartbeat_time).total_seconds()
        return elapsed > self.offline_threshold
    
    def get_offline_duration(self):
        if self.last_heartbeat_time is None:
            return 0.0
        return (datetime.now() - self.last_heartbeat_time).total_seconds()
    
    def get_history_dataframe(self, last_n=100):
        with self._lock:
            if not self.heartbeat_history:
                return pd.DataFrame()
            data = self.heartbeat_history[-last_n:]
            return pd.DataFrame([{
                '序号': p.seq,
                '时间': p.timestamp.strftime('%H:%M:%S'),
                '电量': round(p.battery, 1),
                '高度': round(p.altitude, 1),
                '速度': round(p.speed, 1),
                '俯仰角': round(p.pitch, 2),
                '横滚角': round(p.roll, 2),
                '偏航角': round(p.yaw, 2),
                '纬度': round(p.latitude, 6),
                '经度': round(p.longitude, 6)
            } for p in data])
    
    def get_latest_packet(self):
        with self._lock:
            if self.heartbeat_history:
                return self.heartbeat_history[-1]
        return None
    
    def get_status_log(self, last_n=50):
        with self._lock:
            if not self.heartbeat_history:
                return []
            data = self.heartbeat_history[-last_n:]
            logs = []
            for p in data:
                status = "正常" if p.battery > 30 else "低电量警告"
                logs.append(f"[{p.timestamp.strftime('%H:%M:%S')}] 心跳#{p.seq} | 状态: {status} | 电量: {p.battery:.1f}% | 高度: {p.altitude:.1f}m")
            return logs
    
    def get_route_waypoints(self):
        return [(wp.lat, wp.lng) for wp in self.waypoints]
    
    def get_obstacles(self):
        return [{'name': obs.name, 'coords': obs.coords} for obs in self.obstacles]

class GroundStation:
    def __init__(self, uav):
        self.uav = uav
        self.alert_history = []
        self.obstacle_alerts = []
        
    def monitor(self):
        is_offline = self.uav.check_offline()
        latest = self.uav.get_latest_packet()
        
        result = {
            'status': '离线' if is_offline else '在线',
            'is_offline': is_offline,
            'offline_duration': self.uav.get_offline_duration() if is_offline else 0,
            'latest_packet': latest,
            'sequence': latest.seq if latest else 0,
            'last_time': latest.timestamp if latest else None,
            'current_waypoint': self.uav.current_waypoint_index,
            'total_waypoints': len(self.uav.waypoints)
        }
        
        if is_offline:
            alert_msg = f"[{datetime.now().strftime('%H:%M:%S')}] 🚨 警告: 无人机已离线 {result['offline_duration']:.1f} 秒!"
            if alert_msg not in self.alert_history[-1:] and len(self.alert_history) == 0 or alert_msg != self.alert_history[-1]:
                self.alert_history.append(alert_msg)
        
        if latest:
            current_pos = (latest.latitude, latest.longitude)
            for obs in self.uav.obstacles:
                if self.uav._point_in_polygon(latest.latitude, latest.longitude, obs.coords):
                    alert_msg = f"[{datetime.now().strftime('%H:%M:%S')}] ⚠️ 警告: 接近障碍物 {obs.name}"
                    if alert_msg not in self.obstacle_alerts[-1:] and len(self.obstacle_alerts) == 0 or alert_msg != self.obstacle_alerts[-1]:
                        self.obstacle_alerts.append(alert_msg)
                        self.alert_history.append(alert_msg)
        
        if len(self.alert_history) > 100:
            self.alert_history = self.alert_history[-50:]
        
        return result
    
    def get_alerts(self, last_n=20):
        return self.alert_history[-last_n:]
    
    def get_obstacle_alerts(self, last_n=10):
        return self.obstacle_alerts[-last_n:]

def render_flight_monitor_page():
    st.header("🚁 飞行监控")

    if 'uav_simulator' not in st.session_state:
        st.session_state.uav_simulator = UAVSimulator(interval=1.0, offline_threshold=3.0)
        st.session_state.ground_station = GroundStation(st.session_state.uav_simulator)
        st.session_state.was_offline = False

    uav = st.session_state.uav_simulator
    ground_station = st.session_state.ground_station

    tab_main, tab_data, tab_map = st.tabs(["📡 实时监控", "📊 数据分析", "🗺️ 地图监控"])

    with tab_main:
        col_control, col_status = st.columns([1, 2])

        with col_control:
            st.subheader("🎮 控制面板")

            col_btn1, col_btn2 = st.columns(2)
            with col_btn1:
                if st.button("▶️ 启动模拟", key="start_sim", disabled=uav.is_running, use_container_width=True):
                    uav.start()
                    st.session_state.was_offline = False
                    st.success("✅ 无人机模拟已启动!")
                    st.rerun()

            with col_btn2:
                if st.button("⏹️ 停止模拟", key="stop_sim", disabled=not uav.is_running, use_container_width=True):
                    uav.stop()
                    st.warning("⏹️ 无人机模拟已停止!")
                    st.rerun()

            st.markdown("---")
            st.subheader("🛤️ 航线管理")
            
            route_mode = st.selectbox("选择航线模式", 
                                     ["默认航线", "矩形航线", "圆形航线", "从文件加载"],
                                     key="route_mode")
            
            if route_mode == "矩形航线":
                col_w1, col_w2 = st.columns(2)
                with col_w1:
                    width_km = st.number_input("宽度(km)", min_value=0.1, max_value=2.0, value=0.2, step=0.1)
                with col_w2:
                    height_km = st.number_input("高度(km)", min_value=0.1, max_value=2.0, value=0.2, step=0.1)
                if st.button("应用矩形航线", use_container_width=True):
                    uav.set_rectangular_route(uav.base_lat, uav.base_lon, width_km, height_km)
                    st.success("✅ 矩形航线已设置!")
            
            elif route_mode == "圆形航线":
                col_c1, col_c2 = st.columns(2)
                with col_c1:
                    radius_km = st.number_input("半径(km)", min_value=0.1, max_value=1.0, value=0.15, step=0.05)
                with col_c2:
                    num_points = st.number_input("点数", min_value=4, max_value=16, value=8, step=1)
                if st.button("应用圆形航线", use_container_width=True):
                    uav.set_circular_route(uav.base_lat, uav.base_lon, radius_km, num_points)
                    st.success("✅ 圆形航线已设置!")
            
            elif route_mode == "从文件加载":
                if st.button("📥 加载航线规划", use_container_width=True):
                    success = uav.load_route_from_file()
                    if success:
                        st.success(f"✅ 成功加载 {len(uav.waypoints)} 个航点和 {len(uav.obstacles)} 个障碍物!")
                    else:
                        st.error("❌ 加载失败，请先在航线规划页面保存航线")
            
            else:
                if st.button("应用默认航线", use_container_width=True):
                    uav._generate_default_route()
                    uav.current_waypoint_index = 0
                    uav.progress_to_next_waypoint = 0.0
                    uav.flying_route = True
                    st.success("✅ 默认航线已设置!")

            st.markdown("---")
            st.subheader("⚙️ 参数设置")
            new_interval = st.number_input(
                "心跳间隔(秒)",
                min_value=0.5, max_value=5.0,
                value=uav.interval, step=0.5
            )
            if new_interval != uav.interval:
                uav.interval = new_interval

            new_threshold = st.number_input(
                "离线阈值(秒)",
                min_value=1.0, max_value=10.0,
                value=uav.offline_threshold, step=0.5
            )
            if new_threshold != uav.offline_threshold:
                uav.offline_threshold = new_threshold

            st.markdown("---")
            st.subheader("🔌 测试功能")
            if st.button("🔌 模拟离线5秒", key="simulate_offline", use_container_width=True):
                if uav.is_running:
                    uav.stop()
                    st.session_state.was_offline = True
                    st.rerun()

        with col_status:
            st.subheader("📡 连接状态")

            status = ground_station.monitor()

            if status['is_offline']:
                offline_duration = status['offline_duration']
                st.markdown(f"""
                <div style="background: linear-gradient(135deg, #ff4444 0%, #cc0000 100%);
                            padding: 20px; border-radius: 10px; text-align: center;
                            animation: pulse 1s infinite;">
                    <h2 style="color: white; margin: 0;">🚨 无人机离线警报!</h2>
                    <p style="color: white; font-size: 18px; margin: 10px 0 0 0;">
                        已离线 <span style="font-size: 24px; font-weight: bold;">{offline_duration:.1f}</span> 秒
                    </p>
                </div>
                <style>@keyframes pulse {{ 0% {{ opacity: 1; }} 50% {{ opacity: 0.7; }} 100% {{ opacity: 1; }} }}</style>
                """, unsafe_allow_html=True)

                alerts = ground_station.get_alerts(last_n=5)
                if alerts:
                    st.markdown("**🔔 最新报警:**")
                    st.code(alerts[-1], language="log")
            else:
                st.markdown(f"""
                <div style="background: linear-gradient(135deg, #28a745 0%, #20c997 100%);
                            padding: 20px; border-radius: 10px; text-align: center;">
                    <h2 style="color: white; margin: 0;">✅ 在线</h2>
                    <p style="color: white; font-size: 14px; margin: 8px 0 0 0;">
                        心跳序号: <span style="font-size: 18px; font-weight: bold;">{status['sequence']}</span>
                    </p>
                </div>
                """, unsafe_allow_html=True)

            latest = status['latest_packet']
            if latest:
                st.markdown("---")
                st.subheader("📈 实时数据")

                col_m1, col_m2, col_m3, col_m4 = st.columns(4)
                with col_m1:
                    battery_delta = f"{latest.battery - 100:.1f}%" if latest.battery < 30 else None
                    st.metric("🔋 电量", f"{latest.battery:.1f}%", battery_delta,
                             delta_color="inverse" if latest.battery < 30 else "normal")
                with col_m2:
                    st.metric("📏 高度", f"{latest.altitude:.1f}m")
                with col_m3:
                    st.metric("⚡ 速度", f"{latest.speed:.1f}m/s")
                with col_m4:
                    st.metric("🧭 偏航角", f"{latest.yaw:.1f}°")

                st.markdown("---")
                st.subheader("📍 GPS坐标")
                col_g1, col_g2 = st.columns(2)
                with col_g1:
                    st.metric("纬度", f"{latest.latitude:.6f}°")
                with col_g2:
                    st.metric("经度", f"{latest.longitude:.6f}°")

                st.markdown("---")
                st.subheader("🛤️ 航点进度")
                if uav.flying_route and len(uav.waypoints) > 0:
                    progress = (status['current_waypoint'] / max(1, status['total_waypoints'] - 1)) * 100
                    st.progress(progress)
                    st.markdown(f"当前航点: **{uav.waypoints[status['current_waypoint']].name if uav.waypoints[status['current_waypoint']].name else f'WP{status['current_waypoint'] + 1}'}** ({status['current_waypoint'] + 1}/{status['total_waypoints']})")
                else:
                    st.info("💡 未启用航线飞行")

                st.caption(f"🕐 更新时间: {latest.timestamp.strftime('%H:%M:%S')}")

    with tab_data:
        df = uav.get_history_dataframe(last_n=100)

        if not df.empty:
            col_chart1, col_chart2 = st.columns(2)

            with col_chart1:
                st.subheader("💓 心跳序号曲线")
                fig_seq = go.Figure()
                fig_seq.add_trace(go.Scatter(x=df['时间'], y=df['序号'], mode='lines+markers',
                                             name='心跳序号', line=dict(color='#00D4AA', width=2), marker=dict(size=5)))
                fig_seq.update_layout(template='plotly_dark', height=300, hovermode='x unified',
                                      margin=dict(l=40, r=20, t=40, b=40))
                st.plotly_chart(fig_seq, use_container_width=True)

            with col_chart2:
                st.subheader("🔋 电池电量曲线")
                fig_battery = go.Figure()
                fig_battery.add_trace(go.Scatter(x=df['时间'], y=df['电量'], mode='lines',
                                                 name='电量', line=dict(color='#FF6B6B', width=2),
                                                 fill='tozeroy', fillcolor='rgba(255, 107, 107, 0.2)'))
                fig_battery.add_hline(y=30, line_dash="dash", line_color="red", annotation_text="低电量警告")
                fig_battery.update_layout(template='plotly_dark', height=300, yaxis=dict(range=[0, 105]),
                                          hovermode='x unified', margin=dict(l=40, r=20, t=40, b=40))
                st.plotly_chart(fig_battery, use_container_width=True)

            st.markdown("### 🎯 飞行姿态")

            col_att1, col_att2 = st.columns(2)

            with col_att1:
                fig_att = go.Figure()
                fig_att.add_trace(go.Scatter(x=df['时间'], y=df['俯仰角'], mode='lines', name='俯仰角',
                                             line=dict(color='#4ECDC4', width=2)))
                fig_att.add_trace(go.Scatter(x=df['时间'], y=df['横滚角'], mode='lines', name='横滚角',
                                             line=dict(color='#FFE66D', width=2)))
                fig_att.update_layout(title='俯仰角/横滚角', template='plotly_dark', height=280,
                                      legend=dict(orientation="h", y=1.1), margin=dict(l=40, r=20, t=40, b=40))
                st.plotly_chart(fig_att, use_container_width=True)

            with col_att2:
                fig_alt = go.Figure()
                fig_alt.add_trace(go.Scatter(x=df['时间'], y=df['高度'], mode='lines', name='高度',
                                              line=dict(color='#95E1D3', width=2), fill='tozeroy',
                                              fillcolor='rgba(149, 225, 211, 0.2)'))
                fig_alt.update_layout(title='飞行高度', template='plotly_dark', height=280,
                                       margin=dict(l=40, r=20, t=40, b=40))
                st.plotly_chart(fig_alt, use_container_width=True)

            with st.expander("📋 查看原始数据"):
                st.dataframe(df, use_container_width=True, hide_index=True)

            col_log1, col_log2 = st.columns(2)

            with col_log1:
                st.subheader("📝 状态日志")
                logs = uav.get_status_log(last_n=10)
                if logs:
                    st.code("\n".join(logs[-10:]), language="log", height=200)
                else:
                    st.info("暂无日志")

            with col_log2:
                st.subheader("🚨 告警记录")
                alerts = ground_station.get_alerts(last_n=10)
                if alerts:
                    st.code("\n".join(alerts[-10:]), language="log", height=200)
                else:
                    st.success("✅ 无告警记录")
        else:
            st.info("💡 暂无数据，请点击「启动模拟」开始采集数据")

    with tab_map:
        st.subheader("🗺️ 飞行监控地图")

        df = uav.get_history_dataframe(last_n=50)
        route_waypoints = uav.get_route_waypoints()
        obstacles = uav.get_obstacles()

        if not df.empty and len(df) >= 2:
            latest_lat = df['纬度'].iloc[-1]
            latest_lon = df['经度'].iloc[-1]
            center_lat = df['纬度'].mean()
            center_lon = df['经度'].mean()

            route_coords = [[row['纬度'], row['经度']] for _, row in df.iterrows()]
            planned_route_coords = [[wp[0], wp[1]] for wp in route_waypoints]
            obstacles_json = json.dumps(obstacles)

            map_html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8">
                <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
                <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
                <style>
                    #map {{ width: 100%; height: 550px; margin: 0; }}
                    .info {{ position: absolute; top: 10px; right: 10px; z-index: 1000; background: rgba(255,255,255,0.9); padding: 10px; border-radius: 5px; font-size: 12px; }}
                    .legend {{ position: absolute; bottom: 10px; left: 10px; z-index: 1000; background: rgba(255,255,255,0.9); padding: 8px; border-radius: 5px; font-size: 11px; }}
                </style>
            </head>
            <body>
                <div id="map"></div>
                <div class="info">
                    <div><strong>📍 无人机状态</strong></div>
                    <div>状态: <span style="color: {'green' if {status['is_offline'] == False} else 'red'}">{'在线' if {status['is_offline'] == False} else '离线'}</span></div>
                    <div>航点: {len(route_waypoints)} 个</div>
                    <div>障碍物: {len(obstacles)} 个</div>
                </div>
                <div class="legend">
                    <div><b>图例</b></div>
                    <div>🟢 起点</div>
                    <div>🔴 当前位置</div>
                    <div>🔵 实际轨迹</div>
                    <div>🟠 规划航线</div>
                    <div>🟡 障碍物</div>
                </div>
                <script>
                    var map = L.map('map').setView([{center_lat}, {center_lon}], 16);
                    L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                        attribution: '© OpenStreetMap', maxZoom: 19
                    }}).addTo(map);
                    L.control.scale({{imperial: false}}).addTo(map);

                    var plannedRoute = {json.dumps(planned_route_coords)};
                    if (plannedRoute.length > 1) {{
                        L.polyline(plannedRoute, {{color: '#FFA500', weight: 3, dashArray: '10, 5', opacity: 0.8}}).addTo(map);
                        plannedRoute.forEach(function(wp, index) {{
                            L.circleMarker(wp, {{radius: 6, color: '#FFA500', fillColor: '#FFA500', fillOpacity: 1}})
                                .addTo(map).bindPopup('航点 ' + (index + 1));
                        }});
                    }}

                    var obstacles = {obstacles_json};
                    obstacles.forEach(function(obs) {{
                        L.polygon(obs.coords, {{
                            color: '#FFD700',
                            fillColor: '#FFD700',
                            fillOpacity: 0.3,
                            weight: 2
                        }}).addTo(map).bindPopup('障碍物: ' + obs.name);
                    }});

                    var flightPath = {json.dumps(route_coords)};
                    if (flightPath.length > 1) {{
                        L.polyline(flightPath, {{color: '#3366FF', weight: 3, opacity: 0.7}}).addTo(map);
                        L.circleMarker(flightPath[0], {{radius: 6, color: '#28a745', fillColor: '#28a745', fillOpacity: 1}})
                            .addTo(map).bindPopup('起点');
                    }}

                    L.circleMarker([{latest_lat}, {latest_lon}], {{
                        radius: 10, 
                        color: '#FF6B6B', 
                        fillColor: '#FF6B6B', 
                        fillOpacity: 1,
                        weight: 2
                    }}).addTo(map).bindPopup('<b style="color:red;">当前位置</b>');

                    map.fitBounds(L.latLngBounds(flightPath), {{padding: [30, 30]}});
                </script>
            </body>
            </html>
            """
            components.html(map_html, height=580, scrolling=False)
        else:
            map_html = f"""
            <!DOCTYPE html>
            <html>
            <head>
                <meta charset="utf-8">
                <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
                <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
                <style>html, body, #map {{ width: 100%; height: 550px; margin: 0; }}</style>
            </head>
            <body>
                <div id="map"></div>
                <script>
                    var map = L.map('map').setView([{uav.base_lat}, {uav.base_lon}], 15);
                    L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                        attribution: '© OpenStreetMap', maxZoom: 19
                    }}).addTo(map);
                    L.control.scale({{imperial: false}}).addTo(map);
                    
                    var routeCoords = {json.dumps(route_waypoints)};
                    if (routeCoords.length > 1) {{
                        L.polyline(routeCoords, {{color: '#FFA500', weight: 3, dashArray: '10, 5'}}).addTo(map);
                    }}
                    
                    var obstacles = {json.dumps(obstacles)};
                    obstacles.forEach(function(obs) {{
                        L.polygon(obs.coords, {{color: '#FFD700', fillColor: '#FFD700', fillOpacity: 0.3}}).addTo(map);
                    }});
                </script>
            </body>
            </html>
            """
            components.html(map_html, height=580, scrolling=False)
            st.info("💡 启动模拟后将在地图上显示飞行轨迹")

    if uav.is_running:
        time.sleep(0.8)
        st.rerun()
    elif st.session_state.get('was_offline', False):
        elapsed = (datetime.now() - uav.last_heartbeat_time).total_seconds() if uav.last_heartbeat_time else 0
        if elapsed >= 5:
            uav.start()
            st.session_state.was_offline = False
            st.rerun()
        else:
            time.sleep(0.5)
            st.rerun()