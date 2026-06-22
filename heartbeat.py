import time
import threading
import pandas as pd
from datetime import datetime
from dataclasses import dataclass
from typing import List, Optional, Tuple
import random
import math

@dataclass
class HeartbeatPacket:
    seq: int
    timestamp: datetime
    battery: float
    altitude: float
    speed: float
    pitch: float
    roll: float
    yaw: float
    latitude: float
    longitude: float

@dataclass
class Waypoint:
    lat: float
    lng: float
    altitude: float = 100.0
    speed: float = 15.0

class UAVSimulator:
    def __init__(self, interval: float = 1.0, offline_threshold: float = 3.0):
        self.interval = interval
        self.offline_threshold = offline_threshold
        self.sequence = 0
        self.is_running = False
        self.heartbeat_history: List[HeartbeatPacket] = []
        self.last_heartbeat_time: Optional[datetime] = None
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        
        self.base_lat = 32.234104
        self.base_lon = 118.749421
        
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = False
        
        self._generate_default_route()
    
    def _generate_default_route(self):
        self.waypoints = [
            Waypoint(lat=32.234104, lng=118.749421, altitude=100.0),
            Waypoint(lat=32.234500, lng=118.750000, altitude=100.0),
            Waypoint(lat=32.235000, lng=118.749800, altitude=120.0),
            Waypoint(lat=32.235200, lng=118.749000, altitude=120.0),
            Waypoint(lat=32.234800, lng=118.748500, altitude=100.0),
            Waypoint(lat=32.234300, lng=118.748800, altitude=100.0),
            Waypoint(lat=32.234104, lng=118.749421, altitude=100.0),
        ]
    
    def set_route(self, waypoints: List[Tuple[float, float]]):
        self.waypoints = [Waypoint(lat=lat, lng=lng) for lat, lng in waypoints]
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = True
    
    def set_circular_route(self, center_lat: float, center_lng: float, radius_km: float, num_points: int = 8):
        waypoints = []
        for i in range(num_points):
            angle = (i / num_points) * 2 * math.pi
            dx = radius_km * math.cos(angle) / 111.0
            dy = radius_km * math.sin(angle) / 111.0
            waypoints.append(Waypoint(lat=center_lat + dx, lng=center_lng + dy))
        waypoints.append(waypoints[0])
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = True
    
    def set_rectangular_route(self, start_lat: float, start_lng: float, width_km: float, height_km: float):
        waypoints = [
            Waypoint(lat=start_lat, lng=start_lng),
            Waypoint(lat=start_lat, lng=start_lng + width_km / 111.0),
            Waypoint(lat=start_lat + height_km / 111.0, lng=start_lng + width_km / 111.0),
            Waypoint(lat=start_lat + height_km / 111.0, lng=start_lng),
            Waypoint(lat=start_lat, lng=start_lng),
        ]
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.progress_to_next_waypoint = 0.0
        self.flying_route = True
    
    def _get_current_position(self) -> Tuple[float, float, float, float]:
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
        
        self.progress_to_next_waypoint += 0.02
        
        if self.progress_to_next_waypoint >= 1.0:
            self.progress_to_next_waypoint = 0.0
            self.current_waypoint_index += 1
        
        noise = 0.00005
        return (
            lat + random.uniform(-noise, noise),
            lng + random.uniform(-noise, noise),
            alt + random.uniform(-2, 2),
            spd + random.uniform(-1, 1)
        )
    
    def _calculate_yaw(self, lat1: float, lng1: float, lat2: float, lng2: float) -> float:
        d_lng = lng2 - lng1
        yaw = math.atan2(d_lng, lat2 - lat1) * 180 / math.pi
        return (yaw + 360) % 360
    
    def _generate_packet(self) -> HeartbeatPacket:
        self.sequence += 1
        
        lat, lng, alt, spd = self._get_current_position()
        
        if self.flying_route and len(self.waypoints) > 1:
            if self.current_waypoint_index < len(self.waypoints) - 1:
                next_wp = self.waypoints[self.current_waypoint_index + 1]
                yaw = self._calculate_yaw(lat, lng, next_wp.lat, next_wp.lng)
            else:
                yaw = random.uniform(0, 360)
        else:
            yaw = random.uniform(0, 360)
        
        packet = HeartbeatPacket(
            seq=self.sequence,
            timestamp=datetime.now(),
            battery=max(20, 100 - self.sequence * 0.02 + random.uniform(-0.5, 0.5)),
            altitude=max(50, alt),
            speed=max(5, spd),
            pitch=random.uniform(-2, 2),
            roll=random.uniform(-2, 2),
            yaw=yaw,
            latitude=lat,
            longitude=lng
        )
        return packet
    
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
    
    def check_offline(self) -> bool:
        if self.last_heartbeat_time is None:
            return True
        elapsed = (datetime.now() - self.last_heartbeat_time).total_seconds()
        return elapsed > self.offline_threshold
    
    def get_offline_duration(self) -> float:
        if self.last_heartbeat_time is None:
            return 0.0
        return (datetime.now() - self.last_heartbeat_time).total_seconds()
    
    def get_history_dataframe(self, last_n: int = 100) -> pd.DataFrame:
        with self._lock:
            if not self.heartbeat_history:
                return pd.DataFrame()
            data = self.heartbeat_history[-last_n:]
            return pd.DataFrame([
                {
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
                }
                for p in data
            ])
    
    def get_latest_packet(self) -> Optional[HeartbeatPacket]:
        with self._lock:
            if self.heartbeat_history:
                return self.heartbeat_history[-1]
        return None
    
    def get_status_log(self, last_n: int = 50) -> List[str]:
        with self._lock:
            if not self.heartbeat_history:
                return []
            data = self.heartbeat_history[-last_n:]
            logs = []
            for p in data:
                status = "正常" if p.battery > 30 else "低电量警告"
                logs.append(f"[{p.timestamp.strftime('%H:%M:%S')}] 心跳#{p.seq} | 状态: {status} | 电量: {p.battery:.1f}% | 高度: {p.altitude:.1f}m")
            return logs
    
    def get_route_waypoints(self) -> List[Tuple[float, float]]:
        return [(wp.lat, wp.lng) for wp in self.waypoints]

class GroundStation:
    def __init__(self, uav: UAVSimulator):
        self.uav = uav
        self.alert_history: List[str] = []
        
    def monitor(self) -> dict:
        is_offline = self.uav.check_offline()
        latest = self.uav.get_latest_packet()
        
        result = {
            'status': '离线' if is_offline else '在线',
            'is_offline': is_offline,
            'offline_duration': self.uav.get_offline_duration() if is_offline else 0,
            'latest_packet': latest,
            'sequence': latest.seq if latest else 0,
            'last_time': latest.timestamp if latest else None
        }
        
        if is_offline:
            alert_msg = f"[{datetime.now().strftime('%H:%M:%S')}] 警告: 无人机已离线 {result['offline_duration']:.1f} 秒!"
            self.alert_history.append(alert_msg)
            if len(self.alert_history) > 100:
                self.alert_history = self.alert_history[-50:]
        
        return result
    
    def get_alerts(self, last_n: int = 20) -> List[str]:
        return self.alert_history[-last_n:]