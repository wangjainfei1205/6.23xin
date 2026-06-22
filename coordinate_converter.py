import math
import pandas as pd
from typing import Tuple, List, Union

x_pi = 3.14159265358979324 * 3000.0 / 180.0
pi = 3.1415926535897932384626
a = 6378245.0
ee = 0.00669342162296594323

def out_of_china(lng: float, lat: float) -> bool:
    if lng < 72.004 or lng > 137.8347:
        return True
    if lat < 0.8293 or lat > 55.8271:
        return True
    return False

def transform_lat(lng: float, lat: float) -> float:
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
          0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 *
            math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * pi) + 40.0 *
            math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 *
            math.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret

def transform_lng(lng: float, lat: float) -> float:
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
          0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 *
            math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * pi) + 40.0 *
            math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 *
            math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
    return ret

def wgs84_to_gcj02(lng: float, lat: float) -> Tuple[float, float]:
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
    mglat = lat + dlat
    mglng = lng + dlng
    return mglng, mglat

def gcj02_to_wgs84(lng: float, lat: float) -> Tuple[float, float]:
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
    mglat = lat * 2 - (lat + dlat)
    mglng = lng * 2 - (lng + dlng)
    return mglng, mglat

def gcj02_to_bd09(lng: float, lat: float) -> Tuple[float, float]:
    z = math.sqrt(lng * lng + lat * lat) + 0.00002 * math.sin(lat * x_pi)
    theta = math.atan2(lat, lng) + 0.000003 * math.cos(lng * x_pi)
    bd_lng = z * math.cos(theta) + 0.0065
    bd_lat = z * math.sin(theta) + 0.006
    return bd_lng, bd_lat

def bd09_to_gcj02(lng: float, lat: float) -> Tuple[float, float]:
    x = lng - 0.0065
    y = lat - 0.006
    z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * x_pi)
    theta = math.atan2(y, x) - 0.000003 * math.cos(x * x_pi)
    gcj_lng = z * math.cos(theta)
    gcj_lat = z * math.sin(theta)
    return gcj_lng, gcj_lat

def wgs84_to_bd09(lng: float, lat: float) -> Tuple[float, float]:
    gcj_lng, gcj_lat = wgs84_to_gcj02(lng, lat)
    return gcj02_to_bd09(gcj_lng, gcj_lat)

def bd09_to_wgs84(lng: float, lat: float) -> Tuple[float, float]:
    gcj_lng, gcj_lat = bd09_to_gcj02(lng, lat)
    return gcj02_to_wgs84(gcj_lng, gcj_lat)

class CoordinateConverter:
    def __init__(self):
        self.history: List[dict] = []
    
    def convert(self, lng: float, lat: float, from_sys: str, to_sys: str) -> Tuple[float, float]:
        result = (lng, lat)
        
        if from_sys == 'WGS-84' and to_sys == 'GCJ-02':
            result = wgs84_to_gcj02(lng, lat)
        elif from_sys == 'WGS-84' and to_sys == 'BD-09':
            result = wgs84_to_bd09(lng, lat)
        elif from_sys == 'GCJ-02' and to_sys == 'WGS-84':
            result = gcj02_to_wgs84(lng, lat)
        elif from_sys == 'GCJ-02' and to_sys == 'BD-09':
            result = gcj02_to_bd09(lng, lat)
        elif from_sys == 'BD-09' and to_sys == 'WGS-84':
            result = bd09_to_wgs84(lng, lat)
        elif from_sys == 'BD-09' and to_sys == 'GCJ-02':
            result = bd09_to_gcj02(lng, lat)
        
        self.history.append({
            'input_lng': lng,
            'input_lat': lat,
            'from_sys': from_sys,
            'to_sys': to_sys,
            'output_lng': result[0],
            'output_lat': result[1]
        })
        
        return result
    
    def batch_convert(self, coords: List[Tuple[float, float]], from_sys: str, to_sys: str) -> List[Tuple[float, float]]:
        return [self.convert(lng, lat, from_sys, to_sys) for lng, lat in coords]
    
    def get_history_dataframe(self) -> pd.DataFrame:
        if not self.history:
            return pd.DataFrame()
        return pd.DataFrame(self.history)
    
    def clear_history(self):
        self.history = []
    
    def export_history(self, format: str = 'csv') -> str:
        df = self.get_history_dataframe()
        if format == 'csv':
            return df.to_csv(index=False)
        elif format == 'json':
            return df.to_json(orient='records', force_ascii=False)
        return ""