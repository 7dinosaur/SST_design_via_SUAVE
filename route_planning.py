from hmac import new

import folium
from global_land_mask import globe
import json
import numpy as np
from numpy import ndarray
import math
import matplotlib.pyplot as plt
from scipy import interpolate

R_earth = 6371000 # 地球半径（米）

class RoutePlanner:
    def __init__(self, geojson=None, points=None):
        if geojson is not None:
            self.geojson = geojson
            self.points = self.geojson_to_points()
        elif points is not None:
            self.points = points
        else:
            raise ValueError("必须提供geojson文件路径或点坐标数组")

    def geojson_to_points(self) -> ndarray:
        # 1. 读取并解析GeoJSON文件
        with open(self.geojson, 'r', encoding='utf-8') as f:
            geojson_data = json.load(f)
        # 2. 提取 MultiLineString 坐标
        coordinates = geojson_data["geometry"]["coordinates"]
        
        all_points = []
        for line in coordinates:
            for point in line:
                all_points.append(point)  # point 是 [lon, lat]
        
        return np.array(all_points)
    
    def draw_map(self, output_html):
        points = self.points[:, [1, 0]]
        for point in points:
            if point[1] < 0:
                    point[1] += 360  # 转换为0~360度经度
        map = folium.Map(location=[30, 160], tiles="OpenStreetMap", zoom_start=3)
        folium.PolyLine(points, color="blue", weight=3).add_to(map)

        folium.Marker([points[0][0], points[0][1]], tooltip="Start").add_to(map)
        folium.Marker([points[-1][0], points[-1][1]], tooltip="End").add_to(map)

        map.save(output_html)

    def latlon_distance(self, lat1, lon1, lat2, lon2):
        # 转为弧度
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)
        
        # Haversine 公式
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        distance = R_earth * c  
        return distance

    def route_total_distance(self):
        points = self.points
        total = 0
        for i in range(len(points)-1):
            lon1, lat1 = points[i]
            lon2, lat2 = points[i+1]
            total += self.latlon_distance(lat1, lon1, lat2, lon2)
        
        return total
    
    def flight_mission_set(self):
        flight_mission = np.zeros((len(self.points), 2))  # [range, is_land]
        range_accum = 0
        last_coord = np.array([0.0, 0.0])
        for i, (lon, lat) in enumerate(self.points):
            if i == 0:
                last_coord = np.array([lon, lat])
            range_accum += self.latlon_distance(last_coord[1], last_coord[0], lat, lon)
            last_coord = np.array([lon, lat])
            flight_mission[i, 0] = range_accum
            flight_mission[i, 1] = globe.is_land(lat, lon)

        plt.figure(figsize=(15, 4))
        plt.plot(flight_mission[:, 0]/1000, flight_mission[:, 1])
        plt.xlabel("Range (km)")
        plt.ticklabel_format(style='plain', axis='x')

    def rearrange_points(self, target_num=100):
        points = self.points[:, [1, 0]]
        for point in points:
            if point[1] < 0:
                    point[1] += 360  # 转换为0~360度经度
        # 如果点太少，直接返回（避免插值出错）
        if len(points) <= 2:
            return points
        
        # 拆分 x 和 y
        x = points[:, 0]
        y = points[:, 1]

        # 生成路径长度（用于均匀插值）
        t = np.zeros_like(x)
        t[1:] = np.sqrt((x[1:] - x[:-1])**2 + (y[1:] - y[:-1])**2)
        t = np.cumsum(t)
        t /= t[-1]  # 归一化到 0~1

        # 生成新的均匀插值点
        t_new = np.linspace(0, 1, target_num)
        x_new = interpolate.interp1d(t, x, kind='linear')(t_new)
        y_new = interpolate.interp1d(t, y, kind='linear')(t_new)

        # 合并成最终点集
        rearranged = np.column_stack([x_new, y_new])
        # 大于180度的经度转换回负值
        for point in rearranged:
            if point[1] > 180:
                point[1] -= 360
        return rearranged[:, [1, 0]]  # 转回 [lon, lat]

if __name__ == "__main__":
    route = RoutePlanner("beijing-to-las.geojson")
    # route.draw_map("route_map.html")
    print("总距离（米）：", route.route_total_distance())
    new_points = route.rearrange_points(target_num=11)
    new_route = RoutePlanner(points=new_points)
    new_route.draw_map("rearranged_route_map.html")
    old_points = route.points
    plt.figure(figsize=(8, 6))
    plt.plot(old_points[:, 0], old_points[:, 1], label='Original Points')
    plt.plot(new_points[:, 0], new_points[:, 1], label='Rearranged Points')
    print("总距离:(米)", new_route.route_total_distance())
    route.flight_mission_set()
    plt.show()