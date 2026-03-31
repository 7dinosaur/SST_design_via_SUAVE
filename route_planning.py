import folium
from global_land_mask import globe
import json
import numpy as np
from numpy import ndarray
import math
import matplotlib.pyplot as plt

R_earth = 6371000 # 地球半径（米）

class RoutePlanner:
    def __init__(self, geojson):
        self.geojson = geojson
        self.points = self.geojson_to_points()

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

        folium.Marker([points[0][1], points[0][0]], tooltip="Start").add_to(map)
        folium.Marker([points[-1][1], points[-1][0]], tooltip="End").add_to(map)

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

        plt.plot(flight_mission[:, 0], flight_mission[:, 1])
        plt.xlabel("Range (m)")

if __name__ == "__main__":
    route = RoutePlanner("beijing-to-las.geojson")
    route.draw_map("route_map.html")
    print("总距离（米）：", route.route_total_distance())
    route.flight_mission_set()
    plt.show()