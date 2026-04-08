import folium
from global_land_mask import globe
import json
import numpy as np
from numpy import ndarray
import math
import matplotlib.pyplot as plt
from requests import head
from scipy import interpolate, optimize
from scipy.optimize import differential_evolution
from geopy.distance import great_circle

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
        points = self.rearrange_points(target_num=1000)  # 重新排列点以获得更平滑的路径
        points = points[:, [1, 0]]
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
    
    def flight_mission_set(self, fig=False):
        points = self.rearrange_points(target_num=103)[:, [1, 0]]  # (lon, lat)
        n = len(points)
        flight_mission = np.zeros((len(points), 4))  # [range, is_land]
        range_accum = 0

        ## 工具函数，两点之间插入等距点
        def interpolate_points(p1, p2, step_km=1.0):
            dist_km = great_circle(p1, p2).km
            num = max(2, int(dist_km / step_km) + 1)
            lats = np.linspace(p1[0], p2[0], num)
            lons = np.linspace(p1[1], p2[1], num)
            return list(zip(lats, lons)), dist_km
        
        def generate_cross_points(center_lat, center_lon, heading, half_width_km=50):
            earth_radius_km = R_earth / 1000
            d = half_width_km / earth_radius_km
            heading_rad = np.radians(heading)

            lat_rad = np.radians(center_lat)
            lon_rad = np.radians(center_lon)

            ## 左边延伸角度
            left_brng = heading_rad - np.pi/2
            ## 用球面公式计算延伸指定距离后的经纬度
            lat_left = np.arcsin(np.sin(lat_rad)*np.cos(d) + np.cos(lat_rad)*np.sin(d)*np.cos(left_brng))
            lon_left = lon_rad + np.arctan2(np.sin(left_brng)*np.sin(d)*np.cos(lat_rad), np.cos(d)-np.sin(lat_rad)*np.sin(lat_left))
            left = (np.degrees(lat_left), np.degrees(lon_left))

            right_brng = heading_rad + np.pi/2
            lat_right = np.arcsin(np.sin(lat_rad)*np.cos(d) + np.cos(lat_rad)*np.sin(d)*np.cos(right_brng))
            lon_right = lon_rad + np.arctan2(np.sin(right_brng)*np.sin(d)*np.cos(lat_rad), np.cos(d)-np.sin(lat_rad)*np.sin(lat_right))
            right = (np.degrees(lat_right), np.degrees(lon_right))
            return left, right

        for i in range(len(points)-1):
            pA = points[i]
            flight_mission[i, 2] = pA[0]; flight_mission[i, 3] = pA[1]
            pB = points[i+1]
            this_points, _ = interpolate_points(pA, pB)
            check_points = []

            for j in range(len(this_points)-1):
                p1 = this_points[j]
                p2 = this_points[j+1]

                dy = p2[0] - p1[0]
                dx = p2[1] - p1[1]
                heading = np.degrees(np.arctan2(dx, dy))

                left, right = generate_cross_points(p1[0], p1[1], heading)
                check_points.append(left)
                check_points.append(right)

            lats = [p[0] for p in check_points]
            lons = [p[1] - 360 if p[1] > 180 else p[1] + 360 if p[1] < -180 else p[1] for p in check_points]
            land_flags = globe.is_land(lats, lons)
            if np.any(land_flags):
                flight_mission[i+1, 1] = 1

            range_accum += self.latlon_distance(pA[0], pA[1], pB[0], pB[1])
            flight_mission[i+1, 0] = range_accum

        self.flight_mission = flight_mission
        ## 计算水面航程的占比
        total_range = flight_mission[-1, 0]
        seg_dists = np.diff(flight_mission[:, 0])        # 每一段距离
        is_water = (flight_mission[1:, 1] == 0)         # 每一段是否水面
        water_range = np.sum(seg_dists[is_water])       # 水面总距离

        check_after_range = 2000 * 1000 #1000km后不允许有陆地
        check_last_range = total_range - check_after_range #最后1000km允许有陆地
        after_points = flight_mission[(flight_mission[:, 0] >= check_after_range) & (flight_mission[:, 0] <= check_last_range)]
        
        land_flags = after_points[:, 1]
        no_land_after = True
        for t in range(2, len(land_flags)):
            if after_points[t, 1] * after_points[t-1, 1] * after_points[t-2, 1] == 1:
                no_land_after = False
                break

        if fig:
            plt.figure(figsize=(18, 4))
            plt.plot(flight_mission[:, 0]/1000, flight_mission[:, 1])
            plt.xlabel("Range (km)")
            plt.ticklabel_format(style='plain', axis='x')

        return water_range / total_range if total_range != 0 else 0.0, no_land_after

    def rearrange_points(self, target_num=100):
        points = self.points[:, [1, 0]]
        for point in points:
            if point[1] < 0:
                    point[1] += 360  # 转换为0~360度经度

        x_col = points[:, 0]
        _, unique_idx = np.unique(x_col, return_index=True)
        unique_idx = np.sort(unique_idx)  # 保持原始顺序不变
        points = points[unique_idx]  # 去重后的点

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
        x_new = interpolate.interp1d(t, x, kind=2)(t_new)
        y_new = interpolate.interp1d(t, y, kind=2)(t_new)

        # 合并成最终点集
        rearranged = np.column_stack([x_new, y_new])
        # 大于180度的经度转换回负值
        for point in rearranged:
            if point[1] > 180:
                point[1] -= 360
        return rearranged[:, [1, 0]]  # 转回 [lon, lat]
    
def obj(var, base_route):
    n_change = len(var)
    points = base_route.rearrange_points(target_num=n_change+2)
    points[1:-1, 1] += var  # 调整纬度
    new_route = RoutePlanner(points=points)
    water_percent, no_land = new_route.flight_mission_set()
    range_total = new_route.route_total_distance() / 10000000
    print("当前水面占比", water_percent, "当前航程", f"{range_total*10000:.1f}km")
    if not no_land:  # 如果陆地段超过3段，强烈惩罚
        range_total += 2  # 大幅降低水面占比，迫使优化远离过多陆地段
    return 0.9*range_total - 0.1*water_percent   # 返回水面航程占比

def multi_draw(route_list: list[RoutePlanner], output_html: str):
        map = folium.Map(location=[30, 160], tiles="OpenStreetMap", zoom_start=3)
        for route in route_list:
            points = route.rearrange_points(target_num=1000)  # 重新排列点以获得更平滑的路径
            points = points[:, [1, 0]]
            for point in points:
                if point[1] < 0:
                        point[1] += 360  # 转换为0~360度经度
            folium.PolyLine(points, color="blue", weight=3).add_to(map)
            folium.Marker([points[0][0], points[0][1]], tooltip="Start").add_to(map)
            folium.Marker([points[-1][0], points[-1][1]], tooltip="End").add_to(map)
            route.flight_mission_set()
            flight_mission = route.flight_mission
            for da in flight_mission:
                if da[1] == 1:
                    lat = da[2]; lon = da[3]
                    if lon < 0:
                        lon += 360
                    folium.CircleMarker(location=[lat, lon],radius=6,color='red',fill=True,fill_color='red').add_to(map)

        map.save(output_html)

if __name__ == "__main__":
    route = RoutePlanner("beijing-to-las.geojson")
    n_change = 8
#     x0 = np.random.uniform(-20, 20, size=n_change)  # 初始猜测
#     bounds = [(-20, 20)] * n_change  # 每个变量的边界
#     result = differential_evolution(
#     obj, 
#     bounds,
#     args=(route,),
#     maxiter=100,
#     tol=1e-6,
#     workers=1,
#     disp=True
# )
#     print("优化结果：", result)
#     print("优化后的水面航程占比：", obj(result.x, route))

#     # x = np.array([-8.214e+00, -8.485e+00, -7.185e+00, -7.269e+00, -7.561e+00, -6.509e+00, -5.505e+00, -6.275e+00])
    points = route.rearrange_points(target_num=n_change+2)
#     print(result.x)
    opt_x = np.loadtxt("opt.dat")
    np.savetxt("opt.dat", opt_x, '%.5f')
    points[1:-1, 1] += opt_x  # 调整纬度
    new_route = RoutePlanner(points=points)
    new_route.flight_mission_set(fig=True)
    new_route.draw_map("optimized_route.html")
    
    # print(route.route_total_distance())
    new_route.flight_mission_set(fig=True)
    print("大圆航线航程：", route.route_total_distance(), "优化航线航程：", new_route.route_total_distance())
    multi_draw([route, new_route], "optimized_route.html")
    plt.show()