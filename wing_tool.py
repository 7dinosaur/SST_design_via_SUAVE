import math
import numpy as np
import pandas as pd
from scipy.special import comb

import SUAVE
assert SUAVE.__version__=='2.5.2', 'These tutorials only work with the SUAVE 2.5.2 release'
from SUAVE.Core import Units, Data
from SUAVE.Plots.Performance.Mission_Plots import *
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import segment_properties

def cst_rec(para, N1=0.5, N2=1, n_points=60, psi_end=1.0):
    ##从参数列表提取参数赋值变量
    order = int((len(para) - 8)/2)
    coeffs = np.array([para[1:order+2],para[order+2:(order+1)*2+1]])
    dy_upper = para[-2]; dy_lower = para[-1]

    psi = np.linspace(0, psi_end, n_points)
    coeffs_upper = coeffs[0]
    coeffs_lower = coeffs[1]
    
    # 生成Bernstein基函数
    B = np.zeros((n_points, order+1))
    for i in range(order+1):
        B[:, i] = comb(order, i) * (psi**i) * (1 - psi)**(order-i)
    
    # 计算上下表面坐标
    y_upper = (psi**N1 * (1 - psi)**N2) * (B @ coeffs_upper) + psi*dy_upper
    y_lower = (psi**N1 * (1 - psi)**N2) * (B @ coeffs_lower) + psi*dy_lower

    coord_u = np.array([psi, y_upper])
    coord_l = np.array([psi, y_lower])

    return coord_u, coord_l

def wing_from_para(para_csv: str = "6.64_simple.csv"):
    para = pd.read_csv(para_csv).to_numpy()
    wing = SUAVE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    n_sec = para.shape[0]; cst_order = int((para.shape[1] - 6)/2 - 1) ##6个位置尺寸参数，其余为cst

    chord_root = (para[0, -4] - para[0, -5]) * Units.meters; chord_tip = (para[-1, -4] - para[-1, -5]) * Units.meters
    span = 2*(para[-1, 0] - para[0, 0]) * Units.meters

    wing.total_length            = chord_root
    wing.spans.projected         = span
    wing.chords.root             = chord_root
    wing.chords.tip              = chord_tip
    wing.areas.reference         = 300.6 * Units.meters**2
    # wing.sweeps.quarter_chord    = 33. * Units.degrees
    # wing.twists.root             = 0.0 * Units.degrees
    # wing.twists.tip              = 0.0 * Units.degrees
    # wing.dihedral                = 0 * Units.degrees
    wing.origin                  = [[0.,0.,0]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.dynamic_pressure_ratio  = 1.0

    dih_list = []
    
    area = 0
    for i in range(n_sec-1):
        segment = SUAVE.Components.Wings.Segment()
        segment.tag = 'section_'+str(i+1)
        segment.percent_span_location = (para[i,0]/para[-1,0])
        segment.twist = 0. * Units.deg                      ##无扭转角
        root = para[i, -4] - para[i, -5]
        tip = para[i+1, -4] - para[i+1, -5]
        segment.root_chord_percent = root/wing.chords.root
        dih_ag = float(math.atan((para[i+1,-3]-para[i,-3])/(para[i+1,0]-para[i,0]))*180/math.pi) 
        dih_list.append(dih_ag)
        segment.dihedral_outboard = dih_ag * Units.deg  
        # print(para[i,0], para[-1,0], dih_ag, segment.percent_span_location)
        x00, x01, x10, x11 = para[i, -5], para[i, -4], para[i+1, -5], para[i+1, -4]
        x0 = x00 + 0.25 * (x01 - x00); x1 = x10 + 0.25 * (x11 - x10)
        y0, y1, z0, z1 = para[i, 0], para[i+1, 0], para[i, -3], para[i+1, -3]
        segment.sweeps.quarter_chord = (90 - math.atan(math.sqrt((y1 - y0)**2 + (z1 - z0)**2)
                                                       /(x1 - x0))*180/math.pi) * Units.degrees
        print(segment.sweeps.quarter_chord/Units.degrees)
        area += 0.5 * (root+tip) * (y1-y0)
        segment.airfoil_type = "file"
        airfoil_data1 = Data()
        airfoil_data1.tag = 'airfoil' # 这个 tag 很重要，它作为 key 用于后续访问
        filename = f"airfoil\\{i+1}.dat"
        coord_u, coord_l = cst_rec(para[i], n_points=100)
        thick = (coord_u[1, :] - coord_l[1, :]).max()
        segment.thickness_to_chord = thick   ##计算厚度弦长比
        if i == 0:
            wing.thickness_to_chord = thick
        coords = np.concatenate([coord_l.T[::-1], coord_u.T], axis=0)
        np.savetxt(filename, coords, '%.5f')
        airfoil_data1.coordinate_file = filename # 重要的坐标文件路径
        segment.append_airfoil(airfoil_data1)  # 添加 airfoil 数据到 segment
        wing.Segments.append(segment)    

    wing.aspect_ratio            = (0.5*span)**2/area
    # wing.aspect_ratio *= 4.5
    print(wing.aspect_ratio)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = (para[-1, -4] - para[-1, -5])/(para[0, -4] - para[0, -5])
    segment.dihedral_outboard     = 20 * Units.degrees
    # segment.sweeps.quarter_chord  = sweep_list[12] * Units.degrees
    segment.thickness_to_chord    = 0.04
    wing.Segments.append(segment)  
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)

    return wing