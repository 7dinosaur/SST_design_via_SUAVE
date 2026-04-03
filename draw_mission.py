import matplotlib.pyplot as plt
import numpy as np

# ------------------ 样式设置（关键！）
plt.style.use('default')
plt.rcParams['font.size'] = 12
plt.rcParams['axes.linewidth'] = 1.2
plt.rcParams['lines.linewidth'] = 2
plt.rcParams['lines.markersize'] = 5
plt.rcParams['figure.dpi'] = 150

plt.rcParams['font.sans-serif'] = ['SimHei']  # 用黑体显示中文
plt.rcParams['axes.unicode_minus'] = False    # 正常显示负号
plt.rcParams['font.family'] = 'sans-serif'

# ------------------ 示例数据
mission = np.loadtxt("flight_mission.dat")
mission_boom = np.loadtxt("flight_mission_boom.dat")
x1 = mission[:, 0] * (11000/25)
y1 = mission[:, 1]
x2 = mission_boom[:, 0] * (11000/27)
y2 = mission_boom[:, 1]

# ------------------ 画图
fig, ax = plt.subplots(figsize=(6,4)) # 细长图最适合汇报

# ax.plot(x1, y1, color='#009192', label='无约束典型任务剖面')
ax.plot(x2, y2, color="#FC6E6E", label='考虑声爆约束的任务剖面')

# 网格、标签、图例
ax.grid(True, alpha=0.2, linestyle='--')
ax.set_xlabel('Range (km)')
ax.set_ylabel('Altitude (km)')
# ax.legend(loc="upper right",frameon=False)
plt.xticks(np.arange(0, 12000, 1000))
plt.yticks(np.arange(0, 20, 2))
plt.tight_layout()

plt.savefig('aoa_plot.png', dpi=300, bbox_inches='tight')
plt.show()