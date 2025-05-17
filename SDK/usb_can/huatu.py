import pandas as pd
import matplotlib.pyplot as plt

# df = pd.read_csv('/home/sdu/cmd_data_leggedcontroller.csv', header=None)
df = pd.read_csv('build/data1.csv', header=None)

x = df.iloc[:, 0].values  # 第一列作为横轴
y1 = df.iloc[:, 23].values
y2 = df.iloc[:, 24].values
# y3 = df.iloc[:, 46].values

# 创建图形
plt.figure(figsize=(10, 6))

# 绘制图表，并为每个数据点添加标记
plt.plot(x, y1, color='r', marker='o', markersize=5, label='y1')  # 添加标记
plt.plot(x, y2, color='g', marker='s', markersize=5, label='y2')  # 添加标记
# plt.plot(x, y3, color='b', marker='^', markersize=5, label='y3')  # 添加标记（如果需要）

# 显示网格
plt.grid(True)

# 显示图例
plt.legend()

# 展示图形
plt.show()
