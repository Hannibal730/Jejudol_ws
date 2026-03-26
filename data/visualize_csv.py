
import pandas as pd
import matplotlib.pyplot as plt

# Define file paths
# gps_data_path = f'{processed_dir}/이전대회 GPS 데이터 (기본+평행주차)_direction.csv'
gps_data_path = '/home/hannibal/Jejudol_ws/data/rosbag2_2026_03_26-17_52_41.csv'

# Load the data
gps_data = pd.read_csv(gps_data_path)

# Auto-detect X, Y columns
candidates = [
    ('X(E/m)', 'Y(N/m)'),
    ('UTM_X', 'UTM_Y'),
    ('utm_x', 'utm_y'),
    ('x', 'y'),
    ('X', 'Y'),
]
x_col, y_col = None, None
for cx, cy in candidates:
    if cx in gps_data.columns and cy in gps_data.columns:
        x_col, y_col = cx, cy
        break
if x_col is None or y_col is None:
    raise KeyError(f"X, Y 좌표 필수 컬럼을 찾을 수 없습니다. 현재 데이터프레임 컬럼: {list(gps_data.columns)}")

# Ensure plotting columns are numeric and clean invalid rows
gps_data[x_col] = pd.to_numeric(gps_data[x_col], errors='coerce')
gps_data[y_col] = pd.to_numeric(gps_data[y_col], errors='coerce')
gps_data = gps_data.dropna(subset=[x_col, y_col])

# If 'direction' column is missing, assume all are 0
if 'direction' not in gps_data.columns:
    gps_data['direction'] = 0

# Create a plot
plt.figure(figsize=(12, 12))

# Plot the main GPS path
plt.plot(
    gps_data[x_col].to_numpy(),
    gps_data[y_col].to_numpy(),
    color='black',
    linestyle='-',
    linewidth=1,
    label='Path'
)

# Define colors for each direction
colors = {1: 'red', 0: 'black', -1: 'blue'}

# Plot the colored points for the main GPS path
for direction, color in colors.items():
    subset = gps_data[gps_data['direction'] == direction]
    plt.scatter(
        subset[x_col].to_numpy(),
        subset[y_col].to_numpy(),
        c=color,
        label=f'Direction {direction}',
        s=20
    )

# 각 점 옆에 DataFrame의 행(row) 인덱스 표시
for idx, row in gps_data.iterrows():
    plt.text(row[x_col], row[y_col], f" {idx}", fontsize=8, color='dimgray', alpha=0.8)

# Set labels and title
plt.xlabel(x_col)
plt.ylabel(y_col)
plt.title('Combined GPS Data Visualization')
plt.legend()
plt.grid(True)
plt.axis('equal')

# Show the plot
plt.show()
