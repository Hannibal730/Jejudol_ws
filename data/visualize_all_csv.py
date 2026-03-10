
import pandas as pd
import matplotlib.pyplot as plt

# Define file paths
processed_dir = '/home/hannibal/Jejudol_ws/data'
left_lane_path = f'{processed_dir}/0310_1.csv'
right_lane_path = f'{processed_dir}/0310_2.csv'
# gps_data_path = f'{processed_dir}/이전대회 GPS 데이터 (기본+평행주차)_direction.csv'
gps_data_path = f'{processed_dir}/0310_3.csv'

# Load the data
left_lane_data = pd.read_csv(left_lane_path)
right_lane_data = pd.read_csv(right_lane_path)
gps_data = pd.read_csv(gps_data_path)

# Ensure plotting columns are numeric and clean invalid rows
for df in (left_lane_data, right_lane_data, gps_data):
    df['X(E/m)'] = pd.to_numeric(df['X(E/m)'], errors='coerce')
    df['Y(N/m)'] = pd.to_numeric(df['Y(N/m)'], errors='coerce')

left_lane_data = left_lane_data.dropna(subset=['X(E/m)', 'Y(N/m)'])
right_lane_data = right_lane_data.dropna(subset=['X(E/m)', 'Y(N/m)'])
gps_data = gps_data.dropna(subset=['X(E/m)', 'Y(N/m)'])

# If 'direction' column is missing, assume all are 0
if 'direction' not in gps_data.columns:
    gps_data['direction'] = 0

# Create a plot
plt.figure(figsize=(12, 12))

# Plot left and right lanes
plt.plot(
    left_lane_data['X(E/m)'].to_numpy(),
    left_lane_data['Y(N/m)'].to_numpy(),
    color='orange',
    linestyle='-',
    linewidth=2,
    label='Left Lane'
)
plt.plot(
    right_lane_data['X(E/m)'].to_numpy(),
    right_lane_data['Y(N/m)'].to_numpy(),
    color='green',
    linestyle='-',
    linewidth=2,
    label='Right Lane'
)

# Plot the main GPS path
plt.plot(
    gps_data['X(E/m)'].to_numpy(),
    gps_data['Y(N/m)'].to_numpy(),
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
        subset['X(E/m)'].to_numpy(),
        subset['Y(N/m)'].to_numpy(),
        c=color,
        label=f'Direction {direction}',
        s=20
    )

# Set labels and title
plt.xlabel('X(E/m)')
plt.ylabel('Y(N/m)')
plt.title('Combined GPS Data Visualization')
plt.legend()
plt.grid(True)
plt.axis('equal')

# Show the plot
plt.show()
