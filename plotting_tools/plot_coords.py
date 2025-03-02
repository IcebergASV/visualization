#! /usr/bin/env python

import os
import pandas as pd
import geopandas as gpd
import matplotlib.pyplot as plt
import numpy as np
import argparse
from shapely.geometry import Point

def plot_lat_long(file_path):

    df = pd.read_csv(file_path)

    print(df.head())
    gpd.options.display_precision = 7

    # Filter out rows where longitude or latitude are not numeric
    df = df[pd.to_numeric(df['longitude'], errors='coerce').notnull()]
    df = df[pd.to_numeric(df['latitude'], errors='coerce').notnull()]

    gdf = gpd.GeoDataFrame(
    df, geometry=gpd.points_from_xy(df.longitude, df.latitude), crs="EPSG:4326"
    )
    print(gdf.head())

    fig, ax = plt.subplots(figsize=(10, 6))
    scatter = ax.scatter(df['longitude'], df['latitude'], c='red', alpha=0.7, s=20)

    # Draw arrows from the previous point to the most recent one
    for i in range(1, len(df)):
        x_start, y_start = df.iloc[i-1]['longitude'], df.iloc[i-1]['latitude']
        x_end, y_end = df.iloc[i]['longitude'], df.iloc[i]['latitude']
        
        # Draw an arrow from (x_start, y_start) to (x_end, y_end)
        ax.annotate(
            "", 
            xy=(x_end, y_end), 
            xytext=(x_start, y_start),
            arrowprops=dict(facecolor='green', edgecolor='green', arrowstyle='->', alpha=0.6)
        )
    
    selected_points = []
    
    def on_pick(event):
        ind = event.ind[0]
        point = (df.iloc[ind]['latitude'], df.iloc[ind]['longitude'])
        # Format to 6 decimal places
        formatted_point = [point[0], point[1]]
        selected_points.append(formatted_point)
        print(f"Selected point: {formatted_point}")

    fig.canvas.mpl_connect('pick_event', on_pick)  # Trigger when a point is clicked
    scatter.set_picker(True)

    # Label each point with the value in the 'name' column, treating it as a string
    for i, row in df.iterrows():
        name = str(row['name'])  # Treat the 'name' as a string
        ax.text(row['longitude'], row['latitude'], name, fontsize=9, ha='right', color='blue')

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Geographical Plot with Hover Highlighting and Arrows")
    plt.show()

    return selected_points


def write_mission_file_from_plot(selected_points, directory_path, file_name):
    if not os.path.exists(directory_path):
        print("write_mission_file_from_plot: Directory not found")
        return
    
    base_name = os.path.splitext(os.path.basename(file_name))[0]
    mission_file_path = os.path.join(directory_path, f"{base_name}.waypoints")
    
    with open(mission_file_path, 'w') as f:
        f.write("QGC WPL 110\n")
        for idx, (lat, lon) in enumerate(selected_points):
            line = f"{idx} 0 3 16 0 0 0 0 {round(lat, 6)} {round(lon, 6)} 0 1\n"
            f.write(line)
    
    print(f"Mission file written to {mission_file_path}")


def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Plot coordinates from CSV and generate mission waypoints.")
    parser.add_argument("file_path", help="Path to the CSV file containing coordinates.")
    
    # Parse the arguments
    args = parser.parse_args()

    # Run the plotting and file writing functions
    selected = plot_lat_long(args.file_path)
    write_mission_file_from_plot(selected, "/home/gracepearcey/repos/icebergpath_planning_tool/mp_wp_missions", args.file_path)


if __name__ == "__main__":
    main()
