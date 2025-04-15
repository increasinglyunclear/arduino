import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def load_gesture_data(folder_path):
    """Load all CSV files from the specified folder."""
    data = {}
    folder = Path(folder_path)
    
    for file in folder.glob('*.csv'):
        print(f"\nProcessing file: {file}")
        gesture_name = file.stem.split('_')[0]  # Get gesture name from filename
        if gesture_name not in data:
            data[gesture_name] = []
        # Read CSV with proper column names
        df = pd.read_csv(file, names=['timestamp', 'gesture', 'accelX', 'accelY', 'accelZ', 'gyroX', 'gyroY', 'gyroZ'])
        data[gesture_name].append(df)
    
    return data

def plot_gesture_data(data, gesture_name):
    """Plot accelerometer and gyroscope data for a specific gesture."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot accelerometer data
    for df in data[gesture_name]:
        axes[0].plot(df['accelX'], label='X', alpha=0.3)
        axes[0].plot(df['accelY'], label='Y', alpha=0.3)
        axes[0].plot(df['accelZ'], label='Z', alpha=0.3)
    
    axes[0].set_title(f'{gesture_name.capitalize()} Gesture - Accelerometer Data')
    axes[0].set_xlabel('Sample')
    axes[0].set_ylabel('Acceleration (g)')
    axes[0].legend()
    axes[0].grid(True)
    
    # Plot gyroscope data
    for df in data[gesture_name]:
        axes[1].plot(df['gyroX'], label='X', alpha=0.3)
        axes[1].plot(df['gyroY'], label='Y', alpha=0.3)
        axes[1].plot(df['gyroZ'], label='Z', alpha=0.3)
    
    axes[1].set_title(f'{gesture_name.capitalize()} Gesture - Gyroscope Data')
    axes[1].set_xlabel('Sample')
    axes[1].set_ylabel('Angular Velocity (deg/s)')
    axes[1].legend()
    axes[1].grid(True)
    
    plt.tight_layout()
    plt.savefig(f'{gesture_name}_gesture.png')
    plt.close()

def analyze_gestures(folder_path):
    """Analyze all gesture data and generate plots."""
    # Load all gesture data
    data = load_gesture_data(folder_path)
    
    # Print basic statistics
    print("\nGesture Analysis Summary:")
    print("=" * 50)
    
    for gesture_name, gesture_data in data.items():
        print(f"\n{gesture_name.capitalize()} Gesture:")
        print("-" * 30)
        
        # Calculate statistics for each recording
        for i, df in enumerate(gesture_data):
            print(f"\nRecording {i+1}:")
            print("Accelerometer Range:")
            print(f"  X: {df['accelX'].min():.2f} to {df['accelX'].max():.2f}")
            print(f"  Y: {df['accelY'].min():.2f} to {df['accelY'].max():.2f}")
            print(f"  Z: {df['accelZ'].min():.2f} to {df['accelZ'].max():.2f}")
            print("Gyroscope Range:")
            print(f"  X: {df['gyroX'].min():.2f} to {df['gyroX'].max():.2f}")
            print(f"  Y: {df['gyroY'].min():.2f} to {df['gyroY'].max():.2f}")
            print(f"  Z: {df['gyroZ'].min():.2f} to {df['gyroZ'].max():.2f}")
        
        # Generate plots
        plot_gesture_data(data, gesture_name)

if __name__ == "__main__":
    # Analyze gestures in the current directory
    analyze_gestures('.') 