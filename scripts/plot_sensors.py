
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # Save PNGs without opening plot windows

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot_sensor_folder(data_folder, output_folder):
    data_folder = Path(data_folder)
    output_folder = Path(output_folder)
    output_folder.mkdir(parents=True, exist_ok=True)

    def save_plot(filename):
        plt.tight_layout()
        path = output_folder / filename
        plt.savefig(path, dpi=180)
        plt.close()
        print(f"sensor plot: {path}")

    # IMU measurements
    imu = pd.read_csv(data_folder / "imu.csv")

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i, axis in enumerate("xyz"):
        axes[i].plot(
            imu["t"],
            imu[f"truth_accel_{axis}_mps2"],
            label="Truth",
        )
        axes[i].plot(
            imu["t"],
            imu[f"accel_{axis}_mps2"],
            label="Sensor",
            alpha=0.7,
        )
        axes[i].set_ylabel(f"{axis.upper()} [m/s²]")
        axes[i].grid(True)
        axes[i].legend()
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("IMU Accelerometer")
    save_plot("accelerometer.png")

    # Gyroscope measurements
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for i, axis in enumerate("xyz"):
        axes[i].plot(
            imu["t"],
            np.rad2deg(imu[f"truth_gyro_{axis}_radps"]),
            label="Truth",
        )
        axes[i].plot(
            imu["t"],
            np.rad2deg(imu[f"gyro_{axis}_radps"]),
            label="Sensor",
            alpha=0.7,
        )
        axes[i].set_ylabel(f"{axis.upper()} [deg/s]")
        axes[i].grid(True)
        axes[i].legend()
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("IMU Gyroscope")
    save_plot("gyroscope.png")

    # IMU measurement errors
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for axis in "xyz":
        accel_error = (
            imu[f"accel_{axis}_mps2"]
            - imu[f"truth_accel_{axis}_mps2"]
        )
        gyro_error = (
            imu[f"gyro_{axis}_radps"]
            - imu[f"truth_gyro_{axis}_radps"]
        )
        axes[0].plot(imu["t"], accel_error, label=axis.upper())
        axes[1].plot(
            imu["t"], np.rad2deg(gyro_error), label=axis.upper()
        )
    axes[0].set_ylabel("Accelerometer error [m/s²]")
    axes[1].set_ylabel("Gyroscope error [deg/s]")
    for ax in axes:
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("IMU Measurement Errors")
    save_plot("imu_errors.png")

    # GNSS position and velocity
    gnss = pd.read_csv(data_folder / "gnss.csv")
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    for axis in "NED":
        axes[0].plot(
            gnss["t"], gnss[f"pos_{axis}_m"],
            label=f"Position {axis}",
        )
        axes[1].plot(
            gnss["t"], gnss[f"vel_{axis}_mps"],
            label=f"Velocity {axis}",
        )

    axes[0].set_ylabel("Position [m]")
    axes[1].set_ylabel("Velocity [m/s]")
    for ax in axes:
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("Time [s]")
    fig.suptitle("GNSS Measurements")
    save_plot("gnss.png")

    # Radar altimeter
    radar = pd.read_csv(data_folder / "radar_altimeter.csv")
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(
        radar["t"], radar["truth_altitude_m"],
        label="Truth",
    )
    ax.scatter(
        radar["t"], radar["range_m"],
        s=8, label="Radar",
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Altitude [m]")
    ax.set_title("Radar Altimeter")
    ax.grid(True)
    ax.legend()
    save_plot("radar_altimeter.png")

    # Barometer
    baro = pd.read_csv(data_folder / "barometer.csv")
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(baro["t"], baro["pressure_pa"])
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Pressure [Pa]")
    ax.set_title("Barometer")
    ax.grid(True)
    save_plot("barometer.png")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("data_folder")
    parser.add_argument("output_folder")
    args = parser.parse_args()

    plot_sensor_folder(args.data_folder, args.output_folder)
