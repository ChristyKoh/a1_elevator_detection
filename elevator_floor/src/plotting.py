#!/usr/bin/env python
"""
Authors(s): Abigail Chin, Christy Koh

Some plotting utils to display results
"""

import numpy as np
import matplotlib.pyplot as plt
import altitude_utils as ac
from scipy import signal

def plot_accelerations(process):
    data = np.array(process.accelerations_noisy) - process.accel_offset
    smooth_data = process.accelerations
    plt.plot(data, label='Noisy Data')
    plt.plot(smooth_data, label='Smooth Data')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Acceleration (m/s^2)')
    plt.title('IMU Acceleration Data')
    plt.legend()

    plt.show()


def plot_vel_displ(process):
    velocities = process.velocities
    displacements = process.displacements
    fig, ax = plt.subplots(2)
    # detrended_vel = signal.detrend(velocities)
    # detrend removes the linear trend of the data (centers at 0)
    # plt.plot(detrended_vel, label='Detrended Velocities')
    ax[0].plot(velocities)
    ax[1].plot(displacements)
    # plt.plot(ac.calc_displ(detrended_vel,process.delta_t),label='Adjusted Displacements')
    plt.xlabel('Frequency (Hz)')
    ax[0].set_ylabel('Velocity (m/s)')
    ax[1].set_ylabel('Displacement (m)')
    plt.suptitle('Calculated Velocities and Displacements')
    plt.legend()
    plt.show()