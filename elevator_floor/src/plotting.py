#!/usr/bin/env python
"""
Authors(s): Abigail Chin, Christy Koh

Some plotting utils to display results
"""

import numpy as np
import matplotlib.pyplot as plt
import altitude_calc as ac

def plot_array(process):
    data = np.array(process.accelerations_noisy) - process.accel_offset
    smooth_data = process.accelerations
    plt.plot(data, label='Noisy Data')
    plt.plot(smooth_data, label='Smooth Data')
    plt.legend()

    plt.show()


def plot_acceleration(process):
    data = np.array(process.accelerations_noisy) - process.accel_offset
    fig, ax = plt.subplots(2)
    smooth_data = process.accelerations
    # infls = np.where(np.diff(np.sign(smooth_data)))[0]
    plt.plot(data, label='Noisy Data')
    plt.plot(smooth_data, label='Smooth Data')
    plt.legend()
    
    v, d = ac.calc_vel_displacement(data, process.delta_t)
    plt.plot(v)
    plt.plot(d)
    
    plt.show()
    # for i, infl in enumerate(infls,1):
    #     plt.axvline(x=infl, color='k',label='Inflection point %d'%(i))
    