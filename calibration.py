#!/usr/bin/env python
# coding: utf-8

# In[354]:


# Importing libraries
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import matplotlib.pyplot as plt


# Initializing values
np.random.seed(42)
size = 50           # number of joint angle configurations

l1_true = 0.45      # link 1 true length (m)
l2_true = 0.35      # link 2 true length (m)
b1_true = 0.01      # angle 1 true bias (rad)
b2_true = -0.02     # angle 2 true bias (rad)
params_true = np.array([l1_true, l2_true, b1_true, b2_true])

theta1_list = np.random.randint(-90, 90, size) * (np.pi/180)      # list of configurations for joint angle 1
theta2_list = np.random.randint(-90, 90, size) * (np.pi/180)      # list of configurations for joint angle 2

mean = 0
std_dev = 0.003
noise_x = np.random.normal(mean, std_dev, size)                   # gaussian noise in x-coordinate measurement
noise_y = np.random.normal(mean, std_dev, size)                   # gaussian noise in y-coordinate measurement

fig1, ax1 = plt.subplots(figsize = (12, 6))
fig2, ax2 = plt.subplots(figsize = (12, 6))
bin_edges = [x * 0.02 for x in range(11)]                         # define bin boundaries for plotting histogram of errors


# Defining residuals function for least_squares optimization and plotting histogram
def calc_residuals(params, error_x, error_y, theta1, theta2):
    l1, l2, b1, b2 = params
    x_measured = l1 * np.cos(theta1 + b1) + l2 * np.cos(theta1 + b1 + theta2 + b2) + error_x                    # actual x-coordinate of end-effector using forward kinematics with noise and updated parameters
    x_predicted = l1_true * np.cos(theta1 + b1_true) + l2_true * np.cos(theta1 + b1_true + theta2 + b2_true)    # estimated x-coordinate of end-effector using forward kinematics with true parameters
    y_measured = l1 * np.sin(theta1 + b1) + l2 * np.sin(theta1 + b1 + theta2 + b2) + error_y                    # actual y-coordinate of end-effector using forward kinematics with noise and updated parameters
    y_predicted = l1_true * np.sin(theta1 + b1_true) + l2_true * np.sin(theta1 + b1_true + theta2 + b2_true)    # estimated y-coordinate of end-effector using forward kinematics with true parameters
    residuals = np.sqrt(((x_measured - x_predicted) ** 2) + ((y_measured - y_predicted) ** 2))                  # Euclidean error
    ax2.hist(residuals, bins = bin_edges, color = 'skyblue', edgecolor = 'black')                               # plotting histogram of errors
    return residuals


#  Defining function to calculate RMS error between true values and observed values of parameters
def calc_rms(true_values, observed_values):
    squared_differences = (true_values - observed_values) ** 2
    mean_squared_error = np.mean(squared_differences)
    rms_error = np.sqrt(mean_squared_error)
    rms_error = np.round(rms_error, decimals = 3)
    return rms_error


# Initial guess for parameters
l1_initial = 0.35    # link 1 length (m) initial guess
l2_initial = 0.29    # link 2 length (m) initial guess
b1_initial = 0.0     # angle 1 bias (rad) initial guess
b2_initial = 0.0     # angle 2 bias (rad) initial guess
params_initial_guess = np.array([l1_initial, l2_initial, b1_initial, b2_initial])


# Using least_squares for optimization and extracting optimized parameters
result = least_squares(calc_residuals, params_initial_guess, args = (noise_x, noise_y, theta1_list,theta2_list),
                      ftol = 1e-12, xtol = 1e-12, gtol = 1e-12)
params_optimized = result.x                                     # extraction of optimized parameters
params_optimized = np.round(params_optimized, decimals = 3)


# Calculating calibration errors
params_pre_cal_error = calc_rms(params_true, params_initial_guess)
params_post_cal_error = calc_rms(params_true, params_optimized)


# Calculating pre-calibration and post-calibration end-effector errors
x_measured_pre_cal = l1_initial * np.cos(theta1_list + b1_initial) + l2_initial * np.cos(theta1_list + b1_initial + theta2_list + b2_initial) + noise_x                                                    # pre-calibration actual x-coordinate of end-effector
x_measured_post_cal = params_optimized[0] * np.cos(theta1_list + params_optimized[2]) + params_optimized[1] * np.cos(theta1_list + params_optimized[2] + theta2_list + params_optimized[3]) + noise_x      # post-calibration actual x-coordinate of end-effector
x_predicted = l1_true * np.cos(theta1_list + b1_true) + l2_true * np.cos(theta1_list + b1_true + theta2_list + b2_true)                                                                                    # predicted x-coordinate of end-effector
y_measured_pre_cal = l1_initial * np.sin(theta1_list + b1_initial) + l2_initial * np.sin(theta1_list + b1_initial + theta2_list + b2_initial) + noise_y                                                    # pre-calibration actual y-coordinate of end-effector
y_measured_post_cal = params_optimized[0] * np.sin(theta1_list + params_optimized[2]) + params_optimized[1] * np.sin(theta1_list + params_optimized[2] + theta2_list + params_optimized[3]) + noise_y      # post-calibration actual y-coordinate of end-effector
y_predicted = l1_true * np.sin(theta1_list + b1_true) + l2_true * np.sin(theta1_list + b1_true + theta2_list + b2_true)                                                                                    # predicted y-coordinate of end-effector

ee_pre_cal_error = np.sqrt(((calc_rms(x_predicted, x_measured_pre_cal)) ** 2) + ((calc_rms(y_predicted, y_measured_pre_cal)) ** 2))
ee_post_cal_error = np.sqrt(((calc_rms(x_predicted, x_measured_post_cal)) ** 2) + ((calc_rms(y_predicted, y_measured_post_cal)) ** 2))
ee_pre_cal_error = np.round(ee_pre_cal_error, decimals = 3)
ee_post_cal_error = np.round(ee_post_cal_error, decimals = 3)

# Generating scatter plot of end-effector positions
ax1.plot(x_measured_pre_cal, y_measured_pre_cal, 'o', color = 'red', linestyle = 'None', label = 'Measured pre-calibration position')
ax1.plot(x_measured_post_cal, y_measured_post_cal, 'o', color = 'green', linestyle = 'None', label = 'Measured post-calibration position')
ax1.plot(x_predicted, y_predicted, 'x', color = 'black', linestyle = 'None', markersize = 10, label = 'Predicted position')


# Displaying outputs
print("True parameters: ", params_true)
print("Initial guess: ", params_initial_guess)
print("Pre-calibration RMS error for parameters: ", params_pre_cal_error)
print("Post-calibration RMS error for parameters: ", params_post_cal_error)
print(f"Pre-calibration RMS error for end-effector position = {ee_pre_cal_error} m")
print(f"Post-calibration RMS error for end-effector position = {ee_post_cal_error} m")
print("Estimated parameters: ", params_optimized)


# Plotting and saving graphs
ax1.set_xlabel("x-coordinate (m)")
ax1.set_ylabel("y-coordinate (m)")
ax1.set_title("End-effector position")
ax1.legend()
ax2.set_xlabel("Error (m)")
ax2.set_ylabel("Frequency")
ax2.set_title("Error histogram")
fig1.savefig('End-effector position.png')
fig2.savefig('Error histogram.png')


# In[ ]:




