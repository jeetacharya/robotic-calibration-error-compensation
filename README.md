# robotic-calibration-error-compensation
Robotic Calibration and Error Compensation — 2-link planar arm

## Problem:
1. In real world scenarios, faulty sensor readings may lead to incorrect robotic parameters that may have to be recalibrated.
2. Using different commanded joint angles and an initial guess of the robot’s parameters, in this case, the 2 link lengths and 2 joint angle biases of a 2-link planar robotic arm, the true parameter values need to be recovered so that future forward kinematics equations can be incorporated with the sensor error.
3. This project aims to resolve this problem by optimizing the parameters so that the end effector’s position can be more accurately represented along with the sensor errors.

## Approach:
1. The true parameters of the robot’s links and joint angle biases are recorded.
2. With a set of different pairs of non-singular joint angle configurations and assuming a Gaussian distribution of error, a least squares function is used to optimize the parameters with an initial guess.
3. A residuals function calculates the Euclidean error between the predicted and measured end-effector position and returns that result to the least squares function.
4. The least squares function optimizes the parameters specified in the residuals function such that the Euclidean error returned by the residuals function is minimized. It tries to converge towards a minimum within the specified bounds and tolerances.
5. The least squares function is chosen because it is continuously differentiable (even at 0) which makes it possible to find the optimized parameters.

## Results:
1. The RMS error for parameters was reduced by 98.3% after their optimization.
2. The RMS error for end effector position was reduced by 97.2% after optimizing the parameters.
3. The measured pre-calibration, measured post-calibration and predicted positions of the end effector for all the different sets of joint angle pairs are shown in the graph below:
![End-effector position](https://github.com/jeetacharya/robotic-calibration-error-compensation/blob/6489753581e79f134b83b130ea6423caf43cce74/End-effector%20position.png)
4. The measured post-calibration position of the end effector is more nearer to the predicted position than the measured pre-calibration position proving the effectiveness of the optimization.
5. The error distribution is shown in the form of a histogram below:
![Error histigram]()
