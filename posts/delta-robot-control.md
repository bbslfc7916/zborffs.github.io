# Delta Robot Control (2/2)
*<p style="text-align: center;">October 26, 2021</p>*

## Summary
In this article, I derive a hybrid motion-force control law for delta robots.

This post expands upon a [previous post](./delta-robot-modeling.html) on the modeling of delta robots.

All the source code is available on [github](https://github.com/zborffs/Delta). 

### Demo


## Hybrid Motion-Force Control


### Feedforward PID Motion Control
One of the most common control schemes for the motion control of robotic manipulators is feed-forward PID control.

![Feedforward PID Control topology]()

The general approach is to cancel out the nonlinearities of the model (i.e. the mass, coriolis, and gravity matrices), and then enforce second-order error dynamics with a basic PID setup. Designing such a controller to meet transient and steady-state specifications then becomes trivial assuming that one has a somewhat accurate model.

However, designing such a controller for a delta robot has two additional complications; namely, dealing with the contraint term and only actuating three of the nine joints.

One way of overcoming the first problem would be to ignore the constraint term by only admitting control forces that already satisfy the constraints. This will be the approach I take.

### Feedforward PID Force Control


## B-Splines
One popular class of reference trajectories are [B-Splines](https://en.wikipedia.org/wiki/B-spline).

B-Splines (short for basis splines) can be thought of as regular spline functions composed of linear combinations of basis spline functions. If the basis is chosen appropriately, then representing the spline in this fashion can massively improve the computational efficiency of calculating a trajectory. 

Additionally, by making good choices for the knot points, it is possible to capture discontinuities in the reference velocity or acceleration, enabling the robot to temporarily stop mid-trajectory, while also ensuring that the end-effector doesn't leave the convex-hull of any knot-points tracing a trajectory.

These benefits come in conjunction with the other benefits of using splines, such as the fact that they provide us with closed-form expressions for the displacement, velocity, and acceleration of the end-effector, which can be fed-forward into the control algorithm for the optimal performance.

These benefits motivate writing some code to generate referece trajectories using B-Splines.

## Robustness
The control law that we have derived makes use of knowledge of system parameters to compute output torques. Moreover, our simulations assume that we can perfectly capture the full-state of the system from the sensors without any additive noise. If this controller is to be at all useful for real-world applications, it will need to be robust both to model uncertainty -- that is slightly different model parameters than those the system actually exhibits -- and exogenous disturbances -- that is additive noise or vibrations causing the joints or links in actuality or their sensor measurements to deviate slightly from the controller's predictions.

In this section, we will test whether the controller is robust *in simulacra* by slightly modifying the controller's internal model parameters while simultaneously injecting additive white Gaussian noise to various state variables to simulate sensor noise. 

If controller can still reasonably track reference trajectories and return to the home position despite these factors, then we can be marginally more certain in the robustness of the controller. To test this more rigorously, this controller should be deployed to an actual robot and tested similarly.

### Model Uncertainty


### Exogenous Disturbances


## Digitization
If this code were ever used on physical hardware, it would be necessary first to digitize the controller. In this section, I go over digitization and demonstrate how one might implement the controller in C/C++ function.