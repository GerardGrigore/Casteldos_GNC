# Project description

## Motivations

This project aims to design a complete Guidance, Navigation, and Control (GNC) system to automatically pilot a pleasure boat.
The ship has no sails, and its propulsion relies solely on a diesel engine whose direction can be controlled via a rudder bar.
Therefore, the GNC system to be developed must be capable of controlling heading of the vessel.
Controlling the heading, that is the orientation of the ship angle with respect to North around the vertical axis, allows indirect control of its position.

As a result, a system analysis was initially carried out to determine the main hardware components required to meet the functional requirements. A brief summary is as follows:
* Navigation: The position and heading of the vessel will be obtained through sensor fusion using an IMU, a compass, and a GPS (three types of sensors).
* Guidance: The desired heading, based on a predefined onboard trajectory and the navigation data, will be computed in real time by a guidance algorithm.
* Control: Two control loops will be implemented:
  * Outer control loop: Generates the commanded rudder angle needed to follow the desired heading.
  * Inner control loop: Precisely controls the linear displacement of the actuator moving the rudder to achieve the commanded angle.
 
However, the first months dedicated to the development of this project were allocated to designing a complete numerical simulator to assess the implemented algorithms. 
In the coming days and months, the objective will be to produce three documents:
* A justification note explaining the GNC algorithms.
* A detailed description of the simulator and the equations of motion of the vessel.
* A technical performance report that will simulate a mission and evaluate the performance and robustness of the developed algorithms.

## Numerical simulations & algorithms design

The simulators were constructed in Matlab/Simulink (version 2024). 
![image](https://github.com/user-attachments/assets/8a226e1f-9467-4772-9cb0-c931db80cadd)

The main choices and explored notions were the following:
* Navigation:
  * Position and heading estimation, fusion and filtering: Extended Kalman Filter based on a non-linear model of the motion of the ship.
  * High-frequency heading perturbations due to wave motion filtering: Observer based on a Kalman Filter embedding a frequency model of the wave motion.
  * Online estimation of the wave frequency and damping: Two-stage recursive Extended Least Squares Identification Algorithm ARMA.
  * Numerical and discretized filters to enhance the quality of the heading to be sent to the Controller and the Guidance algorithms.
* Control:
  * Discretized Proportional Integral Derivative Filtered (PIDF) anti wind-up embedded online algorithm. The tuning has been made according to temporal and
    frequency requirements and constraints.
  * Optimal Linear and Quadratic (LQ) controller. The tuning has been made on a simplified version of the simulator but performs very healthily inside the
    overall simulator.

The main design choices and explored concepts were as follows:
* Navigation: (Signal-Processing, Statistics & Probabilities)
  * Position and heading estimation, fusion, and filtering: Implemented using an Extended Kalman Filter based on a nonlinear model of the motion of the vessel.
  * Filtering of high-frequency heading perturbations due to wave motion: Developed an observer based on a Kalman Filter incorporating a wave motion frequency model.
  * Online estimation of wave frequency and damping: Applied a two-stage recursive Extended Least Squares Identification algorithm (ARMA model).
  * Numerical and discretized filters: Used to enhance the quality of the heading signal provided to the Control and Guidance algorithms.
* Control: (Control Theory, Mathematical modeling)
  * Discretized Proportional-Integral-Derivative with Filtering (PIDF) and anti-windup protection: Implemented as an online embedded algorithm, tuned according to both time-domain and frequency-domain 
    requirements and constraints.
  * Optimal Linear Quadratic (LQ) controller: Tuned on a simplified version of the simulator, yet demonstrating strong performance within the complete simulation environment.
  * Handling of non-linearities through the use of an anti wind-up strategy.
* Guidance: (Applied Mathematics)
  * Waypoint-based trajectory generation.
  * Curved-Path Guidance.

## Structure of the repository

This repository follows an incremental approach for the design of both the simulator and the associated algorithms. To get started, download all the project folders. The simulators can be accessed via the file named Simulations (the last item in the repository structure shown below).

 ![image](https://github.com/user-attachments/assets/f18dd96a-1622-4abc-a4d4-7573378d0624)

Each simulation case reflects specific modeling and algorithmic choices, tested with progressively increasing complexity. For instance: Simulation_Case_1 is the simplest simulator. It includes neither environmental disturbances nor estimation algorithms. Simulation_Case_6, on the other hand, incorporates wind and wave models, online GNC algorithms, discretized models for the plant and controllers, numerical filters, observers, and more.
For each subfolder corresponding to a simulation case number i, the following elements are included:
* _Parametrization_Simulation_Case_i_: A script used to initialize and call all functions, algorithms, and scripts relevant to this simulation. It also gathers the associated data.
* _Simulator_Case_i_: A Simulink model embedding the algorithms under validation and simulating a mission. This model should be run after executing the Parametrization_Simulation_Case_i script.
* _Trajectory_Comparison_Case_i_: A script that compares the simulated mission trajectory to the predefined reference trajectory.
* _Description_Case_i_: A text file outlining the main assumptions and modeling choices for the current simulation case. It also provides key conclusions and explains the rationale behind increasing the complexity 
  in successive simulation cases i + 1.

**For a complete and full description of the algorithms, mathematical models and simulators, technical descriptions and justification files will be uploaded soon.**

## Evolutions

In the coming days and months, the following concepts will be explored:
* Dispersion and robustness simulations to validate the theoretical algorithms.
* Transcoding of all developed GNC algorithms into C++.
* Analysis of the behavior of the hardware intended for use in the control loop.
* Implementation of Hardware-in-the-Loop (HIL) simulations.
* Exploration of Model Predictive Control (MPC) approaches.

Please note that some of the committed C++ functions and scripts originate from pre-existing open-source libraries and have not yet been implemented or transcoded from Matlab.
 







