# Where Am I Project Starter Code

The objective of this project is to learn how to utilize ROS packages to accurately localize a mobile robot inside a provided map in the Gazebo and RViz simulation environments.

<!--more-->

[//]: # (Image References)

[image1]: ./misc_images/kalman_filtering.jpg "Kalman filters"
[image2]: ./misc_images/kalman_gain.jpg "Multidimensional Kalman filters"
[image3]: ./misc_images/gaussian.jpg "Gaussian"
[image4]: ./misc_images/formula.jpg "Formula"
[image5]: ./misc_images/linear.jpg "Linear Approximation"
[image6]: ./misc_images/step_0.jpg "MCL Step 0"
[image7]: ./misc_images/step_1.jpg "MCL Step 1"
[image8]: ./misc_images/step_2.jpg "MCL Step 2"
[image9]: ./misc_images/step_3.jpg "MCL Step 3"
[image10]: ./misc_images/step_4.jpg "MCL Step 4"
[image11]: ./misc_images/step_5.jpg "MCL Step 5"
[image12]: ./misc_images/bot_udacity.jpg "Udacity Robot"
[image13]: ./misc_images/bot_me.jpg "Custom Robot"
[image14]: ./misc_images/robot_setup.jpg "Robot Setup"
[image15]: ./misc_images/costmap.jpg "Large Costmap"
[image16]: ./misc_images/costmap_smaller.jpg "Smaller Costmap"
[image17]: ./misc_images/.jpg ""
[image18]: ./misc_images/.jpg ""
[image19]: ./misc_images/.jpg ""
[image20]: ./misc_images/.jpg ""
[image21]: ./misc_images/.jpg ""
[image22]: ./misc_images/.jpg ""
[image23]: ./misc_images/.jpg ""
[image24]: ./misc_images/.jpg ""
[image25]: ./misc_images/.jpg ""
[image26]: ./misc_images/.jpg ""
[image27]: ./misc_images/.jpg ""
[image28]: ./misc_images/.jpg ""
[image29]: ./misc_images/.jpg ""
[image30]: ./misc_images/.jpg ""
[image31]: ./misc_images/.jpg ""
[image32]: ./misc_images/.jpg ""
[image33]: ./misc_images/.jpg ""
[image34]: ./misc_images/.jpg ""

---


#### How to run the program with your own code

For the execution of your own code, we head to the Project Workspace

Go to Desktop

You can launch it by running the following commands first
```bash
  cd /home/workspace/catkin_ws
  catkin_make
  source devel/setup.bas
```

And then run the following in separate terminals
```bash
  roslaunch udacity_bot udacity_bot
  roslaunch udacity_bot amcl
  rosrun udacity_bot navigation goal
```

---

The summary of the files and folders int repo is provided in the table below:

| File/Folder                      | Definition                                                                                            |
| :------------------------------- | :---------------------------------------------------------------------------------------------------- |
| config/*                         | Folder that contains all the objects and settings of the belt image classification.                   |
| launch/*                         | Folder that contains all the parameters and configurations for the generation of the AlexNet model.   |
| maps/*                           | Folder that contains all the parameters and configurations for the generation of the GoogleNet model. |
| meshes/*                         | Folder that contains all the objects and settings of the practice image classification.               |
| src/*                            | Folder that contains all the parameters and configurations for the generation of the AlexNet model.   |
| urdf/*                           | Folder that contains all the parameters and configurations for the generation of the GoogleNet model. |
| worlds/*                         | Folder that contains all the parameters and configurations for the generation of the GoogleNet model. |
| misc_images/*                    | Folder containing the images of the project.                                                          |
|                                  |                                                                                                       |
| README.md                        | Contains the project documentation.                                                                   |
| README.pdf                       | Contains the project documentation in PDF format.                                                     |

---

**Steps to complete the project:**  

1. Follow the steps outlined in this Project Lesson, to create your own ROS package, develop a mobile robot model for Gazebo, and integrate the AMCL and Navigation ROS packages for localizing the robot in the provided map.
2. Using the Parameter Tuning section as the basis, add and tune parameters for your ROS packages to improve your localization results in the map provided. Feel free to explore new parameters using the resources provided earlier in the same section. Please include the image of RViz with the robot at goal position and the PoseArray displayed. Each run may take a long time so don't forget to screenshot your robot at the goal position.
3. After achieving the desired results for the robot model introduced in the lesson, implement your own robot model with significant changes to the robot's base and possibly sensor locations.
4. Check your previous parameter settings with your new robot model, and improve upon your localization results as required for this new robot.
5. Document your work. You could use this Writeup Template. 


## [Rubric Points](https://review.udacity.com/#!/rubrics/1365/view)
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Abstract

Where Am I? The goal of this project is to configure a number of ROS packages that can be used in conjunction to accurately localize and navigate a mobile robot inside a provided map in the Gazebo and RViz simulation environments.

## Introduction

This project consists of creating a complete ROS package that includes a Gazebo world, a robot model in URDF and using the ROS navigation stack to localize the robot on the map as well as move it to a desired destination and pose avoiding any obstacles on the way.


## Background / Formulation

Robots often operates in unpredictable environments where the agent is uncertain about its state. Using calculus and probability theory it is possible to manage the this uncertainty and represent the robots beliefs of its state in mathematical form that can later be used for decision making.

For a robot to find its position on a map (localization problem) it needs to filter noisy sensor data using probabilistic methods. There are 3 types of localization problems:

#### 1. Position tracking

This is the first and simplest form of the localization problem: in this scenario the robots initial position is known and the algorithm keeps track of the robot's position as it moves.

#### 2. Global localization

For cases where the initial position is unknown and needs to be determined at as the robot moves. This problem combines the uncertainties from measurements and actions in a cycles to achieve a precise estimate of the location.

#### 3. Kidnaped robot

The most difficult localization problem is to recover from abrupt changes in position like moving a robot from a position on the map to another. This is particularly difficult for bayesian algorithms to recover from since they preserve an internal belief that interfere with the new robot state.
​
En el programa del curso se nos presentaron dos métodos de localización: Filtros Kalman y filtros de partículas. Ambos métodos probabilísticos se pueden aplicar con éxito al seguimiento de posición y a la localización global y más tarde ayudarán a localizar mi robot en Gazebo y RViz.

### Kalman filters

The Kalman filter estimates the value of a variable by updating its estimate as measurement data is collected filtering out the noise. Kalman Filters models the state uncertainty using Gaussians and it is capable of making accurate estimates with just a few data points.

KF starts with an initial state estimate then performs the following cycle: measurement update produced by sensor measurements followed by a state prediction from control actions.

![alt text][image1]


### Multidimensional KF

Most of real world robots operates on multiple dimensions, i.e. a drone's position is defined by x, y and z coordinates and orientation in roll, pitch and yaw angles. This motivates generalizing the KF to multiple dimensions with equations in matrix format.

The MKF algorithm consists of calculating the Kalman Gain K that determines how much weight should be placed on the state prediction, and how much on the measurement update. It is an averaging factor that changes depending on the uncertainty of the measurement and state updates.

![alt text][image2]
###### The Kalman gain K averages the measurement update P and motion update x


### Extended Kalman Filter

Kalman Filter assumes that motion and measurement models are linear and that the state space can be represented by a unimodal Gaussian distribution. Most mobile robots will execute non linear motion like following a curve. Non linear actions will result in non-Gaussian posterior distributions that cannot be properly modeled by a closed form equation.

![alt text][image3]
###### Gaussian prior (left) subject to a non-linear action (atan(x) — center) results in a non-Gaussian posterior (right)

EKF approximates motion and measurements to linear functions locally (i.e. by using the first two terms of a Taylor series) to compute a best Gaussian approximation of the posterior covariance. This trick allows EKF to be applied efficiently to a non-linear problems.

   y = z − Hx′

is replaced with the nonlinear h(x'):

   y = z — h(x’)

This is where the multivariate Taylor series comes into play. The function function h(x) can be approximated by a Taylor series centered about the mean μ, as defined below.

![alt text][image4]

The h(x) approximation above is now linear around μ and will produce a Gaussian posterior.

![alt text][image5]
###### Linear approximation (in purple) of the atan(x) function around x = 0


### Particle filters

Monte Carlo localization algorithm similar to Kalman Filters estimates the posterior distribution of a robot’s position and orientation based on sensory information but instead of using Gaussians it uses particles to model state.

The algorithm is similar to KF where motion and sensor updates are calculated in a loop but MCL adds one additional step: a particle resampling process where particles with large importance weights (computed during sensor updates) survive while particles with low weights are ignored.

In the MCL example below all particles are uniformly distributed initially. In the following update steps the particles that better match the predicted state survive resulting in a concentration of particles around the robot estimated location.

![alt text][image6]
![alt text][image7]
![alt text][image8]
![alt text][image9]
![alt text][image10]
![alt text][image11]
###### MCL steps visualization — Particles (green/yellow) start uniformly distributed (step 0) and then concentrates around the ground truth (blue) (steps 1–5). Robot's sensors read distances to landmarks (red).

MCL solves the local and global localization problems but similar to KF is not suitable for addressing the kidnaped robot problem.

### Model Configuration

First we started configuring the world and launch files by following the instructions in the classroom, then we proceed to model the 2 robots below. The first robot based off the classroom (left) and the second is a custom robot (left).

![alt text][image12]
![alt text][image13]
###### Classroom robot (left) and custom robot (right).

The custom robot has a differential actuation, just like the classroom robot, instead of 4 wheels of a regular car. so only 2 wheels are maintained to comply with the ROS navigation package hardware requirement.


### Tuning Parameters

Now it will be described the choice the parameters for each of the packages from the ROS navigation stack. The next tutorial is a starting point as it provides an excellent overview of the parameters and packages discussed below.

![alt text][image14]

#### amcl.launch

The only parameters that were needed for tweek in the AMCL launch file was the translational and rotational movement required before performing a filter update. The default values were too large for the slow moving robot. After reducing them by an order of magnitude, status updates were obtained far more frequently resulting in a reduction of the location uncertainty to a minimum.

update_min_d = 0.01
update_min_a = 0.005

With the experiments it was discovered that very good results can be obtained with just 5–20 particles! The more particles the more accurate location will be but at the cost of additional compute resources. To keep the state updates as frequent as posible it's chosen to keep the particle count down to a minimum.

min_particles = 5
max_particles = 20

Noisy readings from the laser sensor are also discovered. It would at times detect obstacles at short distances when there was nothing there. To prevent those readings to interfere with the localization, a minimum range of lasers is defined.

laser_min_range = 0.4

Finally, the estimate of the initial pose is set to zero to coincide with the location of the robot at start-up.

initial_pose = (0, 0, 0)


#### TrajectoryPlannerROS — base_local_planner_params.yaml

The trajectory planner is responsible for computing velocity commands to send to the mobile base of the robot given a high-level plan. It seems useful to first understand what was going on under the planner's covers. Enabling the publish_cost_grid_pc parameter allows you to visualize the cost_cloud topic in RViz.

publish_cost_grid_pc: true

![alt text][image15]
###### A large costmap is heavily influenced by the navigation goal.

It's detected that the robot was deviating too much from the global path as if attempting to head straight to the goal. Therefore, the following parameters are changed to reduce the goal influence (gdist_scale) and increasing the global path (pdist_scale) compliance.

pdist_scale: 1.0
gdist_scale: 0.4


#### local_costmap_params.yaml

The local costmap size was by far the most important parameters to get right to successfully navigate around the corner at the end of the corridor. This is because the goal creates a huge influence over the local costmap. This resulted in the robot getting pulled away from the calculated global path. Reducing the size of the local costmap to approximate to the corridor width solved this problem.

width: 5.0
height: 5.0

![alt text][image16]
###### A smaller costmap will produce a gradient along the global path that is not directly affected by the end goal















