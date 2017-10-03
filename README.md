# TEAM AURORA: SDCND CAPSTONE PROJECT

### Team Lead
Siva Kondubhatla (sivakirank@gmail.com)

### Team Members 
Andrew Stewart (andrew.stewart2@honeywell.com) 
Jim Reynolds (jreyn121@ford.com) 
Peng Zhang (penggzhang@126.com)
Francisco Jose Rey Gozalo (fcojreyg@gmail.com)

### Summary
This is the personal repo for [Jim Reynolds](https://github.com/jamesjosephreynolds), forked from the [Aurora team repo](https://github.com/DFStewart/CarND-Capstone).

The overall project was broken up into two workflows:
1. Waypoint update calculation and vehicle control
2. Stoplight identification and state classification

I focused on waypoint update calculation and vehicle control.

## Waypoint Update
The geographical `x,y,z` position of all path waypoints are provided.  The waypoint updater must calculate the desired velocity (m/sec) at each of the next `N` waypoints.  This includes both the longitudinal and lateral velocity.  The desired velocity at upcoming waypoints can generally be determined based on three cases:
1. If the vehicle is at the desired speed, and there is no red light in the stop horizon, maintain the desired speed.
2. If the vehicle speed is positive, and there is a red light in the stop horizon, calculate a decreasing velocity at the upcoming waypoints until a stop is planned at the stopline. `v(k) >= v(k+1) >= ... >= v(k+M) = 0`
3. If the vehicle speed is less than the desired speed, and there is no red light in the stop horizon, calculate an increasing velocity at the upcoming waypoints until the desired velocity is reached. `v(k) <= v(k+1) <= ... <= v(k+M) = v_des`
*Note that for this project, acceleration (case 3) is handled by the accelerator pedal PID controller, and the desired velocity is planned for all upcoming waypoints.*

For the case where there is no red light within the stop horizon `STOP_DIST_ERR` the desired velocity determination is trivially set to a constant value `SPEED_MPH*MPH2MPS`.

For the case where there is a red stop light ahead, the determination of the desired velocity at the upcoming waypoints is more involved.  The key concept is the planning determines the constant deceleration necessary to stop at the stop line, then back calculates the velocity of each waypoint leading up to the stop line.

The constant deceleration is calculated as shown below:
```Python
decel_mpss = -1.*(vel_i * vel_i / pos_stopline)/2
```

For each waypoint between the vehicle's current location and the stopline, `decel_mpss` is then used to calculate the desired velocity as~
```Python
vel_sq = vel_i * vel_i + 2.*decel_mpss*pos_stopline
if vel_sq < 0.0:
    vel_sq = 0.0
self.target_speed_mps = math.sqrt(vel_sq)
```

Detailed derivations of these calculation are available [here](/documentation/ConstantAccelDerivation.pdf).

## Control Systems
There are three closed-loop controllers present for the vehicle motion control:
1. Throttle (accelerator pedal)
2. Braking
3. Steering

Each of these uses a PID controller.  The throttle and braking controllers are mutually exclusive; when one of the controllers is active the other is held in reset, and vice versa.

### Throttle
The throttle controller is strictly a feedback controller.  There is no feedforward throttle calculation based on speed sepoint.  The velocity error is the current desired linear velocity (m/sec) minus the current linear velocity (m/sec).  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

The throttle controller is active when `vel_err > 0.0`.  Otherwise, the throttle controller is reset, and the brake controller is utilized.

### Brake
The brake controller is strictly a feedback controller.  There is no feedforward brake calculation based on speed sepoint.  The velocity error is the current desired linear velocity (m/sec) minus the current linear velocity (m/sec).  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

The brake controller is active when `vel_err <= 0.0`.  Otherwise, the brake controller is reset, and the throttle controller is utilized.

### Steering
The steering control system has 3 main components:
1. Angular velocity setpoint determination
2. Feedforward steering angle command
3. Feedback steering angle command

Setpoint determination is done within the `waypoint_follower` module, specifically in `pure_pursuit_core.cpp`.  The function fits a curve to the desired waypoints, then uses the curvature (rad/m) and desired linear velocity (m/sec) to calculate the corresponding angular velocity (rad/sec).

The feedforward steering angle command is calculated in the `twist_controller` module, in `twist_controller.py` and `yaw_controller.py`. transforms the desired angular velocity (rad/sec), desired linear velocity (m/sec) and current linear velocity (m/sec) into a steering wheel angle (rad).  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

The feedback steering angle command is calculated in `twist_controller.py`.  Two different methods were evaluated for this controller:
1. Cross track error (CTE) based PID control
2. Angular velocity error based PID control

#### Cross Track Error Based PID Control
The error is calculated as the normal distance between the vehicle and line-of-best-fit of the waypoints, with a 3rd order polynomial.  PID control is applied to this error term.  CTE is calculated in `dbw_node.py`.

```Python
def Compute_CTE(self):
    # Similar to the MPC project we need to convert from global coordinates to local car body coordinates
    # First translate and then rotate
        
    global_car_x = self.current_pose.x
    global_car_y = self.current_pose.y
        
    roll,pitch,yaw = tf.transformations.euler_from_quaternion([self.current_orient.x,self.current_orient.y,self.current_orient.z,self.current_orient.w])
    yaw = -1.*yaw
        
    x_car_body = []
    y_car_body = []
        
    for idx,wpt in enumerate(self.finalwpts):
        del_x = wpt.pose.pose.position.x - global_car_x
        del_y = wpt.pose.pose.position.y - global_car_y
            
        temp1 =  del_x*math.cos(yaw) - del_y*math.sin(yaw)
        temp2 =  del_x*math.sin(yaw) + del_y*math.cos(yaw)
            
        x_car_body.append(temp1)
        y_car_body.append(temp2)
        
    # As with MPC project, fit a 3rd order polynomial that fits most roads    
    coeff_3rd = np.polyfit(x_car_body,y_car_body,3)       
    CTE       = np.polyval(coeff_3rd,0.0)
        
    #Limit CTE to +/-5 which is empirically the limits of the lanes 0 and 2
    if CTE > 5.0:
        CTE = 5.0
    if CTE < -5.0:
        CTE = -5.0       
        
    return CTE
```

#### Angular Velocity Error Based PID Control
The error is calculated as the difference between the measured angular velocity and the desired angular velocity.  PID control is applied to this error term.  A deadzone is applied to the angular velocity error in order to minimize steering corrections (integral windup, oscillation) when very near the setpoint.  The desired velocities are available in `ROS Topic: /twist_cmd` while the current velocities are available in `ROS Topic: /current_velocity`.

```Python
# Compute angular velocity error
ang_err = proposed_angular_velocity - current_angular_velocity

# deadband
if ang_err >= 0.005:
    ang_err -= 0.005
elif ang_err <= -0.005:
    ang_err += 0.005
else:
    ang_err = 0.0

# upper limit
if ang_err >= 0.25:
    ang_err = 0.25
elif ang_err <= -0.25:
    ang_err += 0.25

ang_err = self.LPFilt_Str.filt(ang_err)
```

#### Final Steering Control Design
The CTE based control was chosen in the end as it performed more robustly to momentary spikes in error signal (these may be artifacts of the simulator, or discontinuities in the waypoint calculations).

## Traffic Light Detection Node
Once the vehicle is able to process waypoints, generate steering and throttle commands, and traverse the course, it will also need stop for obstacles. Traffic lights are the first obstacle that we focus on.

The traffic light detection node `tl_detector.py` subscribes to three topics:

* `/base_waypoints` provides the complete list of waypoints for the course.
* `/current_pose` is used to determine the vehicle's location.
* `/image_color` which provides an image stream from the car's camera. These images are used to determine the color of upcoming traffic lights.

The permanent `(x, y)` coordinates for each traffic light's stop line are provided by the config dictionary, which is imported from the `traffic_light_config` file.

The node publish the index of the waypoint for nearest upcoming red light's stop line to a single topic: `/traffic_waypoint`.

#### Pipeline
The task for this light detection portion can be broken into two steps:

1. Use the vehicle's location and the `(x, y)` coordinates for traffic light stop lines to find the nearest visible traffic light ahead of the vehicle. This takes place in the `process_traffic_lights` method of `tl_detector.py`. We use the `get_closest_waypoint` method to find the closest waypoints to the vehicle and lights. Using these waypoint indices, we determine which light is ahead of the vehicle along the list of waypoints.
2. Locate the traffic light in the camera image data and classify its state. The functionality of locating the light takes place in the `get_light_state` method of `tl_detector.py`. For this task, we use the traffic light's exact position in 3D space as provided by the `vehicle/traffic_lights` topic rather than the 2D stop line position from the config file. And then the functionality of classifying the light's color state takes place in the `tl_classifier.py`. 

#### Traffic Light Detection Package Files
Within the traffic light detection package, there are following files to implement the whole detection-and-then-classification pipeline:

* `tl_detector.py`: This python file processes the incoming traffic light data and camera images. It uses the light classifier to get a color prediction, and publishes the location of any upcoming red lights.
* `tl_classifier.py`: This file contains the `TLClassifier class`. We use this class to implement traffic light classification. The `get_classification` method takes a camera image as input and return an ID corresponding to the color state of the traffic light in the image. 
* `/models` directory: Under this directory, there are checkpoint files and label maps of trained networks for classifying light color state. Using these files, incoming images will run through the frozen inference graph and identified color states will be returned.

## Results:
Below is a [video](https://youtu.be/A5ZHc05XyBk) of our car driving around the simulator track 4 times over a 50 minute period:

[![Whoops, there should be a picture here!](https://img.youtube.com/vi/A5ZHc05XyBk/0.jpg)](https://youtu.be/A5ZHc05XyBk)

## Installation Notes

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository

Project Code:
```bash
git clone https://github.com/DFStewart/CarND-Capstone.git
```

Udacity Original Project Code:
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
