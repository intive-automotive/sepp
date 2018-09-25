# sepp
small size evaluation and prototyping platform - yet another autonomous RC car

See SEPP on a first teleoperation rideout here: https://youtu.be/caKfN4w3eOs

See SEPP blink and with new bumpers: https://youtu.be/2P67onShV-0

Read the SEPP wiki, if you want to see more about architecture and development with SEPP: https://github.com/intence/sepp/wiki

![Sepp himself](https://github.com/intence/sepp/blob/master/img/sepp_small.png)

## Goals
* Create a platform to evaluate sensors, sensor fusion approaches and algorithms in a small sized and comparatively cheap prototype
* Use open source hardware and software to enable contributors and users to clone the prototype
* Use computer vision and ultrasonic sensors to implement first autonomous functions
* Use artificial intelligence for trained autonomous functions
* Use the platform to collect data for training of artificial intelligence

## Used technologies (or planned)
* 1:10 remote controlled car set with new motor (crawler brushed motor), new steering servo and stiffer 
* [nvidia jetson tx1](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems-dev-kits-modules/) - GPU computing platform for hosting ROS and computer vision algorithms and machine learning
* [Arduino](https://www.arduino.cc/) - one to have a *sensor board* and one to have a *actuator* board
* [ros](http://www.ros.org/) - Robot Operating System (ROS)
* [OpenCV](https://opencv.org/) - Open Source Computer Vision Library
* [TensorFlow](https://www.tensorflow.org/) - open source machine learning framework [planned]
* sensors: 8 x ultra sonic, rgb camera, 3D camera (kinect style), gyroscope, radar, gps dongle, lidar [planned]
* actuators: thrust servo, steering servo, red flashlight, buzzer
* infrastructure: ethernet shields for arduino, ethernet switch, NiMh accu pack, usb hubs, power bank (to power arduinos an jetson tx1)
* car body: bumpers, constructed aluminium frame designed and made by Andreas N. aka G-Shocker (thanks for that!), constructed 3D printed parts (https://www.thingiverse.com/thing:3105985), plenty of tape, cable ties, jumper cable and solder :)

## Skills of sepp today
* jetson tx1 up and running on ubuntu Xenial
* ROS Lunar Loggerhead up and running on jetson tx1
* simple computer vision pipeline implemented as ROS nodes with openCV
* ultrasonic sensors are read by sensor board (arduino) and sent to ROS core on jetson by ethernet
* thrust and steering servos are controlled by actuator board (arduino) over ethernet messages sent by a ROS node on jetson tx1
* ROS can be run in a distributed setup, so ROS nodes on a remote PC can register on the ROS core on sepp´s jetson tx1
  * debugging and logging of ros messages is possible on a remote machine
  * visualizations of the camera streams and input and output signals is possible on a remote machine
* teleoperation is implemented in a simple manner
  * video streams of rgb camera (backwards) and 3D camera (to the front) ist sent to ROS core on jetson tx1 over Wifi
  * a ROS node which reads joystick input on a remote pc sends steering information to actuator board which controls the servos
  * a simple collition detection is implemented which reads ultrasonic signals and triggers a ermergancy stop if an obstacle is coming in the way of sepp
  
As you can see the real autonomous functions have to be implemented. Some work has to be done ;)

## Possible next steps
* activate buzzer while driving, so nobody falls over sepp
* implement a mode manager for all modes sepp can be in
* try to think about an architecture of ROS nodes to implement sensor fusion, steering of the car and autonomous functions
* think about debugging in ROS nodes running on the jetson tx1
* simple lane depature warning done by computer vision -> can be extended to lane follow function
* do a "trained drive" on a known parkour: train TensorFlow with data of humans driving the parkour

more to follow...
