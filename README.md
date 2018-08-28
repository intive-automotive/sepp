# sepp
small size evaluation and prototyping platform - yet another autonomous RC car

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
* car body: bumpers, constructed aluminium frame, constructed 3D printed parts, plenty of tape, cable ties, jumper cable and solder :)


