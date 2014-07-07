PR2_Kinect2.0
=============

Teleoperation of PR2

 A topic I have always been interested is how effective humans are at directly controlling robots. 
 For example, when search-and-rescue robots are tested, it often takes a very long time to even open a door, as the operators have trouble perceiving the environment, and controlling the robot effectively. 
 To this end, I wanted to map the human joint movement onto a robot's arms, and see how easy it is to control a PR2. Turns out, not quite as easy as I had hoped. 
 Firstly, our system consists of a Kinect 2.0 that feeds skeleton tracking data via rosserial_windows. This must be done as the Kinect 2.0 drivers are currently Windows 8(!) only. 
 The skeleton tracking data looks similar to the following, and is provided in a stream of data points each with an X,Y,Z component in 3D space.
 
 For a full write up, check out:
 http://www.clearpathrobotics.com/blog/grand-opening-pr2-ribbon-cutting/