I have used Tutlebot3(Burger)for this cafe simulation
The cafe environment is designed in gazebo.



Steps involved in the simulation
1) Designing the cafe environment in gazebo using model editor and build editor.World file is being created and it is saved in our workspace.
2) The turtlebot3 package is cloned into  the ros workspace.
3) The robot model file and the world file are launched simultaneously from a single launch file.
4) The robot is spawned into the gazebo environment by using gazebo_ros package.
5) Joint  space publisher package is used to define the state of the joints and it publishes Jointstate messages.
6) Robot space publisher package is used to publish the 3D poses of the robot links.

Teleoperation is  done on the robot to check the mobility of the robot in the cafe environment. This is done using teleop_twist_keyboard package.

Once the teleoperation is finished, gmapping pacakage is installed to perform SLAM.

With the help of Rviz, map is created which produces two files from it.
   1) YAML File
   2) .pgm file(which has the image of the cafe map)


 AMCL package is installed for the localization of the robot in the map.
   1) This pacakge works on the particle filter to localize the robot in the known map.
   2) It takes account of laser scans messages and outputs the pose of the robot.

move_base pacakge is used to navigate the robot in the desred map.
  1) It also uses Global Planner and a Local Planner to achieve the naviagtion.
  2) It publish the cmd_vel messages to the base controller which controls the mobility of the robot.
  3) move_base_simple/goal topic is subscribed and the orientation and the pose of the goal is obtained through this topic.



In this simulation, /order topic is created for the waypoints of the respected table1,table2,table3 and the kitchen and it is published to move_simple_base/goal topic  to perform the naviagtion operation.

![Screenshot from 2024-09-03 08-01-53](https://github.com/user-attachments/assets/1a1c38c0-9ccf-459c-80d7-8f53b4ad3319)



      ![Screenshot from 2024-09-03 08-02-02](https://github.com/user-attachments/assets/d9e3ad44-cf45-45ce-a210-2769972990ab)


      ![WhatsApp Image 2024-09-03 at 7 50 41 AM](https://github.com/user-attachments/assets/606f4bfb-b7a2-4543-9e0f-14e88d1e3b63)


