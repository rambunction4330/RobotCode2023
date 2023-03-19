# RobotCode2023
This is the robot code for FRC team Rambunction 4330 for the 2023 
[*Charged UP*](https://youtu.be/0zpflsYc4PA) 
seaon. During this seson our team's alliance placed in 4th and won the 
Autonomus Award for our robots 100% siuccess rate in balancing during auto.

The project heavily relies on our teams own library, 
[librmb](https://github.com/rambunction4330/librmb), to handle lower level 
functionality such as motor control and drivtrain. It is incorperated into
this project as a vendor dependency.

The computer vision section of the 2023 robot code that ran on a RaspberryPi 
co-processor can be found
[here](https://github.com/rambunction4330/Vision2023).
It uses [AprilTags](https://github.com/AprilRobotics/apriltag)
to locate the robots position on the feild.

## Building
We uses gradle, as is standard in FRC, so the project can be built for testing with 
the command:
```
~$ ./gradlew build
```

Additionaly it deploys to the robot with:
```
~$ ./gradlew deploy
```