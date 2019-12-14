# 662-Final-Project-UR-5-arm

The Repository holds the .zip file of the ros workspace used in generating the Project. 

------------------------------------------------------------------------------------------------------------------------
Steps to launch the robot Arm:
1. Clone the repository, and unzip the ros workspace at a new location, and follow the steps to setup a ros workspace, ros build. 

2. Once configured with ros, under the src files, use the sample.launch file in the src/ur5_robotiq_arm/launch/sample.launch

----------------------------------------------------------------------------------------------------------------------

To create the UR5 arm with the robotiq gripper:

1. Use the src/ur5_robotiq_description folder, which has the urdf.xacro file which combines the robot arm with the gripper.

2. Lauch the urdf.xacro file with changes if needed, in moveit configuration to generate a config by Moveit.
3. Follow the steps to create your own poses with the robot and plan the motion of the robot
4. Create the controllers.yaml file and the ur5_robotiq_moveit_controller_manager.launch.xml and add the appropriate changes as per the reference files provided in the repo.
5. Then create the sample.lauch file, to launch the robot arm onto the 3D simulator called Rviz and a real world simulator called Gazebo. 
6. Watch your robot arm move, in your defined positions with an advanced pathplanning. 

------------------------------------------------------------------------------------------------------------------------------

Forward kinematics are verification in MATLAB:

Explanation  of  the  MATLAB  code  for  validation  of  Forward  kinemat-
ics  for  the  UR5  has  been  uploaded  to  YouTube  on  the  channel  “Kulbir
Ahluwalia” and can be viewed at the link https://youtu.be/GjvW574I-Pc.
The video of the moving UR5 arm can be seen at https://youtu.be/ISjVmBUFw6Y.
