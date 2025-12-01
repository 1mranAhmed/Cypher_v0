This is Cypher. An autonomous All Terrain Rover, designed to provide help and during Disasters, and decrease the respnse delay. 

![Cypher pic](https://github.com/user-attachments/assets/ba592bd5-c42a-43f9-aabb-30a0c96cdee7)


Features : 
  1. Autonomous Navigation
  2. All Terrain
  3. Surveillance and Neutralization
  4. Fire Extingushing
  5. Med-Kit Deployment
  6. Swappable Robotic Arm

Details:

   1. Autonomous Navigation:
   We panned to achieve autonomous Navigation using the SLAM Directory, with ROS2 Humble. The sensors that we were planning on using were 2 motor mounted magnetic encoders of 1300 CPR, an IMU, and a TFmini LiDAR. We were able to implement autonomous navigation on a predictable path. The predictable path was 1 metre forward, 1 metre backwards, 1 metre left and 1 metre right. The processing for this Odometry was done on the Arduino Due. The Arduino Due was chosen for its powerful processor (ARM Cortex M3) and large number of Interrupt capable pins.
    
   2. All Terrain:
  We used the Rocker-Bogie Mechanism developed by NASA to achieve the All Terrain capability. The main body was made using a sheet of Mild Steel 3mm thickness. It was laser cut to ensure precision. Though, the whole Rocker-Bogie Mechanism, which was implemented on the Mars Rover wasnt implemented. A basic Version of it was used instead. The basic Design was done on solidWorks. The image is attached Below:  
  ![Cypher all terrain mech](https://github.com/user-attachments/assets/d773615c-6c60-4213-b09d-6100e7f51c95)

   3. Surveillance and Neutralization:
      This was done using Python and OpenCV library. A model was trained to detect specific objects. The Python Script detected the object and calculated the error from the centre of the detected object to the center of the frame and sent this data to the Microcontroller via Serial Communication. The MicroController contolled the servos the track the detected object and neutralize the detected object.

   4. Fire Extinguisher: 
      The same Model was trained to detect Fires and actuate a Relay. The Fire Extinguisher system consisted of a Air Cannister, Solenoid Switch and Relay. Upon Detection of Fire, the Relay is Actuated and the supressant in the Air Cannister is released.   
  5. Med-Kit Deployment:
      The same Model was trained to detect an injured person. When detected, a servo is actuated to open and throw out a Med-kit for the injured Person.
    6. Swappable Robotic Arm: 
      This Feature is swappable with the neutralizing mechanism, to help assist civilians during disaster situations. 

The main idea was to integrate all of these Features using ROS2, but due to our current technical expretise, we werent able to achieve it. It is a Future goal for now, one we hope to achieve.
