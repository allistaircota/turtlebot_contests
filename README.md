# TurtleBot Contests
## MIE443H1S: Mechatronics Systems: Design and Integration (Group 2, Winter 2017)

This is the repository for the contest code for MIE443H1: Mechatronics Systems: Design and Integration (Group 2, Winter 2017).

The course consisted of three contests where teams program the open-source TurtleBot robotic kit to perform tasks such as mapping an unknown environment, identifying objects of interest, and interaction with humans.

## Team Members

- Allisatir Cota
- Jamal Stetieh
- Kaicheng Zhang
- Samson Chiu

## Hardware

- TurtleBot2
  - iClebo Kobuki Base
  - Microsoft Kinect camera
  - See full specs at https://www.turtlebot.com/turtlebot2/

## Contest Implementation

- **Operating system:** Ubuntu, Robotic Operating System (ROS)
- **Languages:** C++
- **Libraries/Packages:** gmapping, RViz, OpenCV

## Contest 1: Autonomous Robot Search of an Environment

The TurtleBot was programmed to autonomously drive around an unknown environment while using the ROS gmapping library to dynamically produce a map of the environment using the Kinect sensor and wheel odometry. The gmapping package utilizes simultaneous localization and mapping (SLAM) to produce the map. The team had to devise an algorithm that would enable the robot to explore as much of the environment as possible. While every corridor and entry in the environment maze was captured by the map, a weakness of the team's algorithm did not allow the complete capture of very unique objects such as the round trashcans that were placed in the maze. Another lesson learned was that the maze did not have to be traversed so many times as significant drift was eventually captured due to the error associated with the wheel odometry method.

## Contest 2: Finding Objects of Interest in an Environment

The TurtleBot was programmed to navigate an environment to find and identify the image tags on five boxes placed at different locations in the environment. The robot was first localized using the "2D Pose Estimate" feature on RViz, through which the user inputs the robot's starting pose in a known map and then uses Monte Carlo Localization to localize the robot while the user manually pushes the robot along the boundaries of the environment. The coordinates of the object were provided by the teaching assistant in a text file at the contest itself, hence the team ensured the code was robust enough to navigate to any arbitrary set of coordinates.

To minimize the time taken to complete the contest, the code contained an implementation of the Nearest Neighbor Algorithm to determine the order of the boxes to be visited by the TurtleBot. For object identification, the OpenCV SURF feature detection library was used to capture the appropriate frame from the Kinect video feed and identify the tags, and their identities were printed to the laptop terminal. This contest was a success as the TurtleBot correctly identified all the image tags, and also recorded the joint fastest time among the class (1 min 20 seconds) which was well under the maximum time limit of 5 minutes.

See a video of a practice run: https://youtu.be/EfZgIFzAAWA

## Contest 3: Follow Me Robot Companion

This contest involved programming an interactive TurtleBot. The TurtleBot has to follow a member of the team while he/she moved within the environment. In addition, the TurtleBot had to interact with the user via emotions, specifically two primary/reactive emotions and two secondary/deliberative emotions. Each emotion would be shown in response to its distinct stimulus. A panel of judges (comprising of the course instructor and teaching assistants) had to be able to guess each emotion during the contest for full marks. Teams were allowed to use a variety input and output modes, so long as they utilized the standard hardware (no external hardware was allowed).
 
The team used the Kinect depth data to determine object to robot distance, along with the Kinect video feed and Open CV SURF feature detection library to perform image identification. In addition, the Kobuki Base bumpers were utilized to detect stimulus. In terms of outputs, velocity commands to the Kobuki base, audio and emoticon images were used to show each emotion. Being inspired by R2-D2 from Star Wars and the real world commercial pet robot AIBO, the team decided to give the TurtleBot its unique "voice" by developing a tone using the Caustic synthesizer, and playing different note patterns with the same synthesizer settings to distinguish between the sound for each emotion. In addition, the emoticon facial expressions were modified subtly to convey the change in emotion. The team got full marks for the contest implementation but acknowledges that the more complex architectures could be designed given more time.

See a video of the contest run: https://youtu.be/6V_B-d3TWco
