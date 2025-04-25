# EW458_FINAL_PROJECT
[Documentation](https://juanortiz28.github.io/EW458_FINAL_PROJECT/#/)

# Intro
Our names are Ivette Ayala and Juan Ortiz, and we are First Clas Midshipman (senior year students) at the United States Naval Academy. As we began the Spring Semester for the Academic Year 2025, we took the smartest decision on Earth and enrolled for PROG. & PLAN. FOR MOBILE ROBOTS (EW458.) This course explores modern robotic programming tools and their application in developing and implementing high level mobile robot applications. Topics include but are not limited to: mapping, autonomous navigation, algorithm design, logic and state machines. Various challenges are used to give students practical experience with using the techniques on robot hardware.

This website will run you through how we were able to accomplish our amazing Mapping Software that was able to map out Hopper Hall through the use of ROSLIBPY, Python, and HTML.

# Inspiration Section

We chose to use roslibpy because it allowed us to bridge the gap between powerful ROS tools and a lightweight, web-based interface for real-time mapping. Its flexibility enabled us to create a dynamic visualization of our environment—both inside our Hopper Hall classroom and in the surrounding spaces. By integrating LiDAR data and odometry through ROS topics, we were able to build an interactive occupancy grid map accessible from any browser. This solution not only made robotics more accessible but also showcased the power of remote collaboration and live spatial awareness.


# Dependencies
You will need the following softwares and libraries:
- Python
    - https://www.python.org/downloads/
- Roslibpy
    - https://www.python.org/downloads/

# Roslibpy Library Installation
Linux: `pip3 install roslibpy`
Windows: `pip installs roslibpy`

# Pseudocode for Map

```python
import roslibpy

#read scan topic
...

```

# How to run
``` bash
python3 FILENAME.py
```

# Example Maps
The following GIF was generated through running the Map Scanner program on the iRobot, and through the recording function of our professor's HTML program.

![Hopper Map](recording.gif)

# About Designers
pictures
 
