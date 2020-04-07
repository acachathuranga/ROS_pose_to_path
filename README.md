# Python 3 - ROS Package Example

Python 3 is not officially supported by ROS as of now. However, ROS packages written with Python 3 can be run using the following method.

- Step 1 - Install following packages <br /> 
    -  ***sudo apt-get install python3-yaml***
    -  ***sudo pip3 install rospkg catkin_pkg***
- Step 2 - Set python script environment<br />
>> Insert Following line to the top of your python scripts to run them in python 3 environment.

>>`#!/usr/bin/python3.5`

Compile and run the package