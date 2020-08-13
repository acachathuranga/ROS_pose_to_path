#!/usr/bin/python
import rospy
import sys
from path_publisher import PathPublisher
import signal

publishers = []

def pathPublisher():

    rospy.init_node('pose_to_path_publisher')
    signal.signal(signal.SIGINT, terminate)

    path = 0
    while True:
        try:
            config = rospy.get_param(rospy.get_name()+'/path' + str(path))
            if (len(config) == 2) or (len(config) == 3) or (len(config) == 4):
                publishers.append(PathPublisher(*config))
            else:
                print ("Invalid Input Configuration: ", config)
        except Exception as e:
            print ("Number of Paths: %d" %path)
            # rospy.logerr(str(e))
            break 
            
        path += 1

    rospy.spin()

def terminate(*args):
    print "User termination requested"

    for publisher in publishers:
        publisher.stop()
    sys.exit()

if __name__ == '__main__':
    try:
        pathPublisher()
    except rospy.ROSInterruptException:
        pass
    

