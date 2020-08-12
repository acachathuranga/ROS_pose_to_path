import rospy
from threading import Timer
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PathPublisher():

    def __init__(self, poseTopic, pathTopic, publishRate=0):
        """ Pose to Path Publisher

            :poseTopic: Subscribed pose topic (Message type : geometry_msgs/PoseWithCovarianceStamped)
            :pathTopic: Published path topic (Message type: nav_msgs/Path Message)
            :publishRate: (Optional) Rate to publish path message. If not provided, path will be published 
                                        at pose message rate
        """
        self.pub = rospy.Publisher(pathTopic, Path, queue_size=1000)

        if (publishRate==0):
            self.realtime = True
        else:
            self.realtime = False
            interval = 1 / publishRate       # Milliseconds
            self.timer = ClassTimer(self.publishPath, 2)

        self.path = Path()
        sub = rospy.Subscriber(poseTopic, PoseWithCovarianceStamped, self.poseCallback)

        print ("Publishing from\t Pose: /%s to Path: /%s" %(poseTopic, pathTopic))
    
    def poseCallback(self, data):
        self.path.header = data.header
        self.path.header.frame_id = 'odom'

        
        pose_ = PoseStamped()
        pose_.header = data.header
        pose_.header.frame_id = "odom"
        pose_.pose = data.pose.pose

        self.path.poses.append(pose_)

        if self.realtime:
            self.publishPath()

    def publishPath(self):
        self.pub.publish(self.path)

    def stop(self):
        self.timer.stop()

        

class ClassTimer():
        def __init__(self, callback, interval):
            self.callback = callback
            self.interval = interval
            self.timer = Timer(self.interval, self.cb)
            self.timer.start()

        def cb(self):
            self.callback()
            self.timer = Timer(self.interval, self.cb)
            self.timer.start()
    
        def stop(self, *args):
            self.timer.cancel()

    
        