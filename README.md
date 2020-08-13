# Pose to Path Publisher

This package subscribes to a pose topic and combines all previous pose messages to publish a path message

### Subscribed Topics
- pose_topic : Subscribed pose topic

### Published Topics
- path_topic : Published path topic

### Parameters
- publish_rate : The rate at which path messages are published. (0 : Publish at sampling rate)
- sample_rate  : The rate at which pose messages will be sampled, to create the path. (0 : Sample at pose message rate)
                
##### Note that very high sampling rates will result in paths with very high number of points. This can sometimes cause Rviz crash
