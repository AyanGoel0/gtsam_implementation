#!/usr/bin/env python3

# creates marker at mentioned position , produces 4 paths for 4 diffrent odometry sources

# ---------------------------------------------------------------------------- #
#                              Importing libraries                             #
# ---------------------------------------------------------------------------- #
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import logging


# ---------------------------------------------------------------------------- #
#                        Publishing Visualization Marker                       #
# ---------------------------------------------------------------------------- #
def marker_cb():
    rospy.loginfo("Publishing markers...")
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    markers = []  # List to store multiple markers

    # Define positions and characters for each marker
    marker_info = [
        {"position": (0.00, 0.00, 0.0), "character": "A"},
        {"position": (0.06, 4.61, 0.0), "character": "B"},
        {"position": (2.22, 4.61, 0.0), "character": "C"},
        {"position": (5.29, -6.15, 0.0), "character": "D"},
        {"position": (2.22, -7.38, 0.0), "character": "E"},
        {"position": (-0.24, -7.19, 0.0), "character": "F"},
        {"position": (-0.22, 0.0, 0.0), "character": "a"},
        {"position": (-0.22,4.635, 0.0), "character": "b"},
        {"position": (2.25,4.944, 0.0), "character": "d"},
        {"position": (0.0*0.618-0.22, 8*0.618, 0.0), "character": "e"},
        {"position": (0.0*0.618-0.22, 1*0.618, 0.0), "character": "f"},
        {"position": (3.5*0.618-0.22, 1*0.618, 0.0), "character": "g"},
        {"position": (3.5*0.618-0.22,-6*0.618, 0.0), "character": "h"},
        {"position": (6.0*0.618-0.22, -6*0.618, 0.0), "character": "i"},
        {"position": (6.0*0.618-0.22,-10*0.618, 0.0), "character": "j"},
        {"position": (9.0*0.618-0.22,-9*0.618, 0.0), "character": "k"},
        {"position": (6.0*0.618-0.22,-9*0.618, 0.0), "character": "l"},
        {"position": (6.0*0.618-0.22,-7*0.618, 0.0), "character": "m"},
        {"position": (4.0*0.618-0.22,-7*0.618, 0.0), "character": "x"},
        {"position": (2.0*0.618-0.22,-7*0.618, 0.0), "character": "n"},
        {"position": (2.0*0.618-0.22,-11.5*0.618, 0.0), "character": "o"},
        {"position": (3.5*0.618-0.22,-7*0.618, 0.0), "character": "p"},
        {"position": (3.5*0.618-0.22,-2*0.618, 0.0), "character": "q"},
        {"position": (0.0*0.618-0.22,-2*0.618, 0.0), "character": "r"},
    ]

    for idx, info in enumerate(marker_info):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = info["position"][0]
        marker.pose.position.y = info["position"][1]
        marker.pose.position.z = info["position"][2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = info["character"]
        markers.append(marker)

    for marker in markers:
        marker_pub.publish(marker)   


# ---------------------------------------------------------------------------- #
#                               IMU Path Publish                               #
# ---------------------------------------------------------------------------- #
def imu_odom_callback(odom):
    rospy.loginfo("Received IMU odom, publishing Path")
    global imu_path
    imu_path_publisher = rospy.Publisher('/imu_path', Path, queue_size=2)

    imu_path.header.frame_id = "map"
    imu_path.header.stamp = rospy.Time.now()

    pose = PoseStamped()
    pose.header.frame_id="odom_custom"
    pose.pose.position=odom.pose.pose.position
    pose.pose.position.z=0
    pose.pose.orientation.x=0
    pose.pose.orientation.y=0
    pose.pose.orientation.z=0
    pose.pose.orientation.w=1  
    imu_path.poses.append(pose)
    imu_path_publisher.publish(imu_path)


# ---------------------------------------------------------------------------- #
#                          WHeel Encoder Path Publish                          #
# ---------------------------------------------------------------------------- #
def wheel_odom_callback(odom):
    rospy.loginfo("Received wheel odometry callback.")
    global wheel_path, i
    wheel_path_publisher = rospy.Publisher('/wheel_path', Path, queue_size=1)
    wheel_path.header.frame_id = "map"
    wheel_path.header.stamp = rospy.Time.now()

    pose = PoseStamped()
    pose.header.frame_id="odom"
    pose.pose.position=odom.pose.pose.position
    pose.pose.position.z=0
    pose.pose.orientation.x=0
    pose.pose.orientation.y=0
    pose.pose.orientation.z=0
    pose.pose.orientation.w=1 
    wheel_path.poses.append(pose)
    wheel_path_publisher.publish(wheel_path)


# ---------------------------------------------------------------------------- #
#                         Lidar localized Path Publish                         #
# ---------------------------------------------------------------------------- #
def als_odom_callback(pose_stamped):
        rospy.loginfo("Received ALS odometry callback.")
        global als_path
        als_path_publisher = rospy.Publisher('/als_path', Path, queue_size=1)

        als_path.header.frame_id = "map"
        wheel_path.header.stamp = rospy.Time.now()

        pose2 = PoseStamped()
        pose2.header.frame_id="map"
        pose2.pose.position=pose_stamped.pose.position
        pose2.pose.position.z=0
        pose2.pose.orientation.x=0
        pose2.pose.orientation.y=0
        pose2.pose.orientation.z=0
        pose2.pose.orientation.w=1
        als_path.poses.append(pose2)
        als_path_publisher.publish(als_path)



# ---------------------------------------------------------------------------- #
#                              Ideal Path Publish                              #
# ---------------------------------------------------------------------------- #
def ideal_odom_path():
        global ideal_path
        coordinates = [
        (0,0),                              #A
        (-0.22,0),                          #a
        (-0.22,4.635),                      #b
        (2.25,4.635),                       #C
        (2.25,4.944),                       #d
        (0.0*0.618-0.22, 8*0.618),          #e   
        (0.0*0.618-0.22, 1*0.618),          #f
        (3.5*0.618-0.22, 1*0.618),          #g
        (3.5*0.618-0.22,-6*0.618),          #h    
        (6.0*0.618-0.22, -6*0.618),         #i
        (6.0*0.618-0.22,-10*0.618),         #j 
        (5.29,-6.15),                       #D  
        (9.0*0.618-0.22,-9*0.618),          #k
        (6.0*0.618-0.22,-9*0.618),          #l
        (6.0*0.618-0.22,-7*0.618),          #m
        (4.0*0.618-0.22,-7*0.618),          #x
        (2.22, -7.38),                      #E 
        (4.0*0.618-0.22,-7*0.618),          #x
        (2.0*0.618-0.22,-7*0.618) ,         #n
        (2.0*0.618-0.22,-11.5*0.618),       #o
        (-0.24, -7.19),                     #F 
        (2.0*0.618-0.22,-11.5*0.618),       #o
        (2.0*0.618-0.22,-7*0.618) ,         #n
        (3.5*0.618-0.22,-7*0.618),          #p
        (3.5*0.618-0.22,-2*0.618),          #q
        (0.0*0.618-0.22,-2*0.618),          #r
        (-0.22,0)                           #a
        ]

        ideal_path.header.frame_id = "map"
        ideal_path.header.stamp = rospy.Time.now()

        for coord in coordinates:
            pose_stamped = PoseStamped()
            pose_stamped.header = ideal_path.header
            pose_stamped.pose.position.x = coord[0]
            pose_stamped.pose.position.y = coord[1]
            pose_stamped.pose.position.z = 0.0

            ideal_path.poses.append(pose_stamped)
        ideal_path_publisher.publish(ideal_path)
        rospy.loginfo("Generating ideal path...")

# ---------------------------------------------------------------------------- #
#                                 Main Function                                #
# ---------------------------------------------------------------------------- #

if __name__ == '__main__':

    rospy.init_node('path_generator_node', anonymous=True)
    log_file = "path_generator_log.log"
    log_level = logging.DEBUG
    logging.basicConfig(filename=log_file, level=log_level, format='%(asctime)s - %(levelname)s - %(message)s')

    wheel_path = Path()
    als_path = Path()
    ideal_path = Path()
    imu_path = Path()
    als_path_publisher = rospy.Publisher('/als_path', Path, queue_size=1)
    wheel_path_publisher = rospy.Publisher('/wheel_path', Path, queue_size=1)
    ideal_path_publisher = rospy.Publisher('/ideal_path', Path, queue_size=1)
    imu_path_publisher = rospy.Publisher('/imu_path', Path, queue_size=1)

    marker_cb()
    rospy.sleep(2)
    ideal_odom_path()
    rospy.sleep(2)

    while not rospy.is_shutdown():
        try:    
            rospy.Subscriber('/wheel_odom', Odometry, wheel_odom_callback)
            rospy.Subscriber('/odom', Odometry, imu_odom_callback)
            # rospy.Subscriber('/als_odom', PoseStamped, als_odom_callback)
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.logerr("ROS node interrupted.")
            pass