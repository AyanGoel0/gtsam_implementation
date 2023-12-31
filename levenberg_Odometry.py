#!/usr/bin/env python3

from __future__ import print_function

import gtsam
import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool

# Create noise models
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.09]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.01, 0.09]))
last_stamp = 0 


class Odometry_gtsam_node:

    def __init__(self):

        rospy.init_node('odometry_gtsam_node', anonymous=True)

        self.odom_sub = rospy.Subscriber('/wheel_odom' , Odometry , self.odom_subscriber)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.optimizer_pub = rospy.Publisher('/optimize_pub' , Bool , queue_size=10 )

        self.last_stamp = 0 
        self.change_time = False 
        self.rot = []

    def odom_subscriber(self ,msg):
        # print("yello")
        
        # print(msg.header.stamp.secs)
        
        if (msg.header.stamp.secs != self.last_stamp):
            
            rospy.loginfo("in the if function")
            self.change_time = True
            self.last_stamp = msg.header.stamp.secs
            self.inital_x = msg.pose.pose.position.x
            self.inital_y = msg.pose.pose.position.x
            self.rot = [msg.pose.pose.orientation.x ,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w ]
            (roll , pitch , self.inital_yaw) = euler_from_quaternion(self.rot)

            self.poses_array = [self.inital_x , self.inital_y , self.inital_yaw]
            

    def main(self):
        """Main runner"""
        rate = rospy.Rate(10)
        listener_1 = tf.TransformListener()

        x = 0               # For the key value 
        poses_array = []    # For the odom readings
        between_array = []  # For the tranforms value 
        last_poses = [0,0,0]

        while not rospy.is_shutdown():

            rospy.loginfo("in the main while")
            try:
                # rospy.loginfo("in the transform function")
                (trans, rot) = listener_1.lookupTransform('/odom','/base_link',rospy.Time(0))
                (roll , pitch ,yaw) = euler_from_quaternion(rot)
                poses = [trans[0] , trans[1] , yaw]

            except Exception as e:
                # print("exception in transform_pose : ", str(e))
                poses = []
            rate.sleep()


            # rospy.loginfo("printing the poses")
            # rospy.loginfo(poses)   

            graph = gtsam.NonlinearFactorGraph()
            priorMean = gtsam.Pose2(0.0, 0.0, 0.0)  # prior at origin

            initial = gtsam.Values()
            params = gtsam.LevenbergMarquardtParams()

            # Add a prior on the first pose, setting it to the origin
            # A prior factor consists of a mean and a noise model (covariance matrix)

            if not poses:
            
                rospy.loginfo("at the origin")
                rospy.loginfo(poses)

            elif last_poses == poses :

                last_poses = poses
                rospy.loginfo("at rest")

            else:

                odometry = gtsam.Pose2(poses[0],poses[1],poses[2])

                graph.add(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))
                initial.insert(1,gtsam.Pose2(0,0,0))      
                
                if self.change_time :

                    x += 1 
                    rospy.loginfo("in the gtsam if ")
                    rospy.loginfo(x)

                     
                    if x == 1 :

                        graph.add(gtsam.BetweenFactorPose2(x, x+1, odometry, ODOMETRY_NOISE))
                        initial.insert(x+1,gtsam.Pose2(self.inital_x , self.inital_y , self.inital_yaw))

                    else :

                        poses_array.append(self.poses_array)
                        between_array.append(poses)

                        for i in range(len(poses_array)):

                            rospy.loginfo("in the for loop")
                            rospy.loginfo(i)
                            odometry_1 = gtsam.Pose2(between_array[i][1],between_array[i][1],between_array[i][2])
                            inital_1 = gtsam.Pose2(poses_array[i][0], poses_array[i][1],poses_array[i][2])

                            graph.add(gtsam.BetweenFactorPose2(i+1, i+2, odometry_1, ODOMETRY_NOISE))
                            initial.insert(i+2,inital_1)


                    # print("\nInitial Estimate:\n{}".format(initial))
                    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
                    result = optimizer.optimize()
                    rospy.loginfo(result)
                    # 5. Calculate and print marginal covariances for all variables

                    marginals = gtsam.Marginals(graph, result)
                    message = Bool()
                    message.data = True
                    for i in range (len(poses_array)):
                        results = result.atPose2(i+1)

                        result_odom_quaternion = quaternion_from_euler( 0 , 0 , results.theta())
                        rospy.loginfo(result_odom_quaternion)
                        custom_odom = Odometry()
                        custom_odom.header.stamp = rospy.Time.now()
                        custom_odom.header.frame_id = 'custom_odom'
                        custom_odom.child_frame_id = 'base_link'

                        # Set your custom values
                        custom_odom.pose.pose.position.x =    results.x()
                        custom_odom.pose.pose.position.y =    results.y()  
                        custom_odom.pose.pose.orientation.x = result_odom_quaternion[0] 
                        custom_odom.pose.pose.orientation.y = result_odom_quaternion[1]
                        custom_odom.pose.pose.orientation.z = result_odom_quaternion[2] 
                        custom_odom.pose.pose.orientation.w = result_odom_quaternion[3]
                        # Publish the custom Odometry message
                        self.odom_pub.publish(custom_odom)                        
                        # print("result {}" , i)
                        # rospy.loginfo(results.theta())
                        message.data = False

        self.optimizer_pub.publish(message)
if __name__ == "__main__":
    os = Odometry_gtsam_node()
    os.main()
    rospy.spin()