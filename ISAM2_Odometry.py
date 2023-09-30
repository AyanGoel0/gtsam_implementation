#!/usr/bin/env python3

from __future__ import print_function

import gtsam
import numpy as np
import tf
import rospy
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# Import files for path planning
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32MultiArray

import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
# Create noise models
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.04, 0, 0.01]))
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0, 0, 0]))
last_stamp = 0 
time_interval = 2


class Odometry_gtsam_node:

    def __init__(self):

        rospy.init_node('odometry_gtsam_node', anonymous=True)

        self.odom_sub = rospy.Subscriber('/wheel_odom' , Odometry , self.odom_subscriber)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

        self.last_stamp = 0 
        self.change_time = False 
        self.rot = []
        self.odom_path  = Path()
        self.odom_path_publisher = rospy.Publisher('/odom_path', Path, queue_size=10)
        self.poses_x_publisher = rospy.Publisher('/input_array',   Float32MultiArray ,queue_size = 10)
        self.result_x_publisher = rospy.Publisher('/result_array', Float32MultiArray ,queue_size = 10)

    def report_on_progress(graph_1: gtsam.NonlinearFactorGraph,current_estimate_1: gtsam.Values,key_1: int):
        """Print and plot incremental progress of the robot for 2D Pose SLAM using iSAM2."""

        # Print the current estimates computed using iSAM2.
        print("*"*50 + f"\nInference after State {key_1+1}:\n")
        print(current_estimate_1)

        # Compute the marginals for all states in the graph.
        marginals = gtsam.Marginals(graph_1, current_estimate_1)

        # Plot the newly updated iSAM2 inference.
        fig = plt.figure(0)
        axes = fig.gca()
        plt.cla()

        i = 1
        while current_estimate_1.exists(i):
            gtsam_plot.plot_pose2(0, current_estimate_1.atPose2(i), 0.5, marginals.marginalCovariance(i))
            i += 1

        plt.axis('equal')
        axes.set_xlim(-1, 5)
        axes.set_ylim(-1, 3)
        plt.pause(1)

    def odom_subscriber(self ,msg):
        # print("yello")
        
        # print(msg.header.stamp.secs)
        
        if (msg.header.stamp.secs != self.last_stamp):
            
            rospy.loginfo("in the if fucntion")
            self.change_time = True
            self.last_stamp = msg.header.stamp.secs
            self.inital_x = msg.pose.pose.position.x
            self.inital_y = msg.pose.pose.position.x
            self.rot = [msg.pose.pose.orientation.x ,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w ]
            (roll , pitch , self.inital_yaw) = euler_from_quaternion(self.rot)
            
            self.poses_array_1 = [self.inital_x , self.inital_y , self.inital_yaw]
            self.current_time_subs = rospy.Time.now()


    def main(self): 
        """Main runner"""
        rate = rospy.Rate(5)
        listener_1 = tf.TransformListener()

        x = 0
        poses_array = []
        between_array = []
        last_poses = [0,0,0]
        result_x_values = []
        poses_x_values = []
        time_travelled = 130
        current_time= rospy.Time.now()
        b = Float32MultiArray()
        a = Float32MultiArray()
        differnce_poses = [0,0,0]
        previous_poses = [0,0,0]
        while not rospy.is_shutdown():

            rospy.loginfo("in the main while")
            try:
                # rospy.loginfo("in the transform function")
                (trans, rot) = listener_1.lookupTransform('/base_link','/odom',rospy.Time(0))
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
            if not poses:
            
                rospy.loginfo("at the origin")
                rospy.loginfo(poses)

            elif last_poses == poses :

                last_poses = poses
                rospy.loginfo("at rest")

            else:
                odometry = gtsam.Pose2(poses[0],poses[1],poses[2])

                initial = gtsam.Values()

                parameters = gtsam.ISAM2Params()
                parameters.setOptimizationParams(gtsam.ISAM2DoglegParams() ) 
                parameters.setRelinearizeThreshold(0.1)
                parameters.setRelinearizeSkip(1)
                parameters.setEvaluateNonlinearError(True)
                parameters.setEnablePartialRelinearizationCheck(True)
                isam = gtsam.ISAM2(parameters)
                graph.push_back(gtsam.PriorFactorPose2(1, priorMean, PRIOR_NOISE))

                initial.insert(1,gtsam.Pose2(0,0,0))   

                current_estimate = initial

                if self.change_time :

                    x += 1 
                    rospy.loginfo("in the gtsam if ")
                    # rospy.loginfo(x)
                    if poses == differnce_poses :
                        between_array.append(poses)
                    else:
                        differnce_poses = [(previous_poses[0]-poses[0]),(previous_poses[1]-poses[1]),(previous_poses[2]-poses[2])]
                        between_array.append(differnce_poses)
                        previous_poses = poses
                    poses_array.append(self.poses_array_1)


                    for i in range(len(poses_array)):

                        rospy.loginfo("in the for loop")
                        rospy.loginfo(i)
                        # rospy.loginfo(poses_array)
                        # rospy.loginfo(between_array)
                        odometry_1 = gtsam.Pose2(between_array[i][0],between_array[i][1],between_array[i][2])
                        inital_1 = gtsam.Pose2(poses_array[i][0], poses_array[i][1],poses_array[i][2])
                        rospy.loginfo(odometry_1)
                        rospy.loginfo(inital_1)
                        graph.push_back(gtsam.BetweenFactorPose2(i+1, i+2, odometry_1, ODOMETRY_NOISE))
                        # computed_estimate = current_estimate.atPose2(i+1).compose(inital_1)
                        # rospy.loginfo(graph)
                        initial.insert(i+2,inital_1)  
                        isam.update(graph,initial)
                        current_estimate = isam.calculateEstimate()
                        factor_graph = isam.getFactorsUnsafe()

                        # gtsam.plot2DFactorGraph(factor_graph, current_estimate)
                        gtsam.writeG2o(factor_graph , current_estimate , 'factor_graph.g2o')

                        # if i == 5 :
                        #     j = 0
                        #     while j != i :
                        #         rospy.loginfo("heyo")

                        #         self.report_on_progress(graph_1=graph , current_estimate_1=current_estimate , key_1=j )
                        #         j += 1
                        initial.clear()                                           

                    blank_path = Path()
                    blank_path.header.frame_id = "map"
                    blank_path.header.stamp = rospy.Time.now()
                    pose_2 = PoseStamped()
                    pose_2.header.frame_id="odom_custom"
                    pose_2.pose.position.x=0
                    pose_2.pose.position.y=0
                    pose_2.pose.position.z=0
                    pose_2.pose.orientation.x = 0
                    pose_2.pose.orientation.y=0
                    pose_2.pose.orientation.z=0
                    pose_2.pose.orientation.w=1  
                    blank_path.poses = []
                    self.odom_path_publisher.publish(blank_path)
                    
                    self.odom_path.poses = []
                    self.odom_path_publisher.publish(self.odom_path)


                        # 5. Calculate and print marginal covariances for all variables
                    for i in range (len(poses_array)):
                        results = current_estimate.atPose2(i+1)

                        result_odom_quaternion = quaternion_from_euler( 0 , 0 , results.theta())
                        custom_odom = Odometry()
                        custom_odom.header.stamp = rospy.Time.now()
                        custom_odom.header.frame_id = 'custom_odom'
                        custom_odom.child_frame_id = 'base_link'
                        custom_odom.pose.pose.position.x =    results.x()
                        custom_odom.pose.pose.position.y =    results.y()
                        custom_odom.pose.pose.orientation.x = result_odom_quaternion[0] 
                        custom_odom.pose.pose.orientation.y = result_odom_quaternion[1]
                        custom_odom.pose.pose.orientation.z = result_odom_quaternion[2] 
                        custom_odom.pose.pose.orientation.w = result_odom_quaternion[3]

                        self.odom_path.header.frame_id ="map"
                        self.odom_path.header.stamp = rospy.Time.now()
                        pose_1 = PoseStamped()
                        pose_1.header.frame_id="odom_custom"
                        pose_1.pose.position.x=results.x()
                        pose_1.pose.position.y=results.y()
                        pose_1.pose.orientation.x = result_odom_quaternion[0] # Example quaternion w component
                        pose_1.pose.orientation.y = result_odom_quaternion[1]
                        pose_1.pose.orientation.z = result_odom_quaternion[2] # Example quaternion w component
                        pose_1.pose.orientation.w = result_odom_quaternion[3]

                        self.odom_path.poses.append(pose_1)
                        self.odom_pub.publish(custom_odom)       
                        # rospy.loginfo(rospy.Time.now())
                        result_x_values.append(results.x())
                        poses_x_values.append(poses_array[i][0])

                    
                        # rospy.loginfo(results.theta())
                    if  ((rospy.Time.now() - current_time).to_sec())> 45:
                        rospy.loginfo("plotting values")
                        a.data = poses_x_values
                        b.data = result_x_values
                        self.poses_x_publisher.publish(a)
                        self.result_x_publisher.publish(b)

                    else:
                        poses_x_values = []
                        result_x_values = []

                    self.odom_path_publisher.publish(self.odom_path)
                    self.change_time = False
                    last_poses = poses

if __name__ == "__main__":
    os = Odometry_gtsam_node()
    os.main()
    rospy.spin()

# optimizationParams:                type:              ISAM2DoglegParams
# optimizationParams:                initialDelta:      1
# optimizationParams:                wildfireThreshold: 1e-05
# optimizationParams:                adaptationMode:    SEARCH_EACH_ITERATION
# relinearizeThreshold:              0.1
# relinearizeSkip:                   1
# enableRelinearization:             1
# evaluateNonlinearError:            1
# factorization:                     CHOLESKY
# cacheLinearizedFactors:            1
# enableDetailedResults:             0
# enablePartialRelinearizationCheck: 0
# findUnusedFactorSlots:             0
