import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(data):
    print(data.pose.position.x, data.pose.position.y, data.pose.position.z)
    print(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
    print('---')
    global f
    f.write(f"{data.header.stamp.to_sec()} {data.pose.position.x} {data.pose.position.y} {data.pose.position.z} {data.pose.orientation.x} {data.pose.orientation.y} {data.pose.orientation.z} {data.pose.orientation.w}\n")

if __name__ == '__main__':
    rospy.init_node('subscribe_dvl_local_pose')
    rospy.Subscriber('dvl_local_pose', PoseStamped, callback)
    # write to a txt file
    # timestamp tx ty tz qx qy qz qw
    global f
    f = open("/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/dvl_local.txt", 'w')
    
    rospy.spin()



# def callback(data):
#     print(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
#     print(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
#     print('---')
#     global f
#     f.write(f"{data.header.stamp.to_sec()} {data.pose.pose.position.x} {data.pose.pose.position.y} {data.pose.pose.position.z} {data.pose.pose.orientation.x} {data.pose.pose.orientation.y} {data.pose.pose.orientation.z} {data.pose.pose.orientation.w}\n")


# if __name__ == '__main__':
#     rospy.init_node('subscribe_dvl_local_pose')
#     rospy.Subscriber('state', PoseWithCovarianceStamped, callback)
#     # write to a txt file
#     # timestamp tx ty tz qx qy qz qw
#     global f
#     f = open("/home/jingyu/frog/onr_slam_ws/src/onr_posegraph_backend_online/logs_eval/ukf.txt", 'w')
    
#     rospy.spin()