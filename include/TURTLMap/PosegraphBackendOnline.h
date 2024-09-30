// a ros backend node
// have a graph AUVPoseGraph
// multiple subscribers and publishers

#include <ros/ros.h>
#include "TURTLMap/Posegraph.h"
#include <string>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>          // std::mutex
#include <condition_variable> // std::condition_variable
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <turtlmap/save_trajectory.h>
#include <waterlinked_a50_ros_driver/DVL.h>


namespace pose_graph_backend
{
    class PosegraphBackendOnline
    {
    private:
        AUVPoseGraph *posegraph_;

        // define subscribers
        ros::Subscriber imu_subscriber_;
        ros::Subscriber dvl_subscriber_;
        ros::Subscriber baro_subscriber_;
        ros::Subscriber dvl_local_subscriber_;

        ros::ServiceServer save_traj_service_;
        bool save_trajctory_(turtlmap::save_trajectory::Request &req,
                            turtlmap::save_trajectory::Response &res);
        std::vector<double> kf_timestamps_;
        std::vector<double> traj_timestamps_;
        std::vector<gtsam::Pose3> traj_poses_;
        // TODO: orbslam subscriber

        // define publishers
        ros::Publisher pose_publisher_;
        ros::Publisher path_publisher_;
        nav_msgs::Path path_msg_;
        ros::Publisher dvl_local_pose_publisher_;
        // static tf::TransformBroadcaster tf_broadcaster_;

        // define spinners separte spinner for IMU and others
        std::unique_ptr<ros::AsyncSpinner> imu_async_spinner_;
        std::unique_ptr<ros::AsyncSpinner> async_spinner_;

        // define queues
        ros::CallbackQueue imu_queue_;

        std::int64_t frame_count_;

        // subscriber callback 
        void callbackIMU(const sensor_msgs::ImuConstPtr& imu_msg);
        // void callbackDVL(const geometry_msgs::TwistWithCovarianceStampedConstPtr& dvl_msg);
        void callbackDVL(const waterlinked_a50_ros_driver::DVLConstPtr& dvl_msg); // switch to use the official DVL msg
        void callbackBaro(const sensor_msgs::FluidPressureConstPtr& baro_msg);
        void callbackDVLLocal(const geometry_msgs::PoseWithCovarianceStampedConstPtr& dvl_local_msg);
        // TODO: orbslam subscriber callback

        // TODO: think about the strategy for publishing state

        // TODO: thread for the main loop (pose graph optimization)
        void mainLoop();

        void kfLoop();

        // node handle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;

        double first_depth_ = 0.0;
        double prev_dvl_time_ = 0.0;
        double prev_dvl_local_time_ = 0.0;
        double current_kf_time_ = 0.0;
        double prev_kf_time_ = 0.0;
        bool new_kf_flag_ = false;
        // gtsam::Rot3 prev_dvl_local_rot_;
        gtsam::Pose3 prev_dvl_local_pose_;
        gtsam::Pose3 T_w_wd_;
        double kf_gap_time_;

        std::mutex mtx_;

        std::condition_variable cv_; // condition variable for the kf loop

        bool is_using_dvl_v2_factor = true; // TODO: get this from config file

        bool is_rot_initialized_ = false; // indicate if the
        gtsam::Rot3 imu_latest_rot_;
        gtsam::Rot3 dvl_prev_rot_;
        // std::vector<gtsam::Rot3> imu_rot_list_;

        
        int imu_init_count_ = 0;
        std::vector<gtsam::Vector3> imu_init_acc_;
        std::vector<gtsam::Vector3> imu_init_rot_;

        // latest kf pose
        gtsam::Pose3 latest_kf_pose_;
        gtsam::Pose3 latest_publish_pose_;

        // the first DVL measurement - used for prior velocity
        gtsam::Vector3 first_dvl_vel_;

        // adding smoother
        double first_kf_time_;

        // another pim between DVL measurements
        gtsam::PreintegratedCombinedMeasurements *pim_dvl_;
        gtsam::Vector3 latest_dvl_vel_;
        gtsam::Pose3 latest_dvl_pose_;
        int imu_count_ = 0;
        gtsam::NavState latest_imu_prop_state_;
        

    public:
        PosegraphBackendOnline(/* args */);
        ~PosegraphBackendOnline();
        PosegraphBackendOnline(std::string config_file);

    };
} // namespace pose_graph_backend
