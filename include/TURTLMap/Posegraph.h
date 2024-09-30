#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <TURTLMap/PoseGraphParameters.h>
#include <TURTLMap/BluerovBarometerFactor.h>
#include <TURTLMap/VelocityIntegrationFactor.h>

// for including the smoother
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>




#include <cstring>
#include <fstream>
#include <iostream>
#include <random>

#include <gtsam/navigation/AHRSFactor.h>

// TransformStamped
#include <geometry_msgs/TransformStamped.h>

namespace pose_graph_backend
{
    class AUVPoseGraph
    {
    public:
        AUVPoseGraph();
        AUVPoseGraph(std::string &config_file);
        ~AUVPoseGraph();

        int index_;

        // addition of smoother
        gtsam::ISAM2Params smootherParameters;
        double smootherLag = 6.0;
        // IncrementalFixedLagSmoother smootherISAM2;
        gtsam::BatchFixedLagSmoother smootherISAM2;
        FixedLagSmoother::KeyTimestampMap smootherTimestamps;
        void optimizePoseGraphSmoother();


        // GTSAM Related things
        gtsam::NonlinearFactorGraph *graph;
        gtsam::Values *initial_;
        gtsam::Values *result_;
        gtsam::PreintegratedCombinedMeasurements *pim_;
        // BlueRovPreintegratedVelocityMeasurements *pvm_;
        PreintegratedVelocityMeasurementsDvlOnly *pvm_;
        gtsam::PreintegratedCombinedMeasurements::Params *pim_params_;
        PoseGraphParameters *params_;

        // Prior imuBias
        gtsam::imuBias::ConstantBias priorImuBias_;
        gtsam::imuBias::ConstantBias priorDvlBias_ = gtsam::imuBias::ConstantBias(gtsam::Vector3(0.01, 0.01, 0.01), gtsam::Vector3(0, 0, 0));


        // Transforms
        gtsam::Matrix44 T_SD_;   // sensor (IMU)to DVL affine transform
        gtsam::Matrix44 T_SB_;   // sensor (IMU)to robot center affine transform
        gtsam::Matrix44 T_W_WD_; // world to DVL world transform
        // Previous
        gtsam::Pose3 prev_pose_;
        gtsam::Vector3 prev_vel_;
        gtsam::NavState prev_state_;
        double prev_kf_time_; // TODO: used for saving the previous keyframe time

        // Current
        gtsam::Pose3 current_pose_;
        gtsam::Vector3 current_vel_;
        gtsam::NavState current_state_;
        gtsam::imuBias::ConstantBias current_bias_;
        double current_time_;
        std::vector<gtsam::Vector3> current_dvl_vels_;
        std::vector<gtsam::Pose3> current_dvl_poses_;
        std::vector<gtsam::Rot3> current_dvl_rotations_;
        std::vector<double> current_dvl_foms_;
        std::vector<gtsam::Rot3> imu_rot_list_;
        gtsam::Rot3 imu_prev_rot_;

        std::vector<double> current_dvl_timestamps_;
        std::vector<double> current_dvl_local_timestamps_;

        void addBarometricFactor(double W_measurement_z, double measurement_noise, int baro_id);                   // Barometer
        void addImuFactor();                                                                               // IMU
        void addVelocityIntegrationFactor();                                                                  // DVL
        
        void addVelocityIntegrationFactorInterp(bool isUsingSlerp);                                        // Interpolate DVL rotations
        void addDvlFactorV2(bool isUsingSlerp);
        void addDvlFactorImuRot();

        double prevDvlOdomTime_ = 0.0;
        gtsam::Rot3 prevDvlOdomRot_ = gtsam::Rot3();
        
        void addVisualConstraintFactor(gtsam::Pose3 between_pose, double weight, int prev_idx, int curr_idx); // Stereo from ORBSlam as between
        void addSonarFactor();                                                                                // Sonar and DVL
        void addPriorFactor(gtsam::Pose3 initial_pose, gtsam::Vector initial_vel, double pose_noise);         // Prior
        void setImuParams();
        void defineTransforms();
        void addEstimateSimple(double dt, double noise);
        void addInitialEstimate(gtsam::Pose3 initial_pose, gtsam::Vector3 initial_vel);
        void addEstimateVisual(const std::vector<double> vertex);
        void initializePoseGraph();
        void initializePoseGraphFromImu(gtsam::Rot3 initial_rotation);

        void addDvlOdometryFactor(double noise);//Jingyu add for dvl odometry

        void optimizePoseGraph();

        void setDepthMeasurement(double W_measurement_z) { W_measurement_z_ = W_measurement_z; }
        void setAccelerometerMeasurement(gtsam::Vector3 B_accelerometer_S) { B_accelerometer_S_ = B_accelerometer_S; }
        void setGyroscopeMeasurement(gtsam::Vector3 B_gyroscope_S) { B_gyroscope_S_ = B_gyroscope_S; }
        void setVelocityMeasurement(gtsam::Vector3 B_velocity_D) { B_velocity_D_ = B_velocity_D; }
        void setPositionMeasurement(gtsam::Vector3 W_position_C) { W_position_C_ = W_position_C; }
        void setVisualGapTime(double visualGapTime) { visualGapTime_ = visualGapTime; /* in seconds*/ }

        double getDepthMeasurement() { return W_measurement_z_; }
        gtsam::Vector3 getAccelerometerMeasurement() { return B_accelerometer_S_; }
        gtsam::Vector3 getGyroscopeMeasurement() { return B_gyroscope_S_; }
        gtsam::Vector3 getVelocityMeasurement() { return B_velocity_D_; }
        gtsam::Vector3 getPositionMeasurement() { return W_position_C_; }
        double getVisualGapTime() { return visualGapTime_; }
        // gtsam::Pose3 findCurrentPoseForDvlVel(double timestamp);
        gtsam::Rot3 findCurrentPoseForDvlVel(double timestamp);

        void addDvlVelocity(double timeStamp, gtsam::Vector3 velocity);
        void addDvlPose(double timeStamp, gtsam::Pose3 pose);
        void addDvlRotation(double timeStamp, gtsam::Rot3 rotation);

        void addEdgesToGraph(const std::vector<std::vector<double>> &edges);
        std::vector<int> used_idx_;

    private:
        // Random number generator
        // Measurements
        std::mt19937 rng_;
        std::normal_distribution<> normal_distribution_;
        double visualGapTime_;
        double W_measurement_z_;           // world frame depth measurement in the world frame
        gtsam::Vector3 B_accelerometer_S_; // body frame accel in the imu frame
        gtsam::Vector3 B_gyroscope_S_;     // body fraome gyro in the imu frame
        gtsam::Vector3 B_velocity_D_;      // body frame velocity in the DVL frame
        gtsam::Vector3 W_position_C_;      // position in world frame measured by the camea
    };

}