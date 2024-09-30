// cretae a posegraph backend

#include "TURTLMap/PosegraphBackendOnline.h"

// print thread id
#include <thread>



namespace pose_graph_backend
{
    PosegraphBackendOnline::PosegraphBackendOnline(/* args */)
    {
        posegraph_ = new AUVPoseGraph();
    }

    PosegraphBackendOnline::~PosegraphBackendOnline()
    {
        // TODO: check if this is the right way to delete the posegraph

        if (imu_async_spinner_) imu_async_spinner_->stop();
        if (async_spinner_) async_spinner_->stop();

        delete posegraph_;

        // ros log
        ROS_INFO("PosegraphBackendOnline destroyed!");

    }

    bool PosegraphBackendOnline::save_trajctory_(turtlmap::save_trajectory::Request &req,
                            turtlmap::save_trajectory::Response &res)
    {
        // save the trajectory to a file
        std::string pose_file_name = req.trajectory_filename + ".txt";
        std::string timestamp_file_name = req.trajectory_filename + "_timestamp.txt";

        assert (traj_timestamps_.size() == traj_poses_.size());

        std::ofstream pose_file;
        pose_file.open(pose_file_name);
        for (int i = 0; i < traj_poses_.size(); i++)
        {
            // save all the numbers not in scientific notation
            pose_file << std::fixed << std::setprecision(12) << traj_timestamps_[i] << " " << traj_poses_[i].translation().x() << " " << traj_poses_[i].translation().y() << " " << traj_poses_[i].translation().z() << " " << traj_poses_[i].rotation().toQuaternion().w() << " " << traj_poses_[i].rotation().toQuaternion().x() << " " << traj_poses_[i].rotation().toQuaternion().y() << " " << traj_poses_[i].rotation().toQuaternion().z() << std::endl;
        }

        res.result = true;
        res.info = "Trajectory saved to " + pose_file_name + " and " + timestamp_file_name;

        return true;
    }
    
    PosegraphBackendOnline::PosegraphBackendOnline(std::string config_file)
    {
        nh_private_ = ros::NodeHandle("~");
        // record current time with ros::Time::now()
        // ros::Time::now() returns a Time object with the current time.

        // set use_sim_time to true
        nh_private_.setParam("use_sim_time", true);
        double start_time = 0.0;
        int init_count = 0;

        // used to get actual start time with use_sim_time true
        while (true)
        {
            start_time = ros::Time::now().toSec();
            init_count ++;
            if (start_time > 0 or init_count > 1000)
            {
                std::cout << "either start time is larger than 0 or init count is larger than 1000" << std::endl;
                break;
            }
            // sleep for 0.001s
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        // set precision
        std::cout.precision(17);
        std::cout << "PosegraphBackendOnline start time:  " << start_time << std::endl;

        // current_kf_time_ = start_time;
        // prev_kf_time_ = start_time;

        // std::cout << "prev_kf_time_: " << prev_kf_time_ << std::endl;

        posegraph_ = new AUVPoseGraph(config_file);

        // init pim_dvl_ to use the same params as pim_
        pim_dvl_ = new gtsam::PreintegratedCombinedMeasurements(boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params>(posegraph_->pim_params_), posegraph_->priorImuBias_);
        kf_gap_time_ = posegraph_->params_->kf_gap_time_;
        std::cout << "PosegraphBackendOnline created!" << std::endl;

        

        double graph_init_time = ros::Time::now().toSec();

        std::cout << "PosegraphBackendOnline graph init time: " << graph_init_time - start_time << std::endl;
        
        // init ros publishers
        pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("slam_pose", 1000);
        path_publisher_ = nh_.advertise<nav_msgs::Path>("slam_path", 1000);
        dvl_local_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("dvl_local_pose", 1000);

        save_traj_service_ = nh_.advertiseService("save_graph", &PosegraphBackendOnline::save_trajctory_, this);
        // pose_tf_publisher_ = nh_.advertise<geometry_msgs::TransformStamped>("tf", 1000);

        // init ros subscribers

        // IMU subscription
        static constexpr size_t kMaxImuQueueSize = 1000u;
        ros::SubscribeOptions imu_subscriber_options =
            ros::SubscribeOptions::create<sensor_msgs::Imu>(
                posegraph_->params_->sensor_topics_.imu_topic,
                kMaxImuQueueSize,
                boost::bind(&PosegraphBackendOnline::callbackIMU, this, _1),
                ros::VoidPtr(),
                &imu_queue_);

        

        imu_subscriber_options.transport_hints.tcpNoDelay(true);
        imu_subscriber_ = nh_.subscribe(imu_subscriber_options);

        // DVL subscription
        dvl_subscriber_ = nh_.subscribe(posegraph_->params_->sensor_topics_.dvl_topic, 1000, &PosegraphBackendOnline::callbackDVL, this);

        // Barometer subscription
        baro_subscriber_ = nh_.subscribe(posegraph_->params_->sensor_topics_.baro_topic, 1000, &PosegraphBackendOnline::callbackBaro, this);

        // DVL local subscription
        dvl_local_subscriber_ = nh_.subscribe(posegraph_->params_->sensor_topics_.dvl_local_position_topic, 1000, &PosegraphBackendOnline::callbackDVLLocal, this);

        // async spinners
        // For Orin nano there are 6 cores
        static constexpr size_t kImuSpinnerThreads = 2u;
        imu_async_spinner_ =
            std::make_unique<ros::AsyncSpinner>(kImuSpinnerThreads, &imu_queue_);
        
        static constexpr size_t kGlobalSpinnerThreads = 2u;
        async_spinner_ = std::make_unique<ros::AsyncSpinner>(kGlobalSpinnerThreads);

        imu_async_spinner_->start();
        async_spinner_->start();

        // create a thread for pose graph optimization
        // std::thread posegraph_thread(&PosegraphBackendOnline::mainLoop, this);


        // create a main kf thread for processing new kf
        std::thread kf_thread(&PosegraphBackendOnline::kfLoop, this); // kfloop: keyframe loop
        
        // TODO: though works for now but check Kimera implementation for this


        // spin
        // ros::spin();

        // while (ros::ok())
        // {
        //     ros::spinOnce();
        // }
        ros::waitForShutdown();
        // mtx_.unlock();
        // posegraph_thread.join(); // wait for the thread to finish
        // test_thread.join();
        kf_thread.join();

    }

    void PosegraphBackendOnline::kfLoop()
    {
        while (ros::ok()) // Jingyu TODO: find a way to deal with ctrl+c kill node situation
        {
            // std::cout << "kf loop thread id: " << std::this_thread::get_id() << std::endl;
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait(lk, [this]{return new_kf_flag_ == true;});
            posegraph_->index_++;
            // adding smoother
            
            kf_timestamps_.push_back(current_kf_time_);
            double time = current_kf_time_ - first_kf_time_;
            posegraph_->initial_->insert(gtsam::Symbol('d', posegraph_->index_), posegraph_->priorDvlBias_);
            // std::cout << "posegraph_->priorDvlBias_: " << posegraph_->priorDvlBias_ << std::endl;
            posegraph_->initial_->insert(gtsam::Symbol('b', posegraph_->index_), posegraph_->priorImuBias_);
            posegraph_->smootherTimestamps[gtsam::Symbol('d', posegraph_->index_)] = time;
            posegraph_->smootherTimestamps[gtsam::Symbol('b', posegraph_->index_)] = time;
            posegraph_->smootherTimestamps[gtsam::Symbol('x', posegraph_->index_)] = time;
            // posegraph_->smootherTimestamps[gtsam::Symbol('d', posegraph_->index_)] = posegraph_->index_;
            // posegraph_->smootherTimestamps[gtsam::Symbol('b', posegraph_->index_)] = posegraph_->index_;
            // posegraph_->smootherTimestamps[gtsam::Symbol('x', posegraph_->index_)] = posegraph_->index_;
            // std::cout << "cv receive new kf flag is true" <<  std::endl;

            // start to process new kf
            // new_kf_flag_ = false;
            // add different factors to the graph
            if (posegraph_->index_ < 10000)
            posegraph_->addBarometricFactor(posegraph_->getDepthMeasurement(), 0.005, posegraph_->index_);

            // add IMU factor
            
            gtsam::NavState prop_state;
            
            // use previous pose to do a prediction
            // prop_state = posegraph_->pim_->predict(gtsam::NavState(posegraph_->result_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_ - 1)),
            //                                                      posegraph_->result_->at<gtsam::Vector3>(gtsam::Symbol('v', posegraph_->index_ - 1))),
            //                                                      posegraph_->result_->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', posegraph_->index_ - 1))); // use latest bias
            // std::cout << "prev_pose_: " << posegraph_->initial_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_ - 1)) << std::endl;
            // std::cout << "prop_state: " << prop_state << std::endl;
            // std::cout << "dvl velocity reading: " << prop_state.R() * posegraph_->current_dvl_vels_.back() << std::endl;
            if (posegraph_->index_ > 1)
            {
                posegraph_->initial_->insert(gtsam::Symbol('v', posegraph_->index_), posegraph_->result_->at<gtsam::Vector3>(gtsam::Symbol('v', posegraph_->index_-1)));
                posegraph_->smootherTimestamps[gtsam::Symbol('v', posegraph_->index_)] = time;
                // posegraph_->smootherTimestamps[gtsam::Symbol('v', posegraph_->index_)] = posegraph_->index_;
            }
            else
            {
                posegraph_->initial_->insert(gtsam::Symbol('v', posegraph_->index_), gtsam::Vector3(0, 0, 0));
                posegraph_->smootherTimestamps[gtsam::Symbol('v', posegraph_->index_)] = time;
                // posegraph_->smootherTimestamps[gtsam::Symbol('v', posegraph_->index_)] = posegraph_->index_;
            }
            
            posegraph_->addImuFactor();
            if (is_using_dvl_v2_factor)
            {
                // std::cout << "Using DVL V2 factor!" << std::endl;
                posegraph_->addDvlFactorImuRot(); // use V2 dvl factor and slerp
            }
            else
            {
                // c++ exception
                throw std::runtime_error("Not implemented yet!");
            }
            // Jingyu TODO: add IMU factor and visual factor
            
            if (posegraph_->index_ > 0)
            {
                std::cout << "trying to optimize" << std::endl;
                

                if (!posegraph_->params_->using_smoother_)
                {
                    posegraph_->optimizePoseGraph();
                }
                else
                {
                    std::cout << "using smoother" << std::endl;
                    // count the time needed
                    // double start_time = ros::Time::now().toSec();
                    posegraph_->optimizePoseGraphSmoother();
                    // double end_time = ros::Time::now().toSec();
                    // std::cout << "Smoother optimization time: " << end_time - start_time << std::endl;
                }
                
                // debug use
                // posegraph_->pim_->print("after optimization pim: ");

                posegraph_->priorImuBias_ = posegraph_->result_->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', posegraph_->index_));
                std::cout << "IMU bias: " << posegraph_->priorImuBias_ << std::endl;

                posegraph_->pim_->resetIntegrationAndSetBias(posegraph_->priorImuBias_);

                //TODO: when invalid DVL measurement, don't reset the integration
                pim_dvl_->resetIntegrationAndSetBias(posegraph_->priorImuBias_);

                // mtx_.unlock();

                // std::cout << "Unlocked" << std::endl;


                latest_kf_pose_ = posegraph_->result_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_));
                std::cout << "latest_kf_pose_: " << latest_kf_pose_ << std::endl;
                latest_publish_pose_ = latest_kf_pose_;
                // TODO: jingyu: go through the logic of pim_dvl
                latest_dvl_pose_ = latest_kf_pose_;
                latest_dvl_vel_ = posegraph_->result_->at<gtsam::Vector3>(gtsam::Symbol('v', posegraph_->index_));

                // get the optimized result
                posegraph_->prev_pose_ = posegraph_->result_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_));
                if (is_using_dvl_v2_factor)
                {
                    posegraph_->priorDvlBias_ = posegraph_->result_->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('d', posegraph_->index_));
                    std::cout << "DVL bias: " << posegraph_->priorDvlBias_ << std::endl;
                    // get DVL bias in Point3
                    gtsam::Point3 dvl_bias_point3 = posegraph_->priorDvlBias_.accelerometer();
                    dvl_bias_point3 = posegraph_->prev_pose_.rotation() * dvl_bias_point3;
                    std::cout << "DVL bias in world frame: " << dvl_bias_point3 << std::endl;
                    posegraph_->pvm_->resetIntegrationAndBias(posegraph_->priorDvlBias_);
                }
                else
                {
                    // c++ exception
                    throw std::runtime_error("Not implemented yet!");
                }
            }
            
            posegraph_->prev_kf_time_ = current_kf_time_;
            
            

            new_kf_flag_ = false;
            lk.unlock();
            // std::cout << "kf loop unlock " << std::this_thread::get_id() << std::endl;
            cv_.notify_one();

        }
    }


    void PosegraphBackendOnline::mainLoop()
    {
        while (ros::ok())
        {
            // std::cout << "keep running main loop" << std::endl;
            // TODO: add mutex for locking kf

            // TODO: condition variable - std::cv to avoid keep running while loop
            if (new_kf_flag_ == true)
            {
                std::cout << "New KF flag is true!" << std::endl;
                // std::cout << "PosegraphBackendOnline main loop thread id: " << std::this_thread::get_id() << std::endl;
                new_kf_flag_ = false;
                posegraph_->addBarometricFactor(posegraph_->getDepthMeasurement(), 0.1, posegraph_->index_);
                if (is_using_dvl_v2_factor)
                {
                    // std::cout << "Using DVL V2 factor!" << std::endl;
                    posegraph_->addDvlFactorV2(true); // use V2 dvl factor and slerp
                }
                else
                {
                    // c++ exception
                    throw std::runtime_error("Not implemented yet!");
                }
                
                if (posegraph_->index_ > 0)
                {
                    std::cout << "trying to optimize" << std::endl;
                    // posegraph_->optimizePoseGraph();
                    
                    // sleep for 1 second
                    mtx_.lock();
                    std::cout << "Locked" << std::endl;
                    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    posegraph_->optimizePoseGraph();
                    mtx_.unlock();

                    std::cout << "Unlocked" << std::endl;

                    latest_kf_pose_ = posegraph_->result_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_));
                    latest_publish_pose_ = latest_kf_pose_;

                    // get the optimized result
                    posegraph_->prev_pose_ = posegraph_->result_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_));
                    if (is_using_dvl_v2_factor)
                    {
                        posegraph_->priorDvlBias_ = posegraph_->result_->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('d', posegraph_->index_));
                        std::cout << "DVL bias: " << posegraph_->priorDvlBias_ << std::endl;
                        posegraph_->pvm_->resetIntegrationAndBias(posegraph_->priorDvlBias_);
                    }
                    else
                    {
                        // c++ exception
                        throw std::runtime_error("Not implemented yet!");
                    }
                }
                
                posegraph_->prev_kf_time_ = current_kf_time_;
                posegraph_->index_++;
                posegraph_->initial_->insert(gtsam::Symbol('d', posegraph_->index_), posegraph_->priorDvlBias_);
                
            }
            // sleep for 0.1s
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // ros::spinOnce();
        }
            // ros::spinOnce();
        // std::cout << "PosegraphBackendOnline main loop thread id: " << std::this_thread::get_id() << std::endl;
        // ros::spinOnce();
    }

    void PosegraphBackendOnline::callbackIMU(const sensor_msgs::ImuConstPtr& imu_msg)
    {
        // const std::lock_guard<std::mutex> lock(mtx_);
        
        // if is_rot_initialized_ is false, then initialize the rotation with the first IMU message
        if (!is_rot_initialized_)
        {

            // imu_init_acc_.push_back(gtsam::Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z));
            imu_init_count_++;

            // get the rotation from the first IMU message
            gtsam::Rot3 imu_rot = gtsam::Rot3(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
            // convert to euler angles
            imu_init_rot_.push_back(imu_rot.xyz());

        }
        else
        {
            const std::lock_guard<std::mutex> lock(mtx_);
            // TODO: use IMU preintegration if the new IMU quality is good
            // think about the best practice of adding the rotation

            // IMU use case 1
            // only keep the rotation measurement from IMU
            
            // get IMU msg
            gtsam::Vector3 imu_acc = gtsam::Vector3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
            gtsam::Vector3 imu_gyro = gtsam::Vector3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

            imu_latest_rot_ = gtsam::Rot3(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);

            // deal with the pim
            // compute dt_imu
            posegraph_->pim_->integrateMeasurement(imu_acc,
                                                   imu_gyro,
                                                   posegraph_->params_->imu_params_.dt_imu);
            pim_dvl_->integrateMeasurement(imu_acc,
                                           imu_gyro,
                                           posegraph_->params_->imu_params_.dt_imu);
            imu_count_++;
            
            if (imu_count_ >= 5)
            {
                latest_imu_prop_state_ = pim_dvl_->predict(gtsam::NavState(latest_dvl_pose_,
                                                                        latest_dvl_pose_.rotation()*latest_dvl_vel_),
                                                                        // latest_dvl_vel_),
                                                                        posegraph_->priorImuBias_);
                // publish the pose
                gtsam::Pose3 latest_pose = latest_imu_prop_state_.pose();
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = imu_msg->header.stamp;
                pose_msg.header.frame_id = "NED_imu";
                pose_msg.pose.position.x = latest_pose.translation().x();
                pose_msg.pose.position.y = latest_pose.translation().y();
                pose_msg.pose.position.z = latest_pose.translation().z();
                pose_msg.pose.orientation.w = latest_pose.rotation().toQuaternion().w();
                pose_msg.pose.orientation.x = latest_pose.rotation().toQuaternion().x();
                pose_msg.pose.orientation.y = latest_pose.rotation().toQuaternion().y();
                pose_msg.pose.orientation.z = latest_pose.rotation().toQuaternion().z();
                pose_publisher_.publish(pose_msg);

                static tf::TransformBroadcaster br;
                // also publish the transform
                // - Translation: [0.052, 0.006, 0.242]
                // - Rotation: in Quaternion [-0.495, 0.498, -0.503, 0.503]        
                gtsam::Pose3 T_sensor_zed = gtsam::Pose3(gtsam::Rot3(-0.495, 0.498, -0.503, 0.503), gtsam::Point3(0.052, 0.006, 0.242));
                gtsam::Pose3 latest_publish_pose_base_link = latest_pose * T_sensor_zed.inverse();
                geometry_msgs::TransformStamped pose_tf_msg;
                pose_tf_msg.header.stamp = imu_msg->header.stamp;
                pose_tf_msg.header.frame_id = "NED_imu";
                pose_tf_msg.child_frame_id = "base_link";
                pose_tf_msg.transform.translation.x = latest_publish_pose_base_link.translation().x();
                pose_tf_msg.transform.translation.y = latest_publish_pose_base_link.translation().y();
                pose_tf_msg.transform.translation.z = latest_publish_pose_base_link.translation().z();
                pose_tf_msg.transform.rotation.w = latest_publish_pose_base_link.rotation().toQuaternion().w();
                pose_tf_msg.transform.rotation.x = latest_publish_pose_base_link.rotation().toQuaternion().x();
                pose_tf_msg.transform.rotation.y = latest_publish_pose_base_link.rotation().toQuaternion().y();
                pose_tf_msg.transform.rotation.z = latest_publish_pose_base_link.rotation().toQuaternion().z();
                br.sendTransform(pose_tf_msg);
                imu_count_ = 0;
            }
            

        }


        
    }

    // void PosegraphBackendOnline::callbackDVL(const geometry_msgs::TwistWithCovarianceStampedConstPtr& dvl_msg)
    void PosegraphBackendOnline::callbackDVL(const waterlinked_a50_ros_driver::DVLConstPtr& dvl_msg)
    {
        if (is_rot_initialized_ == false)
        {

            return;
        }

        const std::lock_guard<std::mutex> lock(mtx_);
        // std::cout << "DVL callback thread id: " << std::this_thread::get_id() << std::endl;
        double dt_dvl;
        double dvl_current_time = dvl_msg->header.stamp.toSec();
        gtsam::Vector3 dvl_vel = posegraph_->T_SD_.block(0, 0, 3, 3) * 
                gtsam::Vector3(dvl_msg->velocity.x, dvl_msg->velocity.y, dvl_msg->velocity.z);
        double fom = dvl_msg->fom;
        bool is_valid = dvl_msg->velocity_valid;

        if (prev_dvl_time_ == 0.0)
        {
            prev_dvl_time_ = dvl_current_time;
            dt_dvl = dvl_current_time - current_kf_time_;
            dvl_prev_rot_ = imu_latest_rot_;
            // first_dvl_vel_ = posegraph_->T_SD_.block(0, 0, 3, 3) * gtsam::Vector3(dvl_msg->twist.twist.linear.x, dvl_msg->twist.twist.linear.y, dvl_msg->twist.twist.linear.z);
            first_dvl_vel_ = dvl_vel;
            
            
            posegraph_->initial_->insert(gtsam::Symbol('v', posegraph_->index_), first_dvl_vel_);
            gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());
            posegraph_->graph->add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v', 0), first_dvl_vel_, prior_vel_noise));


            // for pim_dvl_
            latest_dvl_vel_ = first_dvl_vel_;
            latest_dvl_pose_ = latest_publish_pose_;

            // set the initial factor
        }
        else
        {
            dt_dvl = dvl_current_time - prev_dvl_time_;
        }
        
        // gtsam::Vector3 dvl_vel(dvl_msg->twist.twist.linear.x, dvl_msg->twist.twist.linear.y, dvl_msg->twist.twist.linear.z);
        // rotate the velocity to sensor frame
        if (fom >= posegraph_->params_->dvl_fom_threshold_ && !is_valid)
        // if (false)
        {
            // use pim_dvl_ to integrate the velocity
            gtsam::NavState imu_prop_state = pim_dvl_->predict(gtsam::NavState(latest_dvl_pose_,
                                                                latest_dvl_pose_.rotation()*latest_dvl_vel_),
                                                                // latest_dvl_vel_),
                                                                posegraph_->priorImuBias_);
            gtsam::Vector3 old_dvl_vel = dvl_vel;
            dvl_vel = imu_prop_state.pose().rotation().inverse() * imu_prop_state.velocity(); // TODO: jingyu checked this bug
            
            // compute the difference
            gtsam::Vector3 diff = dvl_vel - old_dvl_vel;
            std::cout << "Invalid DVL measurement!!!!!!!!!!!!!!!!!!!!, diff: " << diff << std::endl;

            posegraph_->addDvlVelocity(dvl_current_time, dvl_vel);

            gtsam::Rot3 d_rot = dvl_prev_rot_.inverse() * imu_latest_rot_;
            gtsam::Point3 d_position = d_rot.matrix() * gtsam::Point3(dvl_vel.x() * dt_dvl, dvl_vel.y() * dt_dvl, dvl_vel.z() * dt_dvl);
            gtsam::Pose3 d_pose = gtsam::Pose3(d_rot, d_position);
            latest_publish_pose_ = latest_publish_pose_ * d_pose;

            // latest_publish_pose_ = imu_prop_state.pose();
            latest_dvl_vel_ = dvl_vel;
            latest_dvl_pose_ = latest_publish_pose_;
            pim_dvl_->resetIntegration();
            // do not update latest_dvl_pose_ and latest_dvl_vel_, do not reset the integration
        }
        else
        {
            posegraph_->addDvlVelocity(dvl_current_time, dvl_vel);
            latest_dvl_vel_ = dvl_vel;
            gtsam::Rot3 d_rot = dvl_prev_rot_.inverse() * imu_latest_rot_;
            gtsam::Point3 d_position = d_rot.matrix() * gtsam::Point3(dvl_vel.x() * dt_dvl, dvl_vel.y() * dt_dvl, dvl_vel.z() * dt_dvl);
            gtsam::Pose3 d_pose = gtsam::Pose3(d_rot, d_position);

            latest_publish_pose_ = latest_publish_pose_ * d_pose;
            latest_dvl_pose_ = latest_publish_pose_;
            
            pim_dvl_->resetIntegration(); // only reset the 
        }
        posegraph_->current_dvl_foms_.push_back(fom*4); // multiply by 4 to get the actual fom
        posegraph_->imu_rot_list_.push_back(imu_latest_rot_);
        prev_dvl_time_ = dvl_current_time;
        
        // posegraph_->addDvlVelocity(dvl_current_time, dvl_vel);
        // latest_dvl_vel_ = dvl_vel;
        // new practice: keep the IMU estimated rotation from the latest IMU message
        // append the list with relative rotation
        

        // publish in-between pose by integrating the velocity
        
        // gtsam::Rot3 d_rot = dvl_prev_rot_.inverse() * imu_latest_rot_;
        // gtsam::Point3 d_position = d_rot.matrix() * gtsam::Point3(dvl_vel.x() * dt_dvl, dvl_vel.y() * dt_dvl, dvl_vel.z() * dt_dvl);
        // gtsam::Pose3 d_pose = gtsam::Pose3(d_rot, d_position);

        // latest_publish_pose_ = latest_publish_pose_ * d_pose;
        // latest_dvl_pose_ = latest_publish_pose_;
        
        // pim_dvl_->resetIntegration(); // only reset the 

        // prop_state = posegraph_->pim_->predict(gtsam::NavState(posegraph_->initial_->at<gtsam::Pose3>(gtsam::Symbol('x', posegraph_->index_ - 1)),
        //                                                          posegraph_->initial_->at<gtsam::Vector3>(gtsam::Symbol('v', posegraph_->index_ - 1))),
        //                                                          posegraph_->initial_->at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', posegraph_->index_))); // use latest bias
        
        // latest_publish_pose_ = gtsam::Pose3(latest_state.pose().rotation(), latest_state.pose().translation());

        // use depth for latest publish pose
        // latest_publish_pose_ = gtsam::Pose3(latest_publish_pose_.rotation(), gtsam::Point3(latest_publish_pose_.translation().x(), latest_publish_pose_.translation().y(), posegraph_->getDepthMeasurement()));

        // don't use depth for latest publish pose
        // latest_publish_pose_ = gtsam::Pose3(latest_publish_pose_.rotation(), gtsam::Point3(latest_publish_pose_.translation().x(), latest_publish_pose_.translation().y(), latest_publish_pose_.translation().z()));

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = dvl_msg->header.stamp;
        pose_msg.header.frame_id = "NED_imu";
        pose_msg.pose.position.x = latest_publish_pose_.translation().x();
        pose_msg.pose.position.y = latest_publish_pose_.translation().y();
        pose_msg.pose.position.z = latest_publish_pose_.translation().z();
        pose_msg.pose.orientation.w = latest_publish_pose_.rotation().toQuaternion().w();
        pose_msg.pose.orientation.x = latest_publish_pose_.rotation().toQuaternion().x();
        pose_msg.pose.orientation.y = latest_publish_pose_.rotation().toQuaternion().y();
        pose_msg.pose.orientation.z = latest_publish_pose_.rotation().toQuaternion().z();
        pose_publisher_.publish(pose_msg);

        static tf::TransformBroadcaster br;
        // also publish the transform
        // - Translation: [0.052, 0.006, 0.242]
        // - Rotation: in Quaternion [-0.495, 0.498, -0.503, 0.503]        
        gtsam::Pose3 T_sensor_zed = gtsam::Pose3(gtsam::Rot3(-0.495, 0.498, -0.503, 0.503), gtsam::Point3(0.052, 0.006, 0.242));
        gtsam::Pose3 latest_publish_pose_base_link = latest_publish_pose_ * T_sensor_zed.inverse();
        geometry_msgs::TransformStamped pose_tf_msg;
        pose_tf_msg.header.stamp = dvl_msg->header.stamp;
        pose_tf_msg.header.frame_id = "NED_imu";
        pose_tf_msg.child_frame_id = "base_link";
        pose_tf_msg.transform.translation.x = latest_publish_pose_base_link.translation().x();
        pose_tf_msg.transform.translation.y = latest_publish_pose_base_link.translation().y();
        pose_tf_msg.transform.translation.z = latest_publish_pose_base_link.translation().z();
        pose_tf_msg.transform.rotation.w = latest_publish_pose_base_link.rotation().toQuaternion().w();
        pose_tf_msg.transform.rotation.x = latest_publish_pose_base_link.rotation().toQuaternion().x();
        pose_tf_msg.transform.rotation.y = latest_publish_pose_base_link.rotation().toQuaternion().y();
        pose_tf_msg.transform.rotation.z = latest_publish_pose_base_link.rotation().toQuaternion().z();
        br.sendTransform(pose_tf_msg);

        traj_poses_.push_back(latest_publish_pose_base_link);
        traj_timestamps_.push_back(pose_tf_msg.header.stamp.toSec());




        // create tf msg for world and NED_imu
        // TODO Jingyu: add this to the launch file of the robot
        geometry_msgs::TransformStamped world_tf_msg;
        world_tf_msg.header.stamp = dvl_msg->header.stamp;
        world_tf_msg.header.frame_id = "world";
        world_tf_msg.child_frame_id = "NED_imu";
        world_tf_msg.transform.translation.x = 0.0;
        world_tf_msg.transform.translation.y = 0.0;
        world_tf_msg.transform.translation.z = 0.0;
        world_tf_msg.transform.rotation.w = 0.0;
        world_tf_msg.transform.rotation.x = 1.0;
        world_tf_msg.transform.rotation.y = 0.0;
        world_tf_msg.transform.rotation.z = 0.0;
        br.sendTransform(world_tf_msg);

        // create a path message
        path_msg_.header.stamp = dvl_msg->header.stamp;
        path_msg_.header.frame_id = "NED_imu";
        path_msg_.poses.push_back(pose_msg);
        path_publisher_.publish(path_msg_);



        // after publishing the pose
        dvl_prev_rot_ = imu_latest_rot_;

    }

    void PosegraphBackendOnline::callbackBaro(const sensor_msgs::FluidPressureConstPtr& baro_msg)
    {
        // if (is_rot_initialized_ == false)
        // {
        //     return;
        // }

        // receive the first baro message
        if (is_rot_initialized_ == false)
        {
            while (imu_init_rot_.size() < 5)
            {
                std::cout << "accumulating imu msgs" << std::endl;
            }
            // process the imu_init_rot_
            gtsam::Vector3 imu_rot_mean(0.0, 0.0, 0.0);
            int latest_imu_rot_number = 5;
            for (int i = imu_init_rot_.size()-latest_imu_rot_number; i < imu_init_rot_.size(); i++)
            {
                imu_rot_mean += imu_init_rot_[i];
            }
            imu_rot_mean /= latest_imu_rot_number;

            // get the rotation from the imu_rot_mean
            gtsam::Rot3 imu_rot = gtsam::Rot3::RzRyRx(imu_rot_mean[0], imu_rot_mean[1], imu_rot_mean[2]);
            
            std::cout << "imu_rot: " << imu_rot << std::endl;


            imu_latest_rot_ = imu_rot;
            posegraph_->imu_prev_rot_ = imu_rot;
            imu_init_rot_.clear();

            // initializePoseGraphFromImu
            posegraph_->initializePoseGraphFromImu(imu_rot); // current practice of initializing the pose graph with IMU
            current_kf_time_ = baro_msg->header.stamp.toSec();
            if (posegraph_->params_->using_smoother_)
            {
                posegraph_->smootherTimestamps[gtsam::Symbol('x', posegraph_->index_)] = 0.0;
                posegraph_->smootherTimestamps[gtsam::Symbol('v', posegraph_->index_)] = 0.0;
                posegraph_->smootherTimestamps[gtsam::Symbol('b', posegraph_->index_)] = 0.0;
                posegraph_->smootherTimestamps[gtsam::Symbol('d', posegraph_->index_)] = 0.0;
            }
            first_kf_time_ = current_kf_time_;
            posegraph_->prev_kf_time_ = current_kf_time_;
            kf_timestamps_.push_back(current_kf_time_);
            latest_kf_pose_ = posegraph_->initial_->at<gtsam::Pose3>(gtsam::Symbol('x', 0));
            latest_publish_pose_ = posegraph_->initial_->at<gtsam::Pose3>(gtsam::Symbol('x', 0));

            is_rot_initialized_ = true;

            double depth = (baro_msg->fluid_pressure - posegraph_->params_->baro_atm_pressure_) * 100 / 9.81 / 997.0; // TODO: add this to params
            first_depth_ = depth;
            std::cout << "First depth: " << first_depth_ << std::endl;
            return;
        }



        const std::lock_guard<std::mutex> lock(mtx_);

        // always add the latest barometer measurement to the graph
        double depth = (baro_msg->fluid_pressure - posegraph_->params_->baro_atm_pressure_) * 100 / 9.81 / 997.0; // TODO: add this to params
        if (first_depth_ == 0.0)
        {
            first_depth_ = depth;
            std::cout << "First depth: " << first_depth_ << std::endl;
        }
        posegraph_->setDepthMeasurement(depth - first_depth_);

        // TODO: due to low fps of baro msg we move KF check here
        double baro_current_time = baro_msg->header.stamp.toSec();
        if (baro_current_time - current_kf_time_ > kf_gap_time_)
        {
            // TODO: add IMU integrated pseudo DVL measurement
            if (posegraph_->params_->using_pseudo_dvl_)
            {
                // add pseudo DVL measurement
                gtsam::NavState pseudo_state = pim_dvl_->predict(gtsam::NavState(latest_dvl_pose_,
                                                                        latest_dvl_pose_.rotation()*latest_dvl_vel_),
                                                                        // latest_dvl_vel_),
                                                                        posegraph_->priorImuBias_);
                gtsam::Vector3 pseudo_dvl_vel = pseudo_state.pose().rotation().inverse() * pseudo_state.velocity();
                // gtsam::Vector3 pseudo_dvl_vel = pseudo_state.velocity();
                std::cout << "Pseudo DVL velocity: " << pseudo_dvl_vel << std::endl;
                std::cout << "latest DVL velocity: " << latest_dvl_vel_ << std::endl;
                std::cout << "---------------------" << std::endl;
                // TODO: check if we need to unrotate this velocity
                posegraph_->addDvlVelocity(baro_current_time, pseudo_dvl_vel);
                posegraph_->imu_rot_list_.push_back(imu_latest_rot_);
                posegraph_->current_dvl_foms_.push_back(0.02);
                prev_dvl_time_ = baro_current_time;
                                                                        
            }



            // posegraph_->current_dvl_timestamps_.back() += (baro_current_time - prev_dvl_time_);
            current_kf_time_ = baro_current_time;
            new_kf_flag_ = true;
            cv_.notify_one();
        }
    }

    void PosegraphBackendOnline::callbackDVLLocal(const geometry_msgs::PoseWithCovarianceStampedConstPtr& dvl_local_msg)
    {
        // TODO: think about the need of keeping this

        if (is_rot_initialized_ == false)
        {
            return;
        }

        const std::lock_guard<std::mutex> lock(mtx_);
        // std::cout << "DVL local callback thread id: " << std::this_thread::get_id() << std::endl;
        double dt_dvl_local;
        double dvl_local_current_time = dvl_local_msg->header.stamp.toSec();

        // TODO: get the relative transform only
        // get rotation
        
        gtsam::Pose3 dvl_local_pose = gtsam::Pose3(gtsam::Rot3(dvl_local_msg->pose.pose.orientation.w, dvl_local_msg->pose.pose.orientation.x, dvl_local_msg->pose.pose.orientation.y, dvl_local_msg->pose.pose.orientation.z), 
                gtsam::Point3(dvl_local_msg->pose.pose.position.x, dvl_local_msg->pose.pose.position.y, dvl_local_msg->pose.pose.position.z));
        if (prev_dvl_local_time_ == 0.0)
        {
            prev_dvl_local_time_ = dvl_local_current_time;
            dt_dvl_local = dvl_local_current_time - current_kf_time_;
            // prev_dvl_local_rot_ = gtsam::Rot3(dvl_local_msg->pose.pose.orientation.w, dvl_local_msg->pose.pose.orientation.x, dvl_local_msg->pose.pose.orientation.y, dvl_local_msg->pose.pose.orientation.z);
            // prev_dvl_local_pose_ = gtsam::Pose3(prev_dvl_local_rot_, gtsam::Point3(dvl_local_msg->pose.pose.position.x, dvl_local_msg->pose.pose.position.y, dvl_local_msg->pose.pose.position.z));
            prev_dvl_local_pose_ = dvl_local_pose;
            // compute the relative T
            // gtsam::Pose3 T_sd = gtsam::Pose3(posegraph_->T_SD_)
            T_w_wd_ = latest_publish_pose_ * gtsam::Pose3(posegraph_->T_SD_) * prev_dvl_local_pose_.inverse();
        }
        else
        {
            dt_dvl_local = dvl_local_current_time - prev_dvl_local_time_;
        }

        Eigen::Quaterniond q(dvl_local_msg->pose.pose.orientation.w, dvl_local_msg->pose.pose.orientation.x, dvl_local_msg->pose.pose.orientation.y, dvl_local_msg->pose.pose.orientation.z);
        // Convert to rotation matrix
        gtsam::Rot3 dvl_rot = gtsam::Rot3(q.normalized().toRotationMatrix());
        gtsam::Pose3 dvl_pose = gtsam::Pose3(dvl_rot, gtsam::Point3(dvl_local_msg->pose.pose.position.x, dvl_local_msg->pose.pose.position.y, dvl_local_msg->pose.pose.position.z));
        gtsam::Pose3 d_pose = prev_dvl_local_pose_.inverse() * dvl_pose;
        posegraph_->addDvlPose(dvl_local_current_time, d_pose);
        dvl_local_pose = T_w_wd_ * dvl_local_pose * gtsam::Pose3(posegraph_->T_SD_).inverse();
        // dvl_local_pose = gtsam::Pose3(dvl_local_pose.rotation(), gtsam::Point3(dvl_local_pose.translation().x(), dvl_local_pose.translation().y(), posegraph_->getDepthMeasurement()));

        
        gtsam::Pose3 T_sensor_zed = gtsam::Pose3(gtsam::Rot3(-0.495, 0.498, -0.503, 0.503), gtsam::Point3(0.052, 0.006, 0.242));
        dvl_local_pose = dvl_local_pose * T_sensor_zed.inverse();

        geometry_msgs::PoseStamped dvl_local_pose_msg;
        dvl_local_pose_msg.header.stamp = dvl_local_msg->header.stamp;
        dvl_local_pose_msg.header.frame_id = "NED_imu";
        dvl_local_pose_msg.pose.position.x = dvl_local_pose.translation().x();
        dvl_local_pose_msg.pose.position.y = dvl_local_pose.translation().y();
        dvl_local_pose_msg.pose.position.z = dvl_local_pose.translation().z();
        dvl_local_pose_msg.pose.orientation.w = dvl_local_pose.rotation().toQuaternion().w();
        dvl_local_pose_msg.pose.orientation.x = dvl_local_pose.rotation().toQuaternion().x();
        dvl_local_pose_msg.pose.orientation.y = dvl_local_pose.rotation().toQuaternion().y();
        dvl_local_pose_msg.pose.orientation.z = dvl_local_pose.rotation().toQuaternion().z();
        dvl_local_pose_publisher_.publish(dvl_local_pose_msg);
        

        prev_dvl_local_time_ = dvl_local_current_time;
        prev_dvl_local_pose_ = dvl_pose;

    }

} // namespace pose_graph_backend

