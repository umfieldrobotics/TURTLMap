#include "TURTLMap/VelocityIntegrationFactor.h"
namespace pose_graph_backend
{
  // integrateMeasurements(const gtsam::Vector3 &linear_velocity,
                              //  const double &dt);
  void PreintegratedVelocityMeasurementsDvlOnly::integrateMeasurements(const gtsam::Vector3 &linear_velocity, const gtsam::Rot3 &interpolated_rot,
                              double &dt)
  {
    if (dt <= 0)
    {
      // throw std::runtime_error(
      //     "PreintegratedVelocityMeasurements::integrateMeasurement: dt <=0");
      std::cout << "Warning: dt <=0" << std::endl;
      dt = 0.0001; // Jingyu: added this to avoid the error
    }

    vel_list.push_back(linear_velocity);
    rot_list.push_back(interpolated_rot);
    dt_list.push_back(dt);
  }

  void PreintegratedVelocityMeasurementsDvlOnly::integrateMeasurementsNoise(const gtsam::Vector3 &linear_velocity, const gtsam::Rot3 &interpolated_rot,
                              double &dt, const double &fom)
  {
    if (dt <= 0)
    {
      // throw std::runtime_error(
      //     "PreintegratedVelocityMeasurements::integrateMeasurement: dt <=0");
      std::cout << "Warning: dt <=0" << std::endl;
      dt = 0.0001; // Jingyu: added this to avoid the error
    }
    gtsam::Matrix3 B = interpolated_rot.matrix() * dt;
    gtsam::Matrix3 dvl_cov = fom * gtsam::I_3x3;
    preintMeasCov_ += B * dvl_cov * B.transpose();
    
    
    vel_list.push_back(linear_velocity);
    rot_list.push_back(interpolated_rot);
    dt_list.push_back(dt);
  }

  gtsam::Point3 PreintegratedVelocityMeasurementsDvlOnly::predict(const gtsam::Point3 &bias, gtsam::Matrix3 &H_bias) const
  {
    gtsam::Point3 predicted_position;
    H_bias = gtsam::Matrix3::Zero();
    // integrate using vel_list and rot_list
    for (int i = 0; i < vel_list.size(); i++)
    {
      gtsam::Vector3 vel = vel_list[i] - bias;
      gtsam::Vector3 dPos = rot_list[i].matrix() * vel * dt_list[i];
      predicted_position += dPos;
      H_bias += rot_list[i].matrix() * dt_list[i]; // Jingyu: verified the jacobian for bias
    }
    return predicted_position;
  }

}