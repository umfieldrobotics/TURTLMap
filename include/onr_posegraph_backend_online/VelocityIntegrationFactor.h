#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/base/serialization.h>
#include <Eigen/Dense>

#include <ostream>
namespace pose_graph_backend
{
  struct PreintegratedVelocityParams
  {
    gtsam::Matrix3 biasVelCovariance_; ///< continuous-time "Covariance" describing velocity "measurement" bias random walk
    gtsam::Matrix3 biasInt_;           ///< covariance of bias used as an initial estimate

    PreintegratedVelocityParams() : biasVelCovariance_(gtsam::Matrix3::Zero()), biasInt_(gtsam::Matrix3::Zero()) {}

    PreintegratedVelocityParams(const gtsam::Matrix3 &biasVelCovariance, const gtsam::Matrix3 &biasInt)
        : biasVelCovariance_(biasVelCovariance), biasInt_(biasInt) {}

    PreintegratedVelocityParams(const boost::shared_ptr<PreintegratedVelocityParams> &p)
    {
      biasVelCovariance_ = p->biasVelCovariance_;
      biasInt_ = p->biasInt_;
    }
    void print(const std::string &s = "") const;
    bool equals(const PreintegratedVelocityParams &expected, double tol = 1e-9) const;

    void setBiasVelCovariance(const gtsam::Matrix3 &cov) { biasVelCovariance_ = cov; }
    void setBiasInt(const gtsam::Matrix3 &cov) { biasInt_ = cov; }

  private:
    //** Serialization function */
    friend class boost::serialization::access;
    template <class ARCHIVE>
    void serialize(ARCHIVE &ar, const unsigned int version)
    {
      ar &BOOST_SERIALIZATION_NVP(biasVelCovariance_);
      ar &BOOST_SERIALIZATION_NVP(biasInt_);
    }

  public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW
  };

  
  class PreintegratedVelocityMeasurementsDvlOnly
  {
  public:
    Eigen::Matrix<double, 3, 3> preintMeasCov_; //TODO: change dimension here
    gtsam::Point3 get_accumulated_positions() const { return accumulated_positions_; } 
    
  protected:
    friend class DvlOnlyFactor; // TODO: change 

  private:
    double deltaTij;
    gtsam::Pose3 accumulated_poses_;
    gtsam::Point3 accumulated_positions_;
    // gtsam::Matrix3 accumulated_rotations_;
    gtsam::imuBias::ConstantBias bias_; // this is for the DVL bias
    gtsam::Matrix3 delpdelBiasOmega_;
    gtsam::Matrix3 delpdelBiasDvl_ = gtsam::Matrix3::Zero();

  public:
    PreintegratedVelocityMeasurementsDvlOnly() { preintMeasCov_ = gtsam::Matrix3::Zero(); }// identity matrix

    ~PreintegratedVelocityMeasurementsDvlOnly() {}

    void resetIntegration()
    {
      accumulated_poses_ = gtsam::Pose3();
      accumulated_positions_ = gtsam::Point3(0, 0, 0);
      // accumulated_rotations_ = gtsam::Matrix3();
      // preintMeasCov_.setZero();
      delpdelBiasDvl_ = gtsam::Matrix3::Zero();
      // preintMeasCov_ = gtsam::Matrix3::Identity() * 0.01;
      preintMeasCov_ = gtsam::Matrix3::Zero();
      vel_list.clear();
      rot_list.clear();
      dt_list.clear();
    }

    // TODO: @Onur - check implementation add the reset bias function here
    void resetIntegrationAndBias(const gtsam::imuBias::ConstantBias &bias)
    {
      bias_ = bias;
      resetIntegration();
    }

    gtsam::Matrix preintMeasCov() const { return preintMeasCov_; }

    void integrateMeasurements(const gtsam::Vector3 &linear_velocity, const gtsam::Rot3 &interpolated_rot,
                               double &dt);
    
    void integrateMeasurementsNoise(const gtsam::Vector3 &linear_velocity, const gtsam::Rot3 &interpolated_rot,
                               double &dt, const double &fom);

    gtsam::Pose3 getAccumulatedPoses() const { return accumulated_poses_; }
    gtsam::Matrix getDelPijDelBiasOmega() const { return delpdelBiasOmega_; }
    gtsam::Matrix getDelPijDelBiasDvl() const { return delpdelBiasDvl_; }
    gtsam::Point3 predict(const gtsam::Point3 &bias, gtsam::Matrix3 &H_bias) const;
    std::vector<gtsam::Vector3> vel_list;
    std::vector<gtsam::Rot3> rot_list;
    std::vector<double> dt_list;
  };

  class DvlOnlyFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3, gtsam::imuBias::ConstantBias>
  {

  private:
    const PreintegratedVelocityMeasurementsDvlOnly _PVM_;

  public:
    DvlOnlyFactor() {}
    DvlOnlyFactor(gtsam::Key pose_i, gtsam::Key pose_j,
                          gtsam::Key vbias_i, PreintegratedVelocityMeasurementsDvlOnly &pvm) : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Pose3,
                                                                                                                              gtsam::imuBias::ConstantBias>(gtsam::noiseModel::Gaussian::Covariance(pvm.preintMeasCov_),
                                                                                                                                                                                                        pose_i, pose_j, vbias_i),
                                                                                                                         _PVM_(pvm)
    {
    }

    virtual ~DvlOnlyFactor() {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose_i, const gtsam::Pose3 &pose_j,
                                const gtsam::imuBias::ConstantBias &vbias_i,
                                boost::optional<gtsam::Matrix &> H1, boost::optional<gtsam::Matrix &> H2,
                                boost::optional<gtsam::Matrix &> H3) const
    {
      {
        gtsam::Rot3 R_i = pose_i.rotation();
        gtsam::Rot3 R_j = pose_j.rotation();
        gtsam::Point3 p_i = pose_i.translation();
        gtsam::Point3 p_j = pose_j.translation();
        gtsam::Matrix3 H_vbias;
        gtsam::Point3 vbias = vbias_i.accelerometer();
        gtsam::Point3 accumulated_translation = _PVM_.predict(vbias, H_vbias);

        // TODO: Jingyu: propagate the noise pvm.preintMeasCov_

        // gtsam::Pose3 accumulated_poses = _PVM_.getAccumulatedPoses();
        // std::cout << "accumulated_poses: " << accumulated_translation << std::endl;
        // std::cout << "current bias " << vbias_i << std::endl;


        // Compute the inverse of R_i and save as a Rot3
        gtsam::Vector3 residual = gtsam::Rot3(R_i.matrix().transpose()).rotate(p_j - p_i) - accumulated_translation;
        // std::cout << "residual: " << residual << std::endl;
        // if any of residual is nan
        // if (residual.array().isNaN().any())
        // {
        //   std::cout << "residual is nan" << std::endl;
        //   std::cout << "accumulated_translation: " << accumulated_translation << std::endl;
        //   std::cout << "vbias: " << vbias_i << std::endl;
        //   std::cout << "pose_i: " << pose_i << std::endl;
        //   std::cout << "pose_j: " << pose_j << std::endl;
        //   assert (false);
        // }
        // std::cout << "----------------------" << std::endl;

        if (H1)
        {
          H1->resize(3, 6);
          // -I3x3
          H1->block<3, 3>(0, 0) = -gtsam::Matrix3::Identity();
          H1->block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
        }
        if (H2)
        {
          H2->resize(3, 6);
          H2->block<3, 3>(0, 0) = R_i.transpose().matrix() * R_j.matrix();
          H2->block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
        }
        if (H3)
        {
          H3->resize(3, 6);
          H3->block<3, 3>(0, 0) = H_vbias;
          // H3->block<3, 3>(0, 0) = gtsam::Matrix3::Zero();
          H3->block<3, 3>(0, 3) = gtsam::Matrix3::Zero();
          // std::cout << "H3: " << *H3 << std::endl;
        }
        // gtsam::Vector err(3);
        // err << residual;
        return residual;
      }
    }
  };
}