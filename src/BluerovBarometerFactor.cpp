#include "TURTLMap/BluerovBarometerFactor.h"

Vector BluerovBarometerFactor::evaluateError(const Pose3 &pose, boost::optional<gtsam::Matrix &> H) const
{
    Matrix tH;
    Vector ret = (Vector(1) << (pose.translation(tH).z() - measured_)).finished();
    if (H)
    {
        *H = tH.block<1, 6>(2, 0);
        // std::cout << tH.block<1, 6>(2, 0) << std::endl
        //           << std::endl;
        // std::cout << "H: " << *H << std::endl;
    }
    // std::cout << "Pose z error: " << (pose.z() - measured_) << std::endl;
    double error = (pose.z() - measured_);
    // if error is nan
    // if (std::isnan(error))
    // {
    //     std::cout << "Barometer is nan" << std::endl;
    //     std::cout << "Pose: " << pose << std::endl;
    //     assert(false);
    // }
    
    // return (Vector1() << pose.z() - measured_).finished();
    return (Vector1() << error).finished();
}