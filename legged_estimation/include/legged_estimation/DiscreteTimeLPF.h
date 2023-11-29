//
// Created by sudaxia on 23-10-29.
//

#pragma once

#include "legged_estimation/StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;

class DiscreteTimeLPF : public StateEstimateBase {
public:
    DiscreteTimeLPF(PinocchioInterface pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

    vector_t update(const ros::Time& time, const ros::Duration& period) override;

    void loadSetting(const std::string& taskFile, bool verbose);

private:
    scalar_t gamma_;
    scalar_t beta_;
    scalar_t g_;
    Eigen::Matrix<scalar_t, 12, 18> S_;
    Eigen::Matrix<scalar_t, 18, 1> tau1_;
    Eigen::Matrix<scalar_t, 18, 1> tau2_;
    Eigen::Matrix<scalar_t, 18, 1> lastTau2_;
    Eigen::Matrix<scalar_t, 18, 1> hatTau_;



};

}  // namespace legged

