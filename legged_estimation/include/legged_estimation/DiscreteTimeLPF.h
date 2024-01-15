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
    void updateTorque(const vector_t& jointEffort);

    void loadSetting(const std::string& taskFile, bool verbose);

private:
    // cutoff frequency
    scalar_t lambda_;   // analog
    scalar_t gamma_;    // digital

    scalar_t beta_;
    Eigen::Matrix<scalar_t, 12, 18> j_;
    Eigen::Matrix<scalar_t, 12, 1> aTau_;
    Eigen::Matrix<scalar_t, 12, 18> Sl_;
    Eigen::Matrix<scalar_t, 12, 1> estimateF_;
    Eigen::Matrix<scalar_t, 18, 1> g_;
    Eigen::Matrix<scalar_t, 12, 18> S_;
    Eigen::Matrix<scalar_t, 18, 1> tau1_;
    Eigen::Matrix<scalar_t, 18, 1> tau2_;
    Eigen::Matrix<scalar_t, 18, 1> lastTau2_;
    Eigen::Matrix<scalar_t, 18, 1> hatTau_;
    Eigen::Matrix<scalar_t, 18, 18> lastM_;



};

}  // namespace legged

