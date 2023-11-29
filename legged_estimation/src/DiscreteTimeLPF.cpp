//
// Created by sudaxia on 23-10-30.
//

#include <pinocchio/fwd.hpp>
#include "legged_estimation/DiscreteTimeLPF.h"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>


namespace legged {

DiscreteTimeLPF::DiscreteTimeLPF(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics)
        : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
    gamma_ = 0.3;
    beta_ = (1 - gamma_)/(gamma_ * 0.002);
    g_ = 9.8;
    tau1_.setZero();
    tau2_.setZero();
    lastTau2_.setZero();
    hatTau_.setZero();
}

vector_t DiscreteTimeLPF::update(const ros::Time &time, const ros::Duration &period) {
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();
    size_t actuatedDofNum = info_.actuatedDofNum;

    vector_t qPino(info_.generalizedCoordinatesNum);
    vector_t vPino(info_.generalizedCoordinatesNum);

    qPino.setZero();
    qPino.segment<3>(3) = rbdState_.head<3>();  // Only set orientation, let position in origin.
    qPino.tail(actuatedDofNum) = rbdState_.segment(6, actuatedDofNum);

    vPino.setZero();
    vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            rbdState_.segment<3>(info_.generalizedCoordinatesNum));  // Only set angular velocity, let linear velocity be zero
    vPino.tail(actuatedDofNum) = rbdState_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

    pinocchio::crba(model,data,qPino);
    pinocchio::computeCoriolisMatrix(model, data, qPino, vPino);

    tau1_ = beta_ * data.M * vPino;
    tau2_ = tau1_ + (data.C.transpose() * vPino).eval();
    hatTau_ = gamma_ * tau2_ + (1 - gamma_) * lastTau2_;
    lastTau2_ = tau2_;
    return hatTau_;
}

} // namespace legged