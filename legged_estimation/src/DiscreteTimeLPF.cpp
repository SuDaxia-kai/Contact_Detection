//
// Created by sudaxia on 23-10-30.
//

#include <pinocchio/fwd.hpp>
#include "legged_estimation/DiscreteTimeLPF.h"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {

DiscreteTimeLPF::DiscreteTimeLPF(ocs2::PinocchioInterface pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics)
        : StateEstimateBase(std::move(pinocchioInterface), std::move(info), eeKinematics) {
    lambda_ = 15;
    gamma_ = exp(-lambda_*0.002);
    beta_ = (1 - gamma_)/(gamma_ * 0.002);
    g_.setConstant(9.8);
    S_.setZero();
    S_.block(0, 6, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    tau1_.setZero();
    tau2_.setZero();
    lastTau2_.setZero();
    hatTau_.setZero();
    j_.setZero();
    aTau_.setZero();
    Sl_.setOnes();
    Sl_.block(0, 6, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    estimateF_.setZero();
    pinocchio::crba(pinocchioInterface_.getModel(), pinocchioInterface_.getData(), Eigen::Matrix<scalar_t, 18, 1>::Zero());
    lastM_ = pinocchioInterface_.getData().M;
}

void DiscreteTimeLPF::updateTorque(const vector_t& jointEffort) {
    aTau_ = jointEffort;
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

    pinocchio::crba(model, data, qPino);
    pinocchio::nonLinearEffects(model, data, qPino, vPino);

    // compute the Jacobin of the feet
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
    }

    tau1_ = beta_ * data.M * vPino;
    tau2_ = tau1_ + S_.transpose() * aTau_ - data.nle + (data.M - lastM_) * 500 * vPino;
    hatTau_ = tau1_ - (gamma_ * tau2_ + (1 - gamma_) * lastTau2_);
    estimateF_ = (Sl_ * j_.transpose()).inverse() * Sl_ * hatTau_;
    lastM_ = data.M;
    lastTau2_ = tau2_;
    return estimateF_;
}

} // namespace legged