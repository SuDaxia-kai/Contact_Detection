//
// Created by sudaxia on 24-1-18.
//

#include <legged_estimation/ContactKalman.h>

namespace legged {

ContactKalman::ContactKalman() {
  xHat_.setZero();
  A_.setZero();
  B_.setIdentity();
  Q_.setIdentity();
  Q_ = 0.25 * Q_;
  R_.setIdentity();
  R_ << 5 * Eigen::Matrix<scalar_t, 4, 4>::Identity(), 6.271 * Eigen::Matrix<scalar_t, 4, 4>::Identity();
  Z_.setZero();
  P_.setIdentity();
  P_ = 100. * P_;
  H_.setZero();
  H_.block(0, 0, 4, 4) = Eigen::Matrix<scalar_t, 4, 4>::Identity();
  H_.block(4, 0, 4, 4) = Eigen::Matrix<scalar_t, 4, 4>::Identity();
}

vector_t ContactKalman::update(DiscreteTimeLPF* footEstimate, ContactProbabilityFromGait* gaitEstimate,
                               const ros::Time &time, const ros::Duration &period) {
    Z_ << footEstimate->getProFromHeight(), footEstimate->getProFromForce();
    vector_t contactProFromGait = gaitEstimate->getProFromGait();
    xHat_ = A_ * xHat_ + B_ * contactProFromGait;
    Eigen::Matrix<scalar_t, 4, 4> Pk = A_ * P_ * A_.transpose() + Q_;
    Eigen::Matrix<scalar_t, 8, 1> zModel = H_ * xHat_;
    Eigen::Matrix<scalar_t, 8, 1> ez = Z_ - zModel;
    Eigen::Matrix<scalar_t, 8, 8> Kk_2 = (H_ * Pk * H_.transpose() + R_);
    Eigen::Matrix<scalar_t, 8, 1> inverseKk_2 = Kk_2.lu().solve(ez);
    xHat_ += Pk * H_.transpose() * inverseKk_2;
    Eigen::Matrix<scalar_t, 8, 4> sC = Kk_2.lu().solve(H_);
    P_ = (Eigen::Matrix<scalar_t, 4, 4>::Identity() - Pk * H_.transpose() * sC) * Pk;
    P_ = (P_ + P_.transpose())/2; // guarantee the P_ is a symmetric matrix
    return xHat_;
}

} // namespace legged
