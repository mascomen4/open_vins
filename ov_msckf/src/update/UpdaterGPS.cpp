//
// Created by pi on 12.10.2022.
//

#include "UpdaterGPS.h"

// include the definitions of the classes here
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/sensor_data.h"

using namespace ov_core;
using namespace ov_msckf;

// TODO: Debug the function.
void UpdaterGPS::update(std::shared_ptr<State> state, const GPSData& gpsMessage) {
    auto H = Eigen::Matrix<double, 3, 15>::Zero();
    auto I = Eigen::Matrix<double, 3, 3>::Identity();
    H.block(0, 5, 3, 3) = I;

    std::vector<std::shared_ptr<ov_type::Type>> H_order;
    H_order.push_back(std::shared_ptr<ov_type::IMU>());

    auto res = gpsMessage.pos - state->_imu->pos();

    auto R = gpsMessage.posCov;
    R.resize(3, 3);

    StateHelper::EKFUpdate(state, H_order, H, res, R);
}
