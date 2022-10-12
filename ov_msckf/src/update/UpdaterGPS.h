//
// Created by pi on 12.10.2022.
//

#ifndef OV_MSCKF_UPDATEGPS_H
#define OV_MSCKF_UPDATEGPS_H

#include <Eigen/Eigen>
#include <memory>
#include "UpdaterOptions.h"
#include "types/Vec.h"

namespace ov_core{
    struct GPSData;
}

namespace ov_msckf{

class State; // forward declaration

class UpdaterGPS {
public:
    void update(std::shared_ptr<State> state, const ov_core::GPSData& gpsMessage);

};

}



#endif //OV_MSCKF_UPDATEGPS_H
