#ifndef KINECT_DK_MAPPER_KINECTDKMAPPER_HPP
#define KINECT_DK_MAPPER_KINECTDKMAPPER_HPP


#include <vector>

#include "MappingParams.hpp"


class KinectDKMapper {
public:
    explicit KinectDKMapper(const MappingParameters& params);

    TP update(const DP &sensor_reading);

    const DP& get_global_map() const { return m_global_map; }

private:
    const MappingParameters& m_mapping_params;
    DP m_global_map;
    std::vector<TP> m_sensor_trajectory;
    TP m_sensor_to_map_tf;
    PM::ICP m_icp;
};


#endif //KINECT_DK_MAPPER_KINECTDKMAPPER_HPP
