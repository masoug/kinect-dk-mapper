#include "KinectDKMapper.hpp"


KinectDKMapper::KinectDKMapper(const MappingParameters &params)
        : m_mapping_params(params), m_global_map(), m_sensor_trajectory(),
          m_sensor_to_map_tf(TP::Identity(4, 4)), m_icp()
{
    m_icp.setDefault();
}

TP KinectDKMapper::update(const DP &sensor_reading) {
    if (m_global_map.getNbPoints() == 0) {
        m_global_map = sensor_reading;
        m_sensor_trajectory.push_back(m_sensor_to_map_tf);
        return m_sensor_to_map_tf;
    }

    // align point cloud
    const auto transformation = m_icp(sensor_reading, m_global_map, m_sensor_to_map_tf);

    // apply tranformation to the sensor reading, then concatenate with the global map
    DP new_cloud(sensor_reading);
    m_icp.transformations.apply(new_cloud, transformation);
    m_global_map.concatenate(new_cloud);
    m_icp.referenceDataPointsFilters.apply(m_global_map);

    // save transform to trajectory
    m_sensor_trajectory.push_back(transformation);
    m_sensor_to_map_tf = transformation;

    return transformation;
}
