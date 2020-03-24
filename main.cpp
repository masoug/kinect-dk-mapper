#include <iostream>
#include <map>
#include <vector>

#include "MappingParams.hpp"


int main(const int argc, const char *argv[]) {

    const auto params_optional = setup(argc, argv);
    if (not params_optional) {
        std::cerr << "Failed to setup mapping parameters" << std::endl;
        return -1;
    }

    const auto params = *params_optional;

    // Create the default ICP algorithm
    PM::ICP icp;

    // See the implementation of setDefault() to create a custom ICP algorithm
    icp.setDefault();

    // keep track of the transformation from map to the current/latest sensor frame
    TP tf_sensor_to_map = TP::Identity(4, 4);

    // global point cloud map that integrates all the sensor readings
    DP global_map;

    std::cout << "Building map from multiple sensor readings..." << std::endl;
    for (size_t frame_idx = 0; frame_idx < params.input_datapoints.size(); frame_idx++)
    {
        std::cout << "  Processing frame index: " << frame_idx << std::endl;
        const auto& sensor_reading = params.input_datapoints[frame_idx];
        if (frame_idx == 0) {
            // handle first frame
            assert(global_map.getNbPoints() == 0);
            global_map = sensor_reading;
            continue;
        }

        // align point cloud
        const auto transformation = icp(sensor_reading, global_map, tf_sensor_to_map);
        std::cout << "    Updated transformation: \n" << transformation.inverse() << std::endl << std::endl;

        // apply tranformation to the sensor reading, then concatenate with the global map
        DP new_cloud(sensor_reading);
        icp.transformations.apply(new_cloud, transformation);
        global_map.concatenate(new_cloud);
    }
    std::cout << std::endl;


    return 0;
}
