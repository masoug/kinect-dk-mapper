#include <iostream>
#include <map>
#include <vector>

#include "KinectDKMapper.hpp"


int main(const int argc, const char *argv[]) {

    const auto params_optional = setup_mapping(argc, argv);
    if (not params_optional) {
        std::cerr << "Failed to setup mapping parameters" << std::endl;
        return -1;
    }

    const auto params = *params_optional;

    KinectDKMapper mapper(params);

    std::cout << "Building map from multiple sensor readings..." << std::endl;
    for (size_t frame_idx = 0; frame_idx < params.input_datapoints.size(); frame_idx++)
    {
        std::cout << "  Processing frame index: " << frame_idx << std::endl;
        const auto& sensor_reading = params.input_datapoints[frame_idx];
        const auto transformation = mapper.update(sensor_reading);
        std::cout << "    Updated transformation (sensor-to-map): \n" << transformation.inverse() << std::endl << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Saving global map point cloud (global_map.ply)..." << std::endl;
    mapper.get_global_map().save("global_map.ply");

    std::cout << "Done mapping" << std::endl;
    return 0;
}
