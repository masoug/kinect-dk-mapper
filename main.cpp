#include <iostream>
#include <map>
#include <vector>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <pointmatcher/PointMatcher.h>


using PM = PointMatcher<float>;
using DP = PM::DataPoints;
using TP = PM::TransformationParameters;


struct MappingParameters {
    std::vector<DP> input_datapoints;
};

boost::optional<MappingParameters> setup(const int argc, const char *argv[]) {
    using namespace boost::program_options;
    using namespace boost::filesystem;

    options_description desc("Allowed Options");
    desc.add_options()
            ("help", "Show help message")
            ("input_dir",
             value<std::string>()->required(),
             "Input directory listing of pointcloud ply files.");

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << "Point cloud reconstruction from multiple RGB-D camera frames." << std::endl;
        std::cout << desc << std::endl;
        return {};
    }

    notify(vm);

    const auto input_dir = path(vm["input_dir"].as<std::string>());
    std::cout << "Loading input point clouds from " << input_dir << std::endl;
    if (not is_directory(input_dir)) {
        std::cerr << "Input directory argument " << input_dir << " is not a directory" << std::endl;
        return {};
    }

    // parse list input ply files, then sort by timestamp
    std::map<size_t, std::string> input_pointcloud_filenames;
    for (directory_iterator itr(input_dir); itr != directory_iterator{}; itr++) {
        const auto& p = itr->path();

        if (not is_regular_file(p))
            continue;

        if (p.extension() != ".ply")
            continue;

        input_pointcloud_filenames[std::stol(p.stem().string())] = p.string();
    }

    MappingParameters result;
    for (const auto& fname_pair : input_pointcloud_filenames) {
        std::cout << "  Loading (ts " << fname_pair.first << ") " << fname_pair.second << std::endl;
        result.input_datapoints.emplace_back(DP::load(fname_pair.second));
    }
    std::cout << "Loading done." << std::endl;

    return result;
}

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