#include "MappingParams.hpp"


boost::optional<MappingParameters> setup_mapping(int argc, const char **argv) {
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

