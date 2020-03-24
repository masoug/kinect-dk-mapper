#ifndef MappingParams_h
#define MappingParams_h


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


boost::optional<MappingParameters> setup_mapping(int argc, const char **argv);


#endif /* MappingParams_h */
