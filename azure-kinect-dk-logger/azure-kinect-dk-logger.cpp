// azure-kinect-dk-logger.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <list>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>

#include <k4a/k4a.h>

struct Point {
    int16_t x;
    int16_t y;
    int16_t z;
};


void
tranformation_helpers_write_point_cloud(
    const k4a_image_t point_cloud_image,
    const char* file_name)
{
    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(point_cloud_image);

    int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);

    std::vector<Point> points;
    for (int i = 0; i < width * height; i++)
    {
        Point point{
            point_cloud_image_data[3 * i + 0],
            point_cloud_image_data[3 * i + 1],
            point_cloud_image_data[3 * i + 2]};
        if (point.z == 0)
        {
            continue;
        }

        //point.rgb[0] = color_image_data[4 * i + 0];
        //point.rgb[1] = color_image_data[4 * i + 1];
        //point.rgb[2] = color_image_data[4 * i + 2];
        //uint8_t alpha = color_image_data[4 * i + 3];

        //if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
        //{
        //    continue;
        //}

        points.push_back(point);
    }

#define PLY_START_HEADER "ply"
#define PLY_END_HEADER "end_header"
#define PLY_ASCII "format ascii 1.0"
#define PLY_ELEMENT_VERTEX "element vertex"

    // save to the ply file
    std::ofstream ofs(file_name); // text mode first
    ofs << PLY_START_HEADER << std::endl;
    ofs << PLY_ASCII << std::endl;
    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    //ofs << "property uchar red" << std::endl;
    //ofs << "property uchar green" << std::endl;
    //ofs << "property uchar blue" << std::endl;
    ofs << PLY_END_HEADER << std::endl;
    ofs.close();

    std::stringstream ss;
    for (size_t i = 0; i < points.size(); ++i)
    {
        // image data is BGR
        ss << static_cast<float>(points[i].x) * 1e-3 << " " << static_cast<float>(points[i].y) * -1e-3 << " " << static_cast<float>(points[i].z) * -1e-3;
        //ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void process_captures(const k4a_transformation_t& transformation, const boost::filesystem::path& output_dir, const std::vector<k4a_capture_t>& capture_buffer)
{
    std::cout << std::endl << "Processing " << capture_buffer.size() << " captures..." << std::endl;
    for (size_t i = 0; i < capture_buffer.size(); i++)
    {
        const auto& capture = capture_buffer[i];
        k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
        if (depth_image != NULL) {
            //std::cout << "Depth16 resolution: " << k4a_image_get_width_pixels(depth_image) << "x" << k4a_image_get_height_pixels(depth_image) << std::endl;
            //std::cout << "Stride: " << k4a_image_get_stride_bytes(depth_image) << " bytes" << std::endl;

            // apply transformation to convert the depth image to a point cloud
            k4a_image_t point_cloud_image = NULL;
            if (K4A_FAILED(k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                k4a_image_get_width_pixels(depth_image),
                k4a_image_get_height_pixels(depth_image),
                k4a_image_get_width_pixels(depth_image) * 3 * (int)sizeof(int16_t),
                &point_cloud_image)))
            {
                std::cerr << "Failed to create point cloud image" << std::endl;
                k4a_image_release(depth_image);
                continue;
            }

            if (K4A_FAILED(k4a_transformation_depth_image_to_point_cloud(transformation, depth_image, K4A_CALIBRATION_TYPE_DEPTH, point_cloud_image))) {
                std::cerr << "Failed to apply depth-to-pointcloud transformation" << std::endl;
                k4a_image_release(point_cloud_image);
                k4a_image_release(depth_image);
                continue;
            }

            const auto timestamp = k4a_image_get_device_timestamp_usec(depth_image);
            std::stringstream ss;
            ss << timestamp << ".ply";
            tranformation_helpers_write_point_cloud(point_cloud_image, (output_dir / ss.str()).string().c_str());

            k4a_image_release(point_cloud_image);

            // Release the image
            k4a_image_release(depth_image);
        }

        std::cout << "  Saved capture #" << i + 1 << std::endl;
    }
}

static void create_xy_table(const k4a_calibration_t* const calibration, k4a_image_t xy_table)
{
    k4a_float2_t* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

static void generate_point_cloud(
    const k4a_image_t depth_image,
    const k4a_image_t xy_table,
    k4a_image_t point_cloud,
    int* point_count)
{
    const int width = k4a_image_get_width_pixels(depth_image);
    const int height = k4a_image_get_height_pixels(depth_image);

    const uint16_t* const depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
    const k4a_float2_t* const xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    k4a_float3_t* point_cloud_data = (k4a_float3_t*)(void*)k4a_image_get_buffer(point_cloud);

    *point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            (*point_count)++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}

int main(const int argc, const char* argv[])
{
    std::cout << "Usage: " << argv[0] << " [num frames] [output directory]" << std::endl;
    if (argc < 3) {
        std::cerr << "Insufficient arguments" << std::endl;
        return -1;
    }

    const unsigned int num_frames = std::atoi(argv[1]);
    if (num_frames == 0) {
        std::cerr << "Requested 0 frames to capture" << std::endl;
        return -1;
    }
    std::cout << "Capturing " << num_frames << " frames" << std::endl;

    const auto output_directory = boost::filesystem::path(argv[2]);
    if (not boost::filesystem::is_directory(output_directory)) {
        std::cerr << "Output path " << output_directory << " is not a directory" << std::endl;
        return -1;
    }

    std::cout << "Looking for kinect devices..." << std::endl;
    const auto count = k4a_device_get_installed_count();
    std::cout << "Found " << count << " attached cameras." << std::endl;

    std::cout << "Opening default device..." << std::endl;
    k4a_device_t device = NULL;
    if (K4A_FAILED(k4a_device_open(K4A_DEVICE_DEFAULT, &device))) {
        std::cerr << "Failed to open default device." << std::endl;
        return -1;
    }

    // Get the size of the serial number
    size_t serial_size = 0;
    k4a_device_get_serialnum(device, NULL, &serial_size);

    // Allocate memory for the serial, then acquire it
    char* serial = (char*)(malloc(serial_size));
    k4a_device_get_serialnum(device, serial, &serial_size);
    std::cout << "  Serial Number: " << serial << std::endl;
    free(serial);

    // Configure a stream of 4096x3072 BRGA color data at 15 frames per second
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.synchronized_images_only = true;

    // Start the camera with the given configuration
    if (K4A_FAILED(k4a_device_start_cameras(device, &config))) {
        std::cerr << "Failed to start cameras!" << std::endl;
        k4a_device_close(device);
        return 1;
    }

    k4a_calibration_t calibration;
    if (K4A_FAILED(k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))) {
        std::cout << "Failed to get calibration" << std::endl;
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
        return -1;
    }

    std::cout << "Pre-computing calibration lookup table..." << std::endl;
    k4a_image_t xy_table = NULL;
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        calibration.depth_camera_calibration.resolution_width,
        calibration.depth_camera_calibration.resolution_height,
        calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
        &xy_table);

    create_xy_table(&calibration, xy_table);

    // Capture a series of frames
    std::cout << std::endl << "Capturing frames..." << std::endl;
    for (size_t i = 0; i < num_frames; i++) {
        k4a_capture_t capture;
        if (k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED) {
            std::cerr << "Failed capture" << std::endl;

            k4a_device_stop_cameras(device);
            k4a_device_close(device);
            return -1;
        }

        auto depth_image = k4a_capture_get_depth_image(capture);
        k4a_image_t point_cloud = NULL;

        k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
            calibration.depth_camera_calibration.resolution_width,
            calibration.depth_camera_calibration.resolution_height,
            calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t),
            &point_cloud);

        int point_count;
        generate_point_cloud(depth_image, xy_table, point_cloud, &point_count);

        k4a_image_release(point_cloud);
        k4a_image_release(depth_image);
        k4a_capture_release(capture);
        std::cout << "  Captured frame " << i + 1 << std::endl;
    }
    std::cout << "Capture complete" << std::endl << std::endl;

    // Shut down the camera when finished with application logic
    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    std::cout << "Done" << std::endl;
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
