// azure-kinect-dk-logger.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <list>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>

#include <boost/filesystem.hpp>

#include <k4a/k4a.h>

//void
//tranformation_helpers_write_point_cloud(
//    const k4a_image_t point_cloud_image,
//    const char* file_name)
//{
//    int width = k4a_image_get_width_pixels(point_cloud_image);
//    int height = k4a_image_get_height_pixels(point_cloud_image);
//
//    int16_t* point_cloud_image_data = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
//
//    std::vector<Point> points;
//    for (int i = 0; i < width * height; i++)
//    {
//        Point point{
//            point_cloud_image_data[3 * i + 0],
//            point_cloud_image_data[3 * i + 1],
//            point_cloud_image_data[3 * i + 2]};
//        if (point.z == 0)
//        {
//            continue;
//        }
//
//        //point.rgb[0] = color_image_data[4 * i + 0];
//        //point.rgb[1] = color_image_data[4 * i + 1];
//        //point.rgb[2] = color_image_data[4 * i + 2];
//        //uint8_t alpha = color_image_data[4 * i + 3];
//
//        //if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0 && alpha == 0)
//        //{
//        //    continue;
//        //}
//
//        points.push_back(point);
//    }
//
//#define PLY_START_HEADER "ply"
//#define PLY_END_HEADER "end_header"
//#define PLY_ASCII "format ascii 1.0"
//#define PLY_ELEMENT_VERTEX "element vertex"
//
//    // save to the ply file
//    std::ofstream ofs(file_name); // text mode first
//    ofs << PLY_START_HEADER << std::endl;
//    ofs << PLY_ASCII << std::endl;
//    ofs << PLY_ELEMENT_VERTEX << " " << points.size() << std::endl;
//    ofs << "property float x" << std::endl;
//    ofs << "property float y" << std::endl;
//    ofs << "property float z" << std::endl;
//    //ofs << "property uchar red" << std::endl;
//    //ofs << "property uchar green" << std::endl;
//    //ofs << "property uchar blue" << std::endl;
//    ofs << PLY_END_HEADER << std::endl;
//    ofs.close();
//
//    std::stringstream ss;
//    for (size_t i = 0; i < points.size(); ++i)
//    {
//        // image data is BGR
//        ss << static_cast<float>(points[i].x) * 1e-3 << " " << static_cast<float>(points[i].y) * -1e-3 << " " << static_cast<float>(points[i].z) * -1e-3;
//        //ss << " " << (float)points[i].rgb[2] << " " << (float)points[i].rgb[1] << " " << (float)points[i].rgb[0];
//        ss << std::endl;
//    }
//    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
//    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
//}

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

static k4a_float3_t* depth_to_pointcloud(
    const k4a_image_t depth_image,
    const k4a_image_t xy_table)
{
    const int width = k4a_image_get_width_pixels(depth_image);
    const int height = k4a_image_get_height_pixels(depth_image);

    const uint16_t* const depth_data = (uint16_t*)(void*)k4a_image_get_buffer(depth_image);
    const k4a_float2_t* const xy_table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(xy_table);
    k4a_float3_t* point_cloud_data = (k4a_float3_t*)malloc(width * height * sizeof(k4a_float3_t));

    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }

    return point_cloud_data;
}

void point_cloud_writer_thread(
    const boost::filesystem::path& out_dir,
    const size_t width, const size_t height,
    std::list<k4a_float3_t*>& point_cloud_buffer,
    std::list<uint64_t>& timestamp_buffer,
    std::mutex& cloud_buffer_mutex,
    std::condition_variable& cloud_buffer_cv)
{
    while (true) {
        k4a_float3_t* point_cloud = NULL;
        uint64_t timestamp{};

        auto start_pt = std::chrono::high_resolution_clock::now();
        {
            std::unique_lock<std::mutex> ul(cloud_buffer_mutex);
            cloud_buffer_cv.wait(ul, [&] { return not point_cloud_buffer.empty(); });

            if (point_cloud_buffer.front() == NULL) {
                point_cloud_buffer.pop_front();
                std::cout << "Terminating writer thread..." << std::endl;
                return;
            }

            point_cloud = point_cloud_buffer.front();
            timestamp = timestamp_buffer.front();
            point_cloud_buffer.pop_front();
            timestamp_buffer.pop_front();
        }
        auto pop_queue_pt = std::chrono::high_resolution_clock::now();

        // first cache the output string...
        std::ostringstream out_string;
        size_t point_count = 0;
        char buf[1024];
        for (int i = 0; i < width * height; i++)
        {
            if (isnan(point_cloud[i].xyz.x) || isnan(point_cloud[i].xyz.y) || isnan(point_cloud[i].xyz.z))
            {
                continue;
            }
            
            memset(buf, 0, 1024);
            snprintf(buf, 1024, "%f %f %f\n",
                static_cast<float>(point_cloud[i].xyz.x) * 1e-3,
                static_cast<float>(point_cloud[i].xyz.y) * -1e-3,
                static_cast<float>(point_cloud[i].xyz.z) * -1e-3);

            out_string << buf;
            point_count++;
        }
        auto cache_out_string_pt = std::chrono::high_resolution_clock::now();

        std::ostringstream ss;
        ss << timestamp << ".ply";
        std::ofstream out_file((out_dir / ss.str()).c_str());
        out_file << "ply" << std::endl;
        out_file << "format ascii 1.0" << std::endl;
        out_file << "element vertex"
            << " " << point_count << std::endl;
        out_file << "property float x" << std::endl;
        out_file << "property float y" << std::endl;
        out_file << "property float z" << std::endl;
        out_file << "end_header" << std::endl;
        out_file.write(out_string.str().c_str(), (std::streamsize)out_string.str().length());
        out_file.close();
        auto write_out_pt = std::chrono::high_resolution_clock::now();

        free(point_cloud);

        //std::cout << "writer thread profile:" << std::endl;
        //std::chrono::duration<double> elapsed;
        //elapsed = pop_queue_pt - start_pt;
        //std::cout << "  pop_queue: " << elapsed.count() << std::endl;
        //elapsed = cache_out_string_pt - pop_queue_pt;
        //std::cout << "  cache_out_string_pt: " << elapsed.count() << std::endl;
        //elapsed = write_out_pt - cache_out_string_pt;
        //std::cout << "  write_out_pt: " << elapsed.count() << std::endl;
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

    // setup point cloud buffer
    std::list<k4a_float3_t*> point_cloud_buffer;
    std::list<uint64_t> timestamp_buffer;
    std::mutex cloud_buffer_mutex;
    std::condition_variable cloud_buffer_cv;

    std::thread cloud_writer_thread(
        point_cloud_writer_thread,
        std::ref(output_directory),
        calibration.depth_camera_calibration.resolution_width,
        calibration.depth_camera_calibration.resolution_height,
        std::ref(point_cloud_buffer),
        std::ref(timestamp_buffer),
        std::ref(cloud_buffer_mutex),
        std::ref(cloud_buffer_cv));

    // Capture a series of frames
    std::cout << std::endl << "Capturing frames..." << std::endl;
    for (size_t i = 0; i < num_frames; i++) {
        k4a_capture_t capture;
        if (k4a_device_get_capture(device, &capture, K4A_WAIT_INFINITE) != K4A_WAIT_RESULT_SUCCEEDED) {
            std::cerr << "Failed capture" << std::endl;
            
            {
                std::lock_guard<std::mutex> lg(cloud_buffer_mutex);
                point_cloud_buffer.push_back(NULL);
            }
            cloud_buffer_cv.notify_one();
            std::cout << "Waiting for writer thread to join..." << std::endl;
            cloud_writer_thread.join();

            k4a_device_stop_cameras(device);
            k4a_device_close(device);
            return -1;
        }

        auto depth_image = k4a_capture_get_depth_image(capture);
        k4a_float3_t* cloud = depth_to_pointcloud(depth_image, xy_table);

        {
            std::lock_guard<std::mutex> lg(cloud_buffer_mutex);
            point_cloud_buffer.push_back(cloud);
            timestamp_buffer.push_back(k4a_image_get_device_timestamp_usec(depth_image));
        }
        cloud_buffer_cv.notify_one();

        k4a_image_release(depth_image);
        k4a_capture_release(capture);
        std::cout << "  Captured frame " << i + 1 << std::endl;
    }
    std::cout << "Capture complete" << std::endl << std::endl;

    // Shut down the camera when finished with application logic
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    
    {
        std::lock_guard<std::mutex> lg(cloud_buffer_mutex);
        point_cloud_buffer.push_back(NULL);
    }
    cloud_buffer_cv.notify_one();
    std::cout << "Waiting for writer thread to join..." << std::endl;
    cloud_writer_thread.join();

    // free remaining resources
    k4a_image_release(xy_table);

    assert(point_cloud_buffer.empty());
    assert(timestamp_buffer.empty());

    std::cout << "Done" << std::endl;
    return 0;
}
