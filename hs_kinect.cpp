#include <iostream>
#include <k4a/k4a.hpp>
#include <fstream>


#include <opencv2/opencv.hpp>
#include <ctime>
#include <chrono>



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <w_pc.h>

#include <boost/filesystem.hpp>



std::string path = boost::filesystem::current_path().string() + "\\output\\";

 
std::string yymmddhhmmss()
{
    auto currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    // Convert the current time to a tm structure
    struct tm timeInfo;
    localtime_s(&timeInfo, &currentTime);

    // Extract year, month, day, hour, minute, and second
    int year = timeInfo.tm_year % 100;  // Assuming we are interested in a two-digit year
    int month = timeInfo.tm_mon + 1;     // Months are 0-based
    int day = timeInfo.tm_mday;
    int hour = timeInfo.tm_hour;
    int minute = timeInfo.tm_min;
    int second = timeInfo.tm_sec;

    // Print in the specified format "YYMMDDHHMMSS"
    std::ostringstream oss;
    oss << std::setfill('0') << std::setw(2) << year
        << std::setw(2) << month
        << std::setw(2) << day
        << "_"
        << std::setw(2) << hour
        << std::setw(2) << minute
        << std::setw(2) << second;

    std::cout << oss.str() << std::endl;

    return oss.str();
}


void saveDepthImage (k4a::image colorImage, k4a::image depthImage, cv::Mat colorMat, cv::Mat depthMat, k4a::calibration calibration)
{
    k4a::image transformedDepthImage = k4a::image::create(
        K4A_IMAGE_FORMAT_DEPTH16,
        calibration.color_camera_calibration.resolution_width,
        calibration.color_camera_calibration.resolution_height,
        calibration.color_camera_calibration.resolution_width * (int)sizeof(uint16_t));

    k4a::transformation transformation(calibration);


         if (colorImage.is_valid() && depthImage.is_valid() && depthImage.get_size() > 0)
         {
             // Retrieve image properties
             int colorWidth = colorImage.get_width_pixels();
             int colorHeight = colorImage.get_height_pixels();
             int depthWidth = depthImage.get_width_pixels();
             int depthHeight = depthImage.get_height_pixels();
             int colorStride = colorImage.get_stride_bytes();
             int depthStride = depthImage.get_stride_bytes();
             int colorFormat = colorImage.get_format();
             int depthFormat = depthImage.get_format();
         
             // Check image dimensions
             if (colorWidth <= 0 || colorHeight <= 0 || depthWidth <= 0 || depthHeight <= 0) {
                 std::cerr << "\n\nInvalid image dimensions" << std::endl;
                 //continue; // Skip processing this frame
             }
         
             // Check image properties
             else if (colorFormat != K4A_IMAGE_FORMAT_COLOR_BGRA32 || depthFormat != K4A_IMAGE_FORMAT_DEPTH16 ||
                 colorStride != colorWidth * 4 || depthStride != depthWidth * 2) {
                 std::cerr << "\n\nImage properties mismatch" << std::endl;
                 //continue; // Skip processing this frame
                 }
                 else 
                     {                     
                      try
                      {
                          transformation.depth_image_to_color_camera(depthImage, &transformedDepthImage);
                      }
                      catch (const k4a::error& e) {
                          std::cerr << "Error during depth-to-color transformation: " << e.what() << std::endl;
                          colorMat = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorImage.get_buffer());
                          depthMat = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16UC1, (void*)depthImage.get_buffer(), cv::Mat::AUTO_STEP);
                          cv::Mat depthScaled;
                          depthMat.convertTo(depthScaled, CV_8U, 255.0 / 3500); // Adjust 5000.0 based on the maximum depth range of your device
                      }
                     
                      colorMat = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorImage.get_buffer());
                      depthMat = cv::Mat(transformedDepthImage.get_height_pixels(), transformedDepthImage.get_width_pixels(), CV_16UC1, (void*)transformedDepthImage.get_buffer(), cv::Mat::AUTO_STEP);
                     
                      // Scale the depth values for better visualization
                      cv::Mat depthScaled;
                      depthMat.convertTo(depthScaled, CV_8U, 255.0 / 3500); // Adjust 5000.0 based on the maximum depth range of your device
                     
                      cv::imwrite(path + "depth_image_" + yymmddhhmmss() + " .png", depthMat);
                     }
         
         }
         else { std::cerr << " something not valid to save depth transformed" << std::endl; }
}


k4a_float3_t get_origin_point(k4a::calibration calibration)
{
    std::cout <<  "get_origin_point " << std::endl;

    // Retrieve the camera space origin
    k4a_float3_t cameraSpaceOrigin = calibration.convert_3d_to_3d(k4a_float3_t{ 0.0f, 0.0f, 0.0f },
        k4a_calibration_type_t::K4A_CALIBRATION_TYPE_DEPTH,
        k4a_calibration_type_t::K4A_CALIBRATION_TYPE_DEPTH);

    // Print the camera space origin
    std::cout << "Camera space origin: (" << cameraSpaceOrigin.xyz.x << ", "
        << cameraSpaceOrigin.xyz.y << ", "
        << cameraSpaceOrigin.xyz.z << ")" << std::endl;

    return cameraSpaceOrigin;
}



bool toggle = true;


int main() {
    // Allocate resources
    const int32_t TIMEOUT_IN_MS = 10000;
    uint32_t device_count = 0;

    std::vector<k4a_device_t> devices;
    std::vector<k4a_transformation_t> transformations;
    std::vector<k4a::calibration> calibrations;

    try {
        std::cout << "Starting capture functions" << std::endl;

        // Get the number of connected devices
        device_count = k4a_device_get_installed_count();

        if (device_count == 0) {
            printf("No K4A devices found\n");
            return 0;
        }

        // Open and configure each device
        for (uint32_t i = 0; i < device_count; ++i) {
            k4a_device_t device;
            if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &device)) {
                printf("Failed to open device %d\n", i);
                continue;
            }
            devices.push_back(device);

            k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
            config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
            config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
            config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
            config.camera_fps = K4A_FRAMES_PER_SECOND_15;
            config.synchronized_images_only = true ;

            k4a::calibration calibration;
            if (K4A_RESULT_SUCCEEDED != k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration)) {
                printf("Failed to get calibration for device %d\n", i);
                k4a_device_close(device);
                continue;
            }
            calibrations.push_back(calibration);

            k4a_transformation_t transformation = k4a_transformation_create(&calibration);
            transformations.push_back(transformation);

            if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config)) {
                printf("Failed to start cameras for device %d\n", i);
                k4a_device_close(device);
                continue;
            }

            printf("Device %d cameras started\n", i);
        }

        if (devices.empty()) {
            printf("No devices successfully started\n");
            return 1;
        }

        // Vectors to store images for each device
        std::vector<cv::Mat> colorMats(devices.size());
        std::vector<cv::Mat> depthMats(devices.size());

        // Create OpenCV windows for each device
        for (size_t i = 0; i < devices.size(); ++i) {
            cv::namedWindow("Device " + std::to_string(i) + " - Color Image", cv::WINDOW_NORMAL);
        }

        bool toggle = true;


        std::vector<k4a::image> lastDepthImages(devices.size());
        std::vector<k4a::image> lastColorImages(devices.size());
        std::vector<k4a_capture_t> lastCaptures(devices.size(), nullptr);

        // Main capture loop
        while (true) {
            for (size_t i = 0; i < devices.size(); ++i) {

                k4a_capture_t capture;
                switch (k4a_device_get_capture(devices[i], &capture, TIMEOUT_IN_MS)) {
                case K4A_WAIT_RESULT_SUCCEEDED:
                    // Update the last valid capture
                    if (lastCaptures[i] != nullptr) {
                        k4a_capture_release(lastCaptures[i]); // Release the previous capture
                    }
                    lastCaptures[i] = capture; // Store the new valid capture
                    break;
                case K4A_WAIT_RESULT_TIMEOUT:
                    std::cerr << "Device " << i << " timed out waiting for a capture\n";
                    continue;
                case K4A_WAIT_RESULT_FAILED:
                    std::cerr << "Device " << i << " failed to get a capture\n";
                    continue;
                }

                // Process depth and color images
                k4a::image depthImage = k4a_capture_get_depth_image(capture);
                k4a::image colorImage = k4a_capture_get_color_image(capture);

                if (depthImage.is_valid() && colorImage.is_valid()) {

                    lastDepthImages[i] = depthImage;
                    lastColorImages[i] = colorImage;

                    colorMats[i] = cv::Mat(colorImage.get_height_pixels(), colorImage.get_width_pixels(), CV_8UC4, colorImage.get_buffer());
                    depthMats[i] = cv::Mat(depthImage.get_height_pixels(), depthImage.get_width_pixels(), CV_16UC1, (void*)depthImage.get_buffer());


                    if (toggle) {
                        cv::imshow("Device " + std::to_string(i) + " - Color Image", colorMats[i]);
                    }
                    else {
                        cv::imshow("Device " + std::to_string(i) + " - Depth Image", depthMats[i]);
                    }
                }

                k4a_capture_release(capture);
            }



            // Check for key input
            char key = cv::waitKey(1);
            if (key == 'q') {
                break;
            }
            else if (key == 's') {
                for (size_t i = 0; i < devices.size(); ++i) {
                    if (!colorMats[i].empty()) {

                        std::string pathcolorimg = path + "device_" + std::to_string(i) + "_color" + yymmddhhmmss() + ".png";

                        cv::imwrite(pathcolorimg, colorMats[i]);
                        std::cout << "Device " << i << " color image saved to" + pathcolorimg+ "\n";
                    }
                    if (!depthMats[i].empty()) {
                        std::string pathdepthimg = path + "device_" + std::to_string(i) + "_depth" + yymmddhhmmss() + ".png";
                        cv::imwrite(pathdepthimg, depthMats[i]);
                        std::cout << "Device " << i << " depth image saved to" + pathdepthimg +"\n";
                    }
                
                    // Generate unique point cloud file name
                    std::string pathply = path + "device_" + std::to_string(i) + "_" + yymmddhhmmss();

                    // Call f_capture to save point clouds
                    if (depthMats[i].empty() || colorMats[i].empty()) {
                        std::cerr << "Device " << i << ": Skipping point cloud generation due to missing data.\n";
                    }
                    
                    try {
                        f_capture(
                            pathply,
                            lastDepthImages[i].handle(),        // Pass the raw handle
                            lastColorImages[i].handle(),        // Pass the raw handle
                            transformations[i],         // Transformation for the device
                            calibrations[i],            // Calibration for the device
                            lastCaptures[i]                     // Capture object for the device
                        );
                        std::cout << "Device " << i << " point cloud saved.\n";
                    }
                    catch (const std::exception& e) {
                        std::cerr << "Device " << i << ": Error saving point cloud - " << e.what() << "\n";
                    }
                
                }
            }
            else if (key == 't') {
                toggle = !toggle;
            }
        }
    }
    catch (const k4a::error& e) {
        std::cerr << "Azure Kinect error: " << e.what() << std::endl;
        return 1;
    }

    // Free resources
    for (size_t i = 0; i < devices.size(); ++i) {
        if (transformations[i] != NULL) {
            k4a_transformation_destroy(transformations[i]);
        }
        if (devices[i] != NULL) {
            k4a_device_close(devices[i]);
        }
    }

    return 0;
}
