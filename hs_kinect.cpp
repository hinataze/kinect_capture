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


// CHANGES FOR MEMORY MANAGMENT 
class K4ACaptureWrapper
{
public:
    // Construct from a raw k4a_capture_t
    explicit K4ACaptureWrapper(k4a_capture_t capture = nullptr)
        : capture_(capture)
    {
    }

    // No copy allowed (to prevent double-free)
    K4ACaptureWrapper(const K4ACaptureWrapper&) = delete;
    K4ACaptureWrapper& operator=(const K4ACaptureWrapper&) = delete;

    // Allow move
    K4ACaptureWrapper(K4ACaptureWrapper&& other) noexcept
        : capture_(other.capture_)
    {
        other.capture_ = nullptr;
    }
    K4ACaptureWrapper& operator=(K4ACaptureWrapper&& other) noexcept
    {
        if (this != &other)
        {
            release();
            capture_ = other.capture_;
            other.capture_ = nullptr;
        }
        return *this;
    }

    // Destructor releases the capture if valid
    ~K4ACaptureWrapper()
    {
        release();
    }

    // Explicit release
    void release()
    {
        if (capture_)
        {
            k4a_capture_release(capture_);
            capture_ = nullptr;
        }
    }

    // Access the raw handle
    k4a_capture_t handle() const { return capture_; }

    // Check validity
    bool isValid() const { return (capture_ != nullptr); }

private:
    k4a_capture_t capture_ = nullptr;
};

bool toggle = true;

int main()
{
    const int32_t TIMEOUT_IN_MS = 10000;
    uint32_t device_count = 0;

    // Vectors for devices, transformations, and calibrations
    std::vector<k4a_device_t>          devices;
    std::vector<k4a_transformation_t>  transformations;
    std::vector<k4a::calibration>      calibrations;

    try
    {
        std::cout << "Starting capture functions" << std::endl;
        // Get the number of connected devices
        device_count = k4a_device_get_installed_count();
        if (device_count == 0)
        {
            std::cerr << "No K4A devices found\n";
            return 0;
        }

        // Reserve space to avoid repeated allocations
        devices.reserve(device_count);
        transformations.reserve(device_count);
        calibrations.reserve(device_count);

        // Open and configure each device
        for (uint32_t i = 0; i < device_count; ++i)
        {
            k4a_device_t device = nullptr;
            if (K4A_RESULT_SUCCEEDED != k4a_device_open(i, &device))
            {
                std::cerr << "Failed to open device " << i << "\n";
                continue;
            }
            devices.push_back(device);

            // Configure the device
            k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
            config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
            config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
            config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
            config.camera_fps = K4A_FRAMES_PER_SECOND_15;
            config.synchronized_images_only = true;

            // Get calibration
            k4a::calibration calib;
            if (K4A_RESULT_SUCCEEDED !=
                k4a_device_get_calibration(device,
                    config.depth_mode,
                    config.color_resolution,
                    &calib))
            {
                std::cerr << "Failed to get calibration for device " << i << "\n";
                k4a_device_close(device);
                continue;
            }
            calibrations.push_back(calib);

            // Create transformation
            k4a_transformation_t transformation = k4a_transformation_create(&calib);
            transformations.push_back(transformation);

            // Start cameras
            if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
            {
                std::cerr << "Failed to start cameras for device " << i << "\n";
                k4a_device_close(device);
                continue;
            }
            std::cout << "Device " << i << " cameras started\n";
        }

        if (devices.empty())
        {
            std::cerr << "No devices successfully started\n";
            return 1;
        }

        // Prepare OpenCV Mat vectors (size = number of valid devices)
        std::vector<cv::Mat> colorMats(devices.size());
        std::vector<cv::Mat> depthMats(devices.size());

        // Create OpenCV windows for each device
        for (size_t i = 0; i < devices.size(); ++i)
        {
           cv::namedWindow("Device " + std::to_string(i) + " - Color Image", cv::WINDOW_NORMAL);
        }

        bool toggle = true;

        // Store k4a::image wrappers and captures
        std::vector<k4a::image>      lastDepthImages(devices.size());
        std::vector<k4a::image>      lastColorImages(devices.size());
        std::vector<K4ACaptureWrapper> lastCaptures(devices.size());



        // Main capture loop
        while (true)
        {
            for (size_t i = 0; i < devices.size(); ++i)
            {
                if (!devices[i]) // in case a device failed earlier
                    continue;

                // Attempt to get a capture
                k4a_capture_t rawCapture = nullptr;
                k4a_wait_result_t result = k4a_device_get_capture(
                    devices[i], &rawCapture, TIMEOUT_IN_MS);

                if (result == K4A_WAIT_RESULT_SUCCEEDED)
                {
                    // Release the old capture before overwriting
                    lastCaptures[i].release();

                    // Store the new valid capture in the RAII wrapper
                    lastCaptures[i] = K4ACaptureWrapper(rawCapture);
                }
                else if (result == K4A_WAIT_RESULT_TIMEOUT)
                {
                    std::cerr << "Device " << i << " timed out waiting for a capture\n";
                    continue;
                }
                else // K4A_WAIT_RESULT_FAILED
                {
                    std::cerr << "Device " << i << " failed to get a capture\n";
                    continue;
                }

                // Process depth and color images
                if (lastCaptures[i].isValid())
                {
                    // Wrap them in k4a::image (C++ wrapper)
                    k4a::image depthImage = k4a_capture_get_depth_image(lastCaptures[i].handle());
                    k4a::image colorImage = k4a_capture_get_color_image(lastCaptures[i].handle());

                    if (depthImage.is_valid() && colorImage.is_valid())
                    {
                        // Store for later usage
                        lastDepthImages[i] = depthImage;
                        lastColorImages[i] = colorImage;

                        // Clone buffers into cv::Mat to avoid referencing
                        // Kinect memory that may be freed
                        colorMats[i] = cv::Mat(
                            colorImage.get_height_pixels(),
                            colorImage.get_width_pixels(),
                            CV_8UC4,
                            colorImage.get_buffer()
                        ).clone();

                        depthMats[i] = cv::Mat(
                            depthImage.get_height_pixels(),
                            depthImage.get_width_pixels(),
                            CV_16UC1,
                            depthImage.get_buffer()
                        ).clone();

                        cv::Mat displayColor, displayDepth;
                        cv::resize(colorMats[i], displayColor, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
                        cv::resize(depthMats[i], displayDepth, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);


                        if (toggle)
                        {
                            cv::imshow("Device " + std::to_string(i) + " - Color Image", displayColor);
                        }
                        else
                        {
                            cv::imshow("Device " + std::to_string(i) + " - Depth Image", displayDepth);
                        }
                    }
                }
            }

            // Check for key input
            char key = static_cast<char>(cv::waitKey(1));
            if (key == 'q')
            {
                break;
            }
            else if (key == 's')
            {
                for (size_t i = 0; i < devices.size(); ++i)
                {
                    // Save color image
                    if (!colorMats[i].empty())
                    {
                        std::string pathColorImg = path + "device_" +
                            std::to_string(i) + "_color_" + yymmddhhmmss() + ".png";
                        if (cv::imwrite(pathColorImg, colorMats[i]))
                        {
                            std::cout << "Device " << i
                                << " color image saved to " << pathColorImg << "\n";
                        }
                    }
                    // Save depth image
                    if (!depthMats[i].empty())
                    {
                        std::string pathDepthImg = path + "device_" +
                            std::to_string(i) + "_depth_" + yymmddhhmmss() + ".png";
                        if (cv::imwrite(pathDepthImg, depthMats[i]))
                        {
                            std::cout << "Device " << i
                                << " depth image saved to " << pathDepthImg << "\n";
                        }
                    }

                    // Generate unique point cloud file name
                    std::string pathPointCloud = path + "device_" +
                        std::to_string(i) + "_" + yymmddhhmmss();

                    // Call f_capture to save point clouds
                    if (depthMats[i].empty() || colorMats[i].empty())
                    {
                        std::cerr << "Device " << i
                            << ": Skipping point cloud generation due to missing data.\n";
                        continue;
                    }

                    // Make sure we have valid images & captures
                    if (!lastDepthImages[i].is_valid() ||
                        !lastColorImages[i].is_valid() ||
                        !lastCaptures[i].isValid())
                    {
                        std::cerr << "Device " << i
                            << ": Missing valid depth/color/capture.\n";
                        continue;
                    }

                    try
                    {
                        f_capture(
                            pathPointCloud,
                            lastDepthImages[i].handle(),   // Pass the raw handle
                            lastColorImages[i].handle(),   // Pass the raw handle
                            transformations[i],            // Transformation for the device
                            calibrations[i],               // Calibration for the device
                            lastCaptures[i].handle()       // Capture object for the device
                        );
                        std::cout << "Device " << i << " point cloud saved.\n";
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "Device " << i
                            << ": Error saving point cloud - " << e.what() << "\n";
                    }
                }
            }
            else if (key == 't')
            {
                toggle = !toggle;
            }
        }
    }
    catch (const k4a::error& e)
    {
        std::cerr << "Azure Kinect error: " << e.what() << std::endl;
        return 1;
    }

    //--------------------------------------------------------------------------
    // Cleanup resources
    //--------------------------------------------------------------------------
    for (size_t i = 0; i < devices.size(); ++i)
    {
        if (transformations[i] != nullptr)
        {
            k4a_transformation_destroy(transformations[i]);
            transformations[i] = nullptr;
        }
        if (devices[i] != nullptr)
        {
            k4a_device_close(devices[i]);
            devices[i] = nullptr;
        }
    }

    return 0;
}