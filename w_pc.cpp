
#include <w_pc.h>




static bool point_cloud_color_to_depth (k4a_transformation_t transformation_handle, const k4a_image_t depth_image, const k4a_image_t color_image,std::string file_name)
{
    std::cout << "point_cloud_color_to_depth  " << std::endl;
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);

    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        depth_image_width_pixels,
        depth_image_height_pixels,
        depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
        &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        depth_image_width_pixels,
        depth_image_height_pixels,
        depth_image_width_pixels * 3 * (int)sizeof(int16_t),
        &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
        depth_image,
        color_image,
        transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
        depth_image,
        K4A_CALIBRATION_TYPE_DEPTH,
        point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, transformed_color_image, file_name.c_str());

    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);

    return true;
}

static bool point_cloud_depth_to_color(k4a_transformation_t transformation_handle, const k4a_image_t depth_image,
const k4a_image_t color_image,
    std::string file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * (int)sizeof(uint16_t),
        &transformed_depth_image))
    {
        printf("Failed to create transformed depth image\n");
        return false;
    }

    k4a_image_t point_cloud_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        color_image_width_pixels,
        color_image_height_pixels,
        color_image_width_pixels * 3 * (int)sizeof(int16_t),
        &point_cloud_image))
    {
        printf("Failed to create point cloud image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED !=
        k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
    {
        printf("Failed to compute transformed depth image\n");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
        transformed_depth_image,
        K4A_CALIBRATION_TYPE_COLOR,
        point_cloud_image))
    {
        printf("Failed to compute point cloud\n");
        return false;
    }

    tranformation_helpers_write_point_cloud(point_cloud_image, color_image, file_name.c_str());


    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);

    return true;
}



//static int f_capture(std::string output_dir, uint8_t deviceId = K4A_DEVICE_DEFAULT)
 int f_capture(std::string output_dir, k4a_image_t depth_image , k4a_image_t color_image , k4a_transformation_t transformation, k4a_calibration_t calibration, k4a_capture_t capture)
{
   int returnCode = 1;

   k4a_transformation_t transformation_color_downscaled = NULL;

   std::string file_name = "";

   k4a_image_t color_image_downscaled = NULL;
 
    // Compute color point cloud by warping color image into depth camera geometry

    file_name = output_dir + "_point_cloud_color_to_depth.ply";
    std::cout << "file_name " <<  file_name << std::endl;

    if (point_cloud_color_to_depth(transformation, depth_image, color_image, file_name.c_str()) == false)
    {
        goto Exit;
    }

    // Compute color point cloud by warping depth image into color camera geometry
    file_name = output_dir + "_point_cloud_depth_to_color.ply";
    std::cout << "file_name " << file_name << std::endl;

    if (point_cloud_depth_to_color(transformation, depth_image, color_image, file_name.c_str()) == false)
    {
        goto Exit;
    }

    // Compute color point cloud by warping depth image into color camera geometry with downscaled color image and
    // downscaled calibration. This example's goal is to show how to configure the calibration and use the
    // transformation API as it is when the user does not need a point cloud from high resolution transformed depth
    // image. The downscaling method here is naively to average binning 2x2 pixels, user should choose their own
    // appropriate downscale method on the color image, this example is only demonstrating the idea. However, no matter
    // what scale you choose to downscale the color image, please keep the aspect ratio unchanged (to ensure the
    // distortion parameters from original calibration can still be used for the downscaled image).
    k4a_calibration_t calibration_color_downscaled;
    memcpy(&calibration_color_downscaled, &calibration, sizeof(k4a_calibration_t));
    calibration_color_downscaled.color_camera_calibration.resolution_width /= 2;
    calibration_color_downscaled.color_camera_calibration.resolution_height /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.cy /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fx /= 2;
    calibration_color_downscaled.color_camera_calibration.intrinsics.parameters.param.fy /= 2;
    transformation_color_downscaled = k4a_transformation_create(&calibration_color_downscaled);
    color_image_downscaled = downscale_image_2x2_binning(color_image);
    if (color_image_downscaled == 0)
    {
        printf("Failed to downscaled color image\n");
        goto Exit;
    }

    file_name = output_dir + "point_cloud_depth_to_color_downscaled.ply";
    std::cout << "file_name " << file_name << std::endl;

    if (point_cloud_depth_to_color(transformation_color_downscaled,
        depth_image,
        color_image_downscaled,
        file_name.c_str()) == false)
    {
        goto Exit;
    }

    returnCode = 0;

Exit:


    return returnCode;
}