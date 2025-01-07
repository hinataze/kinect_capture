#pragma once

#include <iostream>
#include <fstream>
#include <k4a/k4a.h>
#include <k4arecord/playback.h>
#include <string>
#include "transformation_helpers.h"
#include "turbojpeg.h"
#include <vector>




int f_capture(std::string output_dir, std::string pathdate, k4a_image_t depth_image, k4a_image_t color_image, k4a_transformation_t transformation, k4a_calibration_t calibration, k4a_capture_t capture);
