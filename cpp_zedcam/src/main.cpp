#include "header.h"
#include <sl/Camera.hpp>

cv::Mat slMat2cvMat(sl::Mat& input)
{
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM::CPU));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("zed_params_node");
    auto zedPub = std::make_shared<ZEDPublisher>(params);

    /*
     * ZED camera settings
     * ref: https://github.com/stereolabs/zed-examples/blob/master/tutorials/tutorial%203%20-%20depth%20sensing/cpp/main.cpp
     */
    sl::Camera zed;

    // Set configuration parameters
    sl::InitParameters init_parameters;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    if (params->camera_height >= 1080)
        init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
    else if (params->camera_height >= 720)
        init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    else if (params->camera_height >= 360)
        init_parameters.camera_resolution = sl::RESOLUTION::VGA;

    if (params->camera_depth_quality == 0)// Performance mode
        init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    else if (params->camera_depth_quality == 1)// Quality mode
        init_parameters.depth_mode = sl::DEPTH_MODE::QUALITY;
    
    if (params->camera_depth_unit == 1)// Unit: millimeter
        init_parameters.coordinate_units = sl::UNIT::MILLIMETER;
    else if (params->camera_depth_unit == 2)// Unit: centimeter
        init_parameters.coordinate_units = sl::UNIT::CENTIMETER;
    else if (params->camera_depth_unit == 3)// Unit: meter
        init_parameters.coordinate_units = sl::UNIT::METER;
    
    // Open the camera
    sl::ERROR_CODE returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error " << returned_state << ", exit program." << std::endl;
        return EXIT_FAILURE;
    }

    // Set runtime parameters after opening the camera
    sl::RuntimeParameters runtime_parameters;
    if (params->camera_sensing_mode == 0)// Standard mode
        // runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;// SDK ver 3.8
        runtime_parameters.enable_fill_mode = false;
    else if (params->camera_sensing_mode == 1)// Fill mode
        // runtime_parameters.sensing_mode = sl::SENSING_MODE::FILL;// SDK ver 3.8
        runtime_parameters.enable_fill_mode = true;
    
    sl::Mat rgbslMat, depthslMat;
    /*
     * ZED camera settings end
     */


    sl::Resolution pubRGBImgSize(params->topic_ZEDCam_RGB_width, params->topic_ZEDCam_RGB_height);
    sl::Resolution pubDepthImgSize(params->topic_ZEDCam_Depth_width, params->topic_ZEDCam_Depth_height);
    cv::Mat rgbMat, depthMat;

    std::vector<int> encodeParam;
    encodeParam.push_back(cv::IMWRITE_JPEG_QUALITY);
    encodeParam.push_back(70);
    std::vector<uchar> pubRGBImgVec;
    std::vector<uchar> pubDepthImgVec;

    if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
    {
        if (params->camera_use_color)
        {
            zed.retrieveImage(rgbslMat, sl::VIEW::LEFT, sl::MEM::CPU, pubRGBImgSize);
            if (rgbslMat.getResolution() != pubRGBImgSize)
                printf("The RGB image will be resized into %dx%d. Current size: %ldx%ld\n", 
                    pubRGBImgSize.width, pubRGBImgSize.height, rgbslMat.getWidth(), rgbslMat.getHeight());
            zedPub->startRGBPub();
        }
        if (params->camera_use_depth)
        {
            zed.retrieveMeasure(depthslMat, sl::MEASURE::DEPTH, sl::MEM::CPU, pubDepthImgSize);
            if (depthslMat.getResolution() != pubDepthImgSize)
                printf("The Depth image will be resized into %dx%d. Current size: %ldx%ld\n", 
                    pubDepthImgSize.width, pubDepthImgSize.height, depthslMat.getWidth(), depthslMat.getHeight());
            zedPub->startDepthPub();
        }
    }
    else
    {
        std::cerr << "Unable to retrieve image\n";
        zed.close();
        return EXIT_FAILURE;
    }

    while (1)
    {
        // Grab image from ZED camera
        if (zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
        {
            if (params->camera_use_color)
            {
                zed.retrieveImage(rgbslMat, sl::VIEW::LEFT, sl::MEM::CPU, pubRGBImgSize);
                rgbMat = slMat2cvMat(rgbslMat);
            }
            if (params->camera_use_depth)
            {
                zed.retrieveMeasure(depthslMat, sl::MEASURE::DEPTH, sl::MEM::CPU, pubDepthImgSize);
                //depthMat = slMat2cvMat(depthslMat);
            }

            // Process image size and publish to ROS2 topic
            if (params->camera_use_color)
            {
                cv::imencode(".jpg", rgbMat, pubRGBImgVec, encodeParam);
                zedPub->setRGBMat(pubRGBImgVec, rgbMat.size(), rgbMat.type());
            }
            if (params->camera_use_depth)
            {
                uint32_t step = depthslMat.getStepBytes();
                size_t size = step * depthslMat.getHeight();
                uint8_t * data_ptr = nullptr;
                if (depthslMat.getDataType() == sl::MAT_TYPE::F32_C1)
                {
                    data_ptr = reinterpret_cast<uint8_t *>(depthslMat.getPtr<sl::float1>());
                    pubDepthImgVec = std::vector<uint8_t>(data_ptr, data_ptr + size);
                    zedPub->setDepthMat(pubDepthImgVec, cv::Size(depthslMat.getWidth(), depthslMat.getHeight()), CV_32FC1, params->camera_depth_unit);
                }
            }
        }
        else
        {
            std::cerr << "Unable to retrieve image\n";
            zed.close();
            return EXIT_FAILURE;
        }
    }
    rclcpp::shutdown();
    zed.close();
    return EXIT_SUCCESS;
}