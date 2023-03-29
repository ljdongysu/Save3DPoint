#include <iostream>
// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "point_cloud.h"

int main(int argc, char ** argv)
{
    bool showDiff = false;

    if (argc == 4)
    {
        showDiff = argv[3];
    }
    else if (argc != 3)
    {
        std::cout << "input error, please input: " << std::endl;
        std::cout << "\tdepth or disparity image file path: [string]" << std::endl;
        std::cout << "\tcamera config file path: [string]" << std::endl;
    }

    std::string depthImageFile = argv[1];
    std::string cameraFile = argv[2];

    cv::Mat depth = cv::imread(depthImageFile);

    psl::CameraParam camera;
    GetCameraParameter(cameraFile, camera);

    DepthPointCloud depthPointCloud;
    PointCloud pointCloud;

    depthPointCloud.SetParameter(camera);

    depthPointCloud.Depth2PointCloud(depth, pointCloud);

    if (showDiff) ShowDiff(pointCloud, depth);

    return 0;
}
