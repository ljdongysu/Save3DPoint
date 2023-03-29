//
// Created by donglijian on 3/29/23.
//

#ifndef SAVE3DPOINT_POINT_CLOUD_H
#define SAVE3DPOINT_POINT_CLOUD_H
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace psl
{
    struct CameraParam
    {
        double _TSC[16];  // 4X4 camera to imu
        int _width;
        int _height;

        float b;
        float bf;

        // distortion_type:equidistant
        double _focal_length[2];     // fx,fy
        double _principal_point[2];  // cx,cy
        // Rectification matrix (stereo cameras only)
        // A rotation matrix aligning the camera coordinate system to the ideal
        // stereo image plane so that epipolar lines in both stereo images are
        // parallel.
        double _R[9];
        // Projection/camera matrix
        //     [fx'  0  cx' Tx]
        // P = [ 0  fy' cy' Ty]
        //     [ 0   0   1   0]
        double _P[12];
        // Intrinsic camera matrix for the raw (distorted) images.
        //     [fx  0 cx]
        // K = [ 0 fy cy]
        //     [ 0  0  1]
        double _K[9];
        // The distortion parameters, size depending on the distortion model.
        // For us, the 4 parameters are: (k1, k2, t1, t2).
        double _D[4];

        CameraParam()
        {
        }
    };
}  // namespace psl


struct PointCloudPoint
{
    float x;
    float y;
    float z;
};

struct PointCloud
{
    std::vector<std::vector<PointCloudPoint>> pointCloudPoints;
};

class DepthPointCloud
{
public:
    DepthPointCloud();

    DepthPointCloud(float camera_factor, float camera_cx, float camera_cy, float camera_fx, float camera_fy);
    void SetParameter(float camera_factor, float camera_cx, float camera_cy,
                      float camera_fx, float camera_fy);

    void SetParameter(const psl::CameraParam &cameraParam);

    void Depth2PointCloud(const cv::Mat &depth, PointCloud &pointCloud);

private:

    PointCloud pointCloud;
    double camera_factor = 1000;
    double camera_cx;// = 325.5;
    double camera_cy;// = 253.5;
    double camera_fx;// = 518.0;
    double camera_fy;// = 519.0;
};


bool GetCameraParameter(const std::string& configFile, psl::CameraParam &camera);

void ShowDiff(const PointCloud &pointCloud, const cv::Mat &depth);

#endif //SAVE3DPOINT_POINT_CLOUD_H
