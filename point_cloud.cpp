//
// Created by donglijian on 3/29/23.
//

# include "point_cloud.h"

DepthPointCloud::DepthPointCloud()
        : camera_factor(1000), camera_cx(320), camera_cy(200),
        camera_fx(320), camera_fy(200)
{

}

DepthPointCloud::DepthPointCloud(float camera_factor, float camera_cx, float camera_cy,
                                 float camera_fx, float camera_fy)
{
    this->camera_factor = camera_factor;
    this->camera_fx = camera_fx;
    this->camera_fy = camera_fy;
    this->camera_cx = camera_cx;
    this->camera_cy = camera_cy;
}

void DepthPointCloud::SetParameter(float camera_factor, float camera_cx, float camera_cy, float camera_fx,
                                   float camera_fy)
{
    this->camera_factor = camera_factor;
    this->camera_fx = camera_fx;
    this->camera_fy = camera_fy;
    this->camera_cx = camera_cx;
    this->camera_cy = camera_cy;
}

void DepthPointCloud::SetParameter(const psl::CameraParam &cameraParam)
{
    this->camera_fx = cameraParam._P[0];
    this->camera_fy = cameraParam._P[5];
    this->camera_cx = cameraParam._P[2];
    this->camera_cy = cameraParam._P[6];
}


void DepthPointCloud::Depth2PointCloud(const cv::Mat &depth, PointCloud &pointCloud)
{
    if (depth.empty())
    {
        std::cout << "depth image empty, exit" << std::endl;
        return;
    }

    for (int m = 0; m < depth.rows; m++)
    {
        std::vector<PointCloudPoint> pointCloudPoints;
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            //            ushort d = depth.ptr<ushort>(m)[n];
            //            unsigned int d = depth.at<uchar>(m,n*3);

            //使用视差图
            unsigned int d = depth.at<uchar>(m, n);

            //使用深度图
            if (depth.channels() == 3)
            {
                d = depth.at<uchar>(m, n * 3) + depth.at<uchar>(m, n * 3 + 1) + depth.at<uchar>(m, n * 3 + 2);
            }

            //使用视差图
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointCloudPoint p;

            p.z = double(d) / camera_factor;

            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            pointCloudPoints.push_back(p);
        }
        pointCloud.pointCloudPoints.push_back(pointCloudPoints);
    }
}

bool GetCameraParameter(const std::string& configFile, psl::CameraParam &camera)
{
    cv::FileStorage fileStream = cv::FileStorage(configFile, cv::FileStorage::READ);

    if (not fileStream.isOpened())
    {
        std:: cout <<"file not exist <" + configFile + ">";
        return false;
    }

    // TODO : the exception for lack option
    cv::Mat_<double> kl, dl, pl, rl;
    fileStream["Kl"] >> kl;
    fileStream["Dl"] >> dl;
    fileStream["Pl"] >> pl;
    fileStream["Rl"] >> rl;

    memcpy(camera._K, kl.data, sizeof(camera._K));
    memcpy(camera._R, rl.data, sizeof(camera._R));
    memcpy(camera._P, pl.data, sizeof(camera._P));
    memcpy(camera._D, dl.data, sizeof(camera._D));

    fileStream.release();
    return true;
}

void ShowDiff(const PointCloud &pointCloud, const cv::Mat &depth)
{
    for (int i = 0; i < pointCloud.pointCloudPoints.size(); ++i)
    {
        for (int j = 0; j < pointCloud.pointCloudPoints[0].size(); ++j)
        {
            if((i * pointCloud.pointCloudPoints[0].size() + j) % 10000 == 0)
                std::cout << "index(i:" << i << ", j: "<< j << "): "
                          << i * pointCloud.pointCloudPoints[0].size() + j
                          << "   x: " << pointCloud.pointCloudPoints[i][j].x << ", "
                          << "y: " << pointCloud.pointCloudPoints[i][j].y << ", "
                          << "z: " << pointCloud.pointCloudPoints[i][j].z << std::endl;
        }
    }

    std::cout << "PointCloud size: " << pointCloud.pointCloudPoints.size() * pointCloud.pointCloudPoints[0].size()
              << std::endl;

    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            if((m * depth.cols + n) % 10000 == 0)
            {
                std::cout <<" depth: " << int(depth.at<uchar>(m, n * 3)) << ", "
                          << int(depth.at<uchar>(m, n * 3 + 1)) << ", " << int(depth.at<uchar>(m, n * 3 + 2));
                std::cout << "  cloud Point.z: " << pointCloud.pointCloudPoints[m][n].z << std::endl;
            }
        }
    }
}
