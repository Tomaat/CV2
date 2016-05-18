/**
 *  Functions.cpp  
 *
 *  Implementation for the main assignment
 *
 *  @author David van Erkelens (10264019) <david.vanerkelens@student.uva.nl>
 *  @author Ysbrand Galama () <ysbrand.galama@student.uva.nl>
 */

/**
 * Dependencies
 */
#include "../include/functions.h"


/**
 *  mergingPointClouds function
 *
 *  This function merges multiple point clouds to one point cloud
 *  @param std::vector<Frame3D> frames
 *  @return point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Functions3D::mergeFrames(const std::vector<Frame3D> &frames)
{
    /**
     *  Initialize a new point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /**
     *  Loop over all frames in the vector
     */
        // Save reference instead of copy
        const Frame3D &current_frame = frames[i];

        // get data from the frame
        cv::Mat depth = current_frame.depth_image_;
        double focal = current_frame.focal_length_;

        cloud = depthToPointCloud(depth, focal, 1);

        break;
    }

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {}

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Functions3D::depthToPointCloud(const cv::Mat &depth_image, double focal_length, double max_depth)
{
    /**
     *  Initialize a new point cloud to save the data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    const float inv_focal_length = 1.0 / focal_length;
    const int half_x = depth_image.cols / 2;
    const int half_y = depth_image.rows / 2;

    /**
     *  Iterate over the depth image and add points to the point cloud
     */
    for(int y = 0; y < depth_image.rows; y++)
    {
        for (int x = 0; x < depth_image.cols; x++)
        {
            float value = depth_image.at<ushort>(cv::Point(x, y)) * 0.001;
            if (value > 0 && value < max_depth)
            {
                pcl::PointXYZ point((x - half_x) * value * inv_focal_length, (y - half_y) * value * inv_focal_length, value);
                cloud->push_back(point);
            }
        }
    }

    cloud->width = depth_image.cols;
    cloud->height = depth_image.rows;

    return cloud;
}

/**
 *  texture function
 *
 *  This function colors the 3D model returned by the mergePointClouds function
 *
 *  @param point cloud mesh
 *  @param std::vector<Frame3D> frames
 */
void Functions3D::texture(int mesh, std::vector<Frame3D> frames)
{
    // std::cout << "Mesh: " << mesh << " 3dframes: " << frames << std::endl;
}