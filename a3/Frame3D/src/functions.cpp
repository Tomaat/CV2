/**
 *  Functions.cpp  
 *
 *  Implementation for the main assignment
 *
 *  @author David van Erkelens (10264019) <david.vanerkelens@student.uva.nl>
 *  @author Ysbrand Galama (10262067) <ysbrand.galama@student.uva.nl>
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
    for (size_t i = 0; i < frames.size(); i++) 
    {
        if (i < 3) continue;
    
        // Save reference instead of copy
        const Frame3D &current_frame = frames[i];

        // get data from the frame
        cv::Mat depth = current_frame.depth_image_;
        double focal = current_frame.focal_length_;

        auto pcloud = depthToPointCloud(depth, focal, 1);
        auto normal_cloud = computeNormals(pcloud);




        cloud = pcloud;
        break;
    }

    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped()) {}

    return cloud;
}
/**
 *  depthToPointCloud function
 *
 *  Takes a depth image, a focal length and a maximum depth and generates a point cloud with this data.
 *  
 *  @param  depth_image  the depth image
 *  @param  focal_length the focal depth
 *  @param  max_depth    the maximum depth (default is 1.0)
 *  @return              the point cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr Functions3D::depthToPointCloud(const cv::Mat &depth_image, double focal_length, double max_depth = 1.0)
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
 *  computeNormals function
 *
 *  Computes the normals for a given point cloud. Taken from the example from blackboard and
 *  edited to fit in our program
 *
 *  @see http://blackboard.uva.nl/ -> 20152016 CV2 
 *  
 *  @param  cloud
 *  @return cloud with normals
 */
pcl::PointCloud<pcl::PointNormal>::Ptr Functions3D::computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>); // Output datasets
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator;
    normal_estimator.setNormalEstimationMethod(normal_estimator.AVERAGE_3D_GRADIENT);
    normal_estimator.setMaxDepthChangeFactor(0.02f);
    normal_estimator.setNormalSmoothingSize(10.0f);
    normal_estimator.setInputCloud(cloud);
    normal_estimator.compute(*cloud_normals);
    pcl::copyPointCloud(*cloud, *cloud_normals);
    return cloud_normals;
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