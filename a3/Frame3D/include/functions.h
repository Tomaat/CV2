/**
 *  Functions.h  
 *
 *  Header file for the main assignment
 *
 *  @author David van Erkelens (10264019) <david.vanerkelens@student.uva.nl>
 *  @author Ysbrand Galama (10262067) <ysbrand.galama@student.uva.nl>
 */

/**
 *  Include guard
 */
#pragma once

/**
 * Dependencies
 */
#include <iostream>
#include "FileUtils.h"
#include "Frame3D.h"

/**
 *  Point Cloud Library (PCL) dependencies
 */
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

/**
 *  Set up namespace
 */
namespace Functions3D { 

/**
 *  mergingPointClouds function
 *
 *  This function merges multiple point clouds to one point cloud
 *  @param std::vector<Frame3D> frames
 *  @return pcl::PointCloud<pcl::PointXYZ>::Ptr
 */
pcl::PointCloud<pcl::PointNormal>::Ptr mergeFrames(const std::vector<Frame3D> &frames);

pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const cv::Mat &depth_image, double focal_length, double max_depth);
pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::PointNormal>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, Eigen::Matrix4f camera_pose);


/**
 *  texture function
 *
 *  This function colors the 3D model returned by the mergePointClouds function
 *
 *  @param point cloud mesh
 *  @param std::vector<Frame3D> frames
 */
void texture(int mesh, std::vector<Frame3D> frames);

/**
 *  End of namespace
 */
}

