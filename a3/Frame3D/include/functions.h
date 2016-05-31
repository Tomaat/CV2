/**
 *  Functions.h  
 *
 *  Header file for the functions used for the main assignment
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
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/texture_mapping.h>

/**
 *  OpenCV dependencies
 */
#include "opencv2/opencv.hpp"

/**
 *  Set up namespace
 */
namespace Functions3D { 

/**
 *  mergeFrames function
 *
 *  This function merges multiple point clouds to one point cloud
 *  @param std::vector<Frame3D> frames
 *  @param std::string filename     the filename to save to
 *  @return point cloud
 */
pcl::PointCloud<pcl::PointNormal>::Ptr mergeFrames(const std::vector<Frame3D> &frames, std::string filename);

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
pcl::PointCloud<pcl::PointXYZ>::Ptr depthToPointCloud(const cv::Mat &depth_image, double focal_length, double max_depth);

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
pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

/**
 *  TransformPointCloud function
 *
 *  This function takes a point cloud and transforms it according to the camera pose
 *  @param  normal_cloud the cloud with normals
 *  @param  camera_pose  the camera pose
 *  @return              the cloud transformed
 */
pcl::PointCloud<pcl::PointNormal>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, Eigen::Matrix4f camera_pose);

/**
 *  Helper function to concatenate point clouds, with help from TA via Github
 *  @see https://github.com/Tomaat/CV2/issues/1
 *
 *  @param  cloud_base  the cloud to which the other cloud will be appended
 *  @param  cloud_add   the cloud to add to the other cloud
 */
void addPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_base, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_add);

/**
 *  Function to create a mesh from a concatenated point cloud
 *
 *  Based on code from the PCL Documentation
 *  @see http://pointclouds.org/documentation/tutorials/greedy_projection.php
 *
 *  @param  cloud       the cloud to generate a mesh from
 *  @param  type        the algorithm to run the generation for
 *  @param  filename    the filename to save the mesh to
 *  @return             the mesh created
 */
pcl::PolygonMesh createMesh(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string type, std::string filename);

/**
 *  texture function
 *
 *  This function colors the 3D model returned by the mergePointClouds function
 *
 *  @param point cloud mesh
 *  @param std::vector<Frame3D> frames
 */
void texture(pcl::PolygonMesh mesh, std::vector<Frame3D> frames, std::string filename);

/**
 *  End of namespace
 */
}

