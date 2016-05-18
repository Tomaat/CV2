/**
 *  Functions.h  
 *
 *  Header file for the main assignment
 *
 *  @author David van Erkelens (10264019) <david.vanerkelens@student.uva.nl>
 *  @author Ysbrand Galama () <ysbrand.galama@student.uva.nl>
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
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
 
/**
 *  Set up namespace
 */
namespace Functions3D { 

/**
 *  mergingPointClouds function
 *
 *  This function merges multiple point clouds to one point cloud
 *  @param std::vector<Frame3D> frames
 *  @return point cloud
 */
int mergePointClouds(std::vector<Frame3D> frames);

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

