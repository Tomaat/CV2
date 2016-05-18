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

/**
 *  Set up namespace
 */
namespace Functions3D { 

/**
 *  mergingPointClouds function
 *
 *  This function merges multiple point clouds to one point cloud
 *  @param 3dframes
 *  @return point cloud
 */
int mergePointClouds(int frames);

/**
 *  texture function
 *
 *  This function colors the 3D model returned by the mergePointClouds function
 *
 *  @param point cloud mesh
 *  @param 3dframes
 */
void texture(int mesh, int frames);

/**
 *  End of namespace
 */
}

