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
 *  @param 3dframes
 *  @return point cloud
 */
int Functions3D::mergePointClouds(int frames)
{
    return frames + 1;
}

/**
 *  texture function
 *
 *  This function colors the 3D model returned by the mergePointClouds function
 *
 *  @param point cloud mesh
 *  @param 3dframes
 */
void Functions3D::texture(int mesh, int frames)
{
    std::cout << "Mesh: " << mesh << " 3dframes: " << frames << std::endl;
}