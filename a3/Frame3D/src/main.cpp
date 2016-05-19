/**
 *  Main.cpp  
 *
 *  Implementation for the main assignment
 *
 *  @author David van Erkelens (10264019) <david.vanerkelens@student.uva.nl>
 *  @author Ysbrand Galama (10262067) <ysbrand.galama@student.uva.nl>
 */

/**
 * Dependencies
 */
#include "../include/main.h"

/**
 *  Use the std namespace
 */
using namespace std;

/**
 *  The main function
 *
 *  From this function, the other functions are called
 */
int main() 
{
    /**
     *  Load the images in the directory
     */
    vector<Frame3D> frames = Frame3D::loadFrames("3dframes"); 

    /**
     *  Merge the point clouds
     */
    auto merged = Functions3D::mergeFrames(frames);

    /**
     *  Try to create a mesh from the point cloud
     *  Example taken from PCL documentation
     *  @see http://pointclouds.org/documentation/tutorials/greedy_projection.php
     */
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(merged);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.25);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (5000);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (merged);
    gp3.setSearchMethod (tree2);
    
    // THIS SEGFAULTS! WHY?
    gp3.reconstruct(triangles);

    pcl::io::saveVTKFile ("../data/mesh.vtk", triangles);

    return 0;
}