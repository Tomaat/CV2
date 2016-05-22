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
    auto merged = Functions3D::mergeFrames(frames, "pointcloud2.ply");

    /**
     *  Try to create a mesh from the point cloud
     */
    auto mesh = Functions3D::createMesh(merged, "notimplemented", "mesh2.vtk");

    return 0;
}