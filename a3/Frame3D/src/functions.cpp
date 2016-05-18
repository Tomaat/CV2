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
int Functions3D::mergePointClouds(std::vector<Frame3D> frames)
{
    /**
     *  Loop over all frames in the vector
     */
    for (size_t i = 0; i < frames.size(); i++ ) {
        // Save reference instead of copy
        const Frame3D &current_frame = frames[i];
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    // //Fillintheclouddata
    cloud->width=50;
    cloud->height=1;
    cloud->is_dense=false;
    cloud->points.resize(cloud->width*cloud->height);
    for(size_t i=0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x=1024*rand()/(50+1.0f);
        cloud->points[i].y=1024*rand()/(50+1.0f);
        cloud->points[i].z=1024*rand()/(50+1.0f);
    }
    // pcl::io::savePCDFileASCII("testpcd.pcd", cloud);

    // pcl::PointCloud<pcl::PointXYZRGB> cloud;
   //... populate cloud
    ::pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {}


    return 1;
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