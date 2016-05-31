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
 *  mergeFrames function
 *
 *  This function merges multiple point clouds to one point cloud
 *  @param std::vector<Frame3D> frames
 *  @param std::string filename     the filename to save to
 *  @return point cloud
 */
pcl::PointCloud<pcl::PointNormal>::Ptr Functions3D::mergeFrames(const std::vector<Frame3D> &frames, std::string filename)
{
    /**
     *  Initialize a new point cloud
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr total_cloud(new pcl::PointCloud<pcl::PointNormal>);

    /**
     *  Check if the file exists, otherwise process all frames
     */
    if (pcl::io::loadPLYFile<pcl::PointNormal>("../data/" + filename, *total_cloud) == -1)
    {
        /**
         *  Loop over all frames in the vector
         */
        for (size_t i = 0; i < frames.size(); i++) 
        {
            std::cout << "Processing frame " << i << std::endl;
        
            // Save reference instead of copy
            const Frame3D &current_frame = frames[i];

            // get data from the frame
            cv::Mat depth = current_frame.depth_image_;
            double focal = current_frame.focal_length_;
            const Eigen::Matrix4f camera_pose = current_frame.getEigenTransform();

            // compute cloud nessecary to append the point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud = depthToPointCloud(depth, focal, 1);
            pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud = computeNormals(pcloud);
            pcl::PointCloud<pcl::PointNormal>::Ptr transformed_normal_cloud = transformPointCloud(normal_cloud, camera_pose);

            // add the cloud to the existing point cloud
            addPointCloud(total_cloud, transformed_normal_cloud);
        }

        // @todo the combination of this with the load function does not work yet
        pcl::io::savePLYFile("../data/" + filename, *total_cloud);
    }

    /**
     *  Convert to XYZ to show the cloud
     *  uncomment other code to show the point cloud
     */
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*total_cloud, *cloud);

    // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    // viewer.showCloud(cloud);
    // while (!viewer.wasStopped()) {}

    /**
     *  Return the concatenated clouds
     */
    return total_cloud;
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
pcl::PointCloud<pcl::PointXYZ>::Ptr Functions3D::depthToPointCloud(const cv::Mat &depth_image, double focal_length, double max_depth = 1.2)
{
    /**
     *  Initialize a new point cloud to save the data
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    /**
     *  Get the data required to add the points to the point cloud
     */
    const float inv_focal_length = 1.0 / focal_length;
    const int half_x = depth_image.cols / 2;
    const int half_y = depth_image.rows / 2;

    cloud->points.reserve(depth_image.rows * depth_image.cols);

    /**
     *  Iterate over the depth image and add points to the point cloud
     */
    for(int y = 0; y < depth_image.rows; y++)
    {
        for (int x = 0; x < depth_image.cols; x++)
        {
            // fetch the depth value
            float value = depth_image.at<ushort>(cv::Point(x, y)) * 0.001;

            // if the depth value is within range, add it to the cloud...
            if (value > 0 && value < max_depth)
            {
                pcl::PointXYZ point((x - half_x) * value * inv_focal_length, (y - half_y) * value * inv_focal_length, value);
                cloud->push_back(point);
            }

            // .. otherwise add a NaN value
            else
            {
                pcl::PointXYZ point(x, y, NAN);
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
 *  TransformPointCloud function
 *
 *  This function takes a point cloud and transforms it according to the camera pose
 *  @param  normal_cloud the cloud with normals
 *  @param  camera_pose  the camera pose
 *  @return              the cloud transformed
 */
pcl::PointCloud<pcl::PointNormal>::Ptr Functions3D::transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, Eigen::Matrix4f camera_pose)
{
    /**
     *  Initialize a new point cloud to save the data
     */
    pcl::PointCloud<pcl::PointNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::transformPointCloudWithNormals(*normal_cloud, *new_cloud, camera_pose);
    return new_cloud;
}

/**
 *  Helper function to concatenate point clouds, with help from TA via Github
 *  @see https://github.com/Tomaat/CV2/issues/1
 *
 *  @param  cloud_base      the cloud to which the other cloud will be appended
 *  @param  cloud_add       the cloud to add to the other cloud
 */
void Functions3D::addPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_base, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_add) 
{
    // get the points from the clouds
    auto &v_base = cloud_base->points;
    auto &v_add  = cloud_add->points;

    // reserve enough space for all clouds
    v_base.reserve(v_base.size() + v_add.size());

    // loop over the points in the cloud to add
    for(const auto &p : v_add) 
    {
        // add the point to the base cloud if the value and normal are not NaN
        if (!isnan(p.z) && !isnan(p.normal_z)) v_base.emplace_back(p);
    }
}


/**
 *  Function to create a mesh from a concatenated point cloud
 *
 *  Based on code from the PCL Documentation
 *  @see http://pointclouds.org/documentation/tutorials/greedy_projection.php
 *
 *  @param  cloud       the cloud to generate a mesh from
 *  @param  type        the algorithm to run the generation for (poisson, marching, greedy)
 *  @param  filename    the filename to save the mesh to
 *  @return             the mesh created
 */
pcl::PolygonMesh Functions3D::createMesh(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, std::string type, std::string filename)
{
    // create a search tree for the algorithm
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud);

    // create the polygonmesh variable
    pcl::PolygonMesh triangles;

    // check for multiple reconstruction types
    if (type == "poisson")
    {
        // Initialize reconstructor
        pcl::Poisson<pcl::PointNormal> reconstructor;
       
        // set parameters
        reconstructor.setDepth(12.0);
        reconstructor.setSamplesPerNode(14.0);

        // Get result
        reconstructor.setInputCloud(cloud);
        reconstructor.setSearchMethod(tree);
        reconstructor.reconstruct(triangles);
    }
    else if (type == "marching")
    {
        // Initialize reconstructor
        pcl::MarchingCubesHoppe<pcl::PointNormal> reconstructor;

        // set parameters
        // @todo optimize these parameters
        reconstructor.setGridResolution(75, 75, 75);
        // reconstructor.setIsoLevel(0.05);
        // reconstructor.setPercentageExtendGrid(15.0);

        // get result
        reconstructor.setInputCloud(cloud);
        reconstructor.setSearchMethod(tree);
        reconstructor.reconstruct(triangles);
    }
    else if (type == "greedy")
    {
        // Initialize reconstructor
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> reconstructor;

        // Set the maximum distance between connected points (maximum edge length)
        reconstructor.setSearchRadius(0.25);

        // Set typical values for the parameters
        reconstructor.setMu(2.5);
        reconstructor.setMaximumNearestNeighbors(5000);
        reconstructor.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        reconstructor.setMinimumAngle(M_PI/18); // 10 degrees
        reconstructor.setMaximumAngle(2*M_PI/3); // 120 degrees
        reconstructor.setNormalConsistency(false);

        // Get result
        reconstructor.setInputCloud(cloud);
        reconstructor.setSearchMethod(tree);
        reconstructor.reconstruct(triangles);
    }
    else
    {
        // type not known, throw error
        throw std::runtime_error(std::string("Mesh reconstructor type '" + type + "' not known"));
    }

    // save the mesh to a file 
    pcl::io::saveVTKFile("../data/" + filename, triangles);

    // return the mesh
    return triangles;
}


/**
 *  texture function
 *
 *  This function colors the 3D model returned by the mergePointClouds function
 *
 *  @param point cloud mesh
 *  @param std::vector<Frame3D> frames
 */
void Functions3D::texture(pcl::PolygonMesh mesh, std::vector<Frame3D> frames, std::string filename)
{
    // load values from the mesh
    auto polygons = mesh.polygons;

    // create cloud to save the points in
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());;

    // create clouds to add the rgb points to
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());;

    // copy mesh cloud to both placeholders
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_rgb);

    // create texture mapping variable
    auto texture_mapping = new pcl::TextureMapping<pcl::PointXYZ>();

    // save uv coordinates
    Eigen::Vector2f uv_coordinates;

    /**
     *  Loop over all frames in the vector
     */
    for (int i = frames.size()-1; i>=0; i--)
    {
        // Save reference instead of copy
        const Frame3D &current_frame = frames[i];

        // get data from the frame
        auto depth = current_frame.depth_image_;
        auto focal = current_frame.focal_length_;
        auto depth_map_size = depth.size();
        auto rgb_image = current_frame.rgb_image_;

        // save and resize rgb image
        cv::Mat rgb_img;
        cv::resize(rgb_image, rgb_img, depth_map_size);

        // inverse the camera pose
        auto camera_pose = current_frame.getEigenTransform();
        camera_pose(3,3) = 1;
        auto inv_camera = camera_pose.inverse().eval();

        // create texture mapping camera
        pcl::texture_mapping::Camera camera;
        camera.focal_length = focal;
        camera.height = rgb_img.size().height;
        camera.width = rgb_img.size().width;

        // reverse the point cloud transformation
        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud, *trans_cloud, inv_camera);

        // create octree for texture mapping
        pcl::TextureMapping<pcl::PointXYZ>::Octree::Ptr octree(new pcl::TextureMapping<pcl::PointXYZ>::Octree(0.01));
        octree->setInputCloud(trans_cloud->makeShared());
        octree->addPointsFromInputCloud();

        /**
         *  Loop over all polygons
         */
        for (pcl::Vertices polygon : polygons) 
        {
            /**
             *  Loop over all vertices in the polygon
             */
            for (uint32_t vertex : polygon.vertices)
            {
                // check if the point is occluded
                auto vertex_point = trans_cloud->points.at(vertex);
                if (!texture_mapping->isPointOccluded(vertex_point, octree)) 
                {
                    // if it is not, get the uv coordinates
                    if (texture_mapping->getPointUVCoordinates(vertex_point, camera, uv_coordinates)) 
                    {
                        // get the coordinates
                        int x = round(uv_coordinates.x() * camera.width);
                        int y = round(camera.height - uv_coordinates.y() * camera.height);
                        float z = depth.at<ushort>(y, x) * 0.001;

                        // if they have the correct depth, save them
                        if (z < 1.2) 
                        {
                            auto pixel = rgb_img.at<cv::Vec3b>(cv::Point(x, y));
                            cloud_rgb->points.at(vertex).b = pixel[0];
                            cloud_rgb->points.at(vertex).g = pixel[1];
                            cloud_rgb->points.at(vertex).r = pixel[2];
                        }
                    }
                }
            }
        }
    }

    // set the rgb cloud as point cloud for the mesh
    pcl::toPCLPointCloud2(*cloud_rgb, mesh.cloud);

    // save the mesh to a file 
    pcl::io::saveVTKFile("../data/" + filename, mesh);
}