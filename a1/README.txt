//////////// FILES INCLUDED///////////////////////

- ##########.jpeg file       : RGB images recorded
- ##########.pcd file        : point clouds recorded
- ##########_camera.xml file : camera parameters
- ##########_depth.png  file : depth images recorded
- ##########_normal.pcd file : normals extracted
- ##########_mask.jpeg  file : object masks


/////////// M-FILE ///////////////////////////////

- readPcd.m 
This m-file is to read provided pcd files by matlab.

Example usage :
pointCloud = readPcd(pcd_img_path);


///////////////////////////////////////////////////

Note: The provided point clouds does not only consist of person (it also contains background). Please apply a distance threshold to remove background (e.g. remove points further than 2meters to the camera). 