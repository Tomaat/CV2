/**
 *  Main.cpp  
 *
 *  Implementation for the main assignment
 *
 *  @author David van Erkelens (10264019) <david.vanerkelens@student.uva.nl>
 *  @author Ysbrand Galama () <ysbrand.galama@student.uva.nl>
 */

/**
 * Dependencies
 */
#include "../include/main.h"

/**
 *  Use the STD namespace
 */
using namespace std;

/**
 *  The main function
 *
 *  From this function, the other functions are called
 *  @return [description]
 */
int main() 
{
    cout << "Dit is een test" << endl;

    /**
     *  Load the images in the directory
     */
    vector<Frame3D> frames = Frame3D::loadFrames("../../3dframes"); 




    // Functions3D::texture(1, 2);
    // 
    cout << Functions3D::mergePointClouds(frames) << std::endl;
}