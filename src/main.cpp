
#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <boost/format.hpp> // for setting image filename

#include "myslam/config.h"

int main ( int argc, char** argv )
{
    // -----------------------------------------------------------
    // Check input arguments: Path to the configuration file
    //      which stores the dataset_dir and camera_info
    const int NUM_ARGUMENTS=1;
    if ( argc-1 != NUM_ARGUMENTS )
    {
        cout<<"Lack arguments: Please input the path to the .yaml config file"<<endl;
        return 1;
    }
    myslam::Config::setParameterFile ( argv[1] );

    // -----------------------------------------------------------
    // Read in all image paths
    cout << endl;
    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    boost::format filename_fmt( dataset_dir + "/rgb_%05d.png" );
    vector<string> image_filenames;
    for (int i=0; i<5; i++){
        image_filenames.push_back((filename_fmt % i).str());
    }
    cout << "Reading from dataset_dir: "<< dataset_dir <<endl;
    cout << "Print the first 5 image filenames:" << endl;
    for (auto s: image_filenames)
        cout << s << endl;
    cout << endl;

    // -----------------------------------------------------------


}