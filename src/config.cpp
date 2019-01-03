
#include "my_basics/config.h"

namespace my_basics
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr ) // if no instance, create one. So there will be at most one instance.
        config_ = shared_ptr<Config>(new Config);

    /* !!! In OpenCV 4.0, I have to use "new" to get the FileStorage.new */
    /* !!! In OpenCV 3.2, There is no need to use new.  */
    cv::FileStorage *fs=new cv::FileStorage( filename, cv::FileStorage::READ );
    if ( fs->isOpened() == false ){
            std::cerr<<"Parameter file "<<filename<<" does not exist."<<std::endl;
            return;
        }
    config_->file_ = *fs;

}


Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
    // delete &file_;
}

shared_ptr<Config> Config::config_ = nullptr;

}
