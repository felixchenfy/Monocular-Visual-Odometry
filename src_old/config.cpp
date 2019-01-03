

#include "my_basics/config.h"

namespace my_slam 
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr ) // if no instance, create one. So there will be at most one instance.
        config_ = shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( config_->file_.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        config_->file_.release();
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}
