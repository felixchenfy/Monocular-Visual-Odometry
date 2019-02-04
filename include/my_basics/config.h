/* This script is mainly copied and then modified from Chapter 9 of Dr. Xiang Gao's book. Link is here:
https://github.com/gaoxiang12/slambook/blob/master/project/0.4/include/myslam/config.h
The modification is that in OpenCV 4.0, I have to use "new" to create a cv::FileStorage instance.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>

namespace my_basics
{
using namespace std;

class Config
{
private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

  Config() {} // private constructor makes a singleton
public:
  ~Config(); // close the file when deconstructing

  // set a new config file
  static void setParameterFile(const std::string &filename);

  // access the parameter values
  template <typename T>
  static T get(const std::string &key)
  {
    return T(Config::config_->file_[key]);
  }

  static bool getBool(const std::string &key)
  {
    string val = string(Config::config_->file_[key]);
    // cout << "getBool:" << val << endl;
    if (val == "true" || val == "True")
      return true; // If I combine this into the above template, the program throws error: could not convert ‘true’ from ‘bool’ to ‘std::__cxx11::basic_string<char>’
    else
      return false;
  }
};
} // namespace my_basics

#endif
