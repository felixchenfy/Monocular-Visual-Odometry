/* This script is mainly copied and then modified from Chapter 9 of Dr. Xiang Gao's book. Link is here:
https://github.com/gaoxiang12/slambook/blob/master/project/0.4/include/myslam/config.h
The modification is that in OpenCV 4.0, I have to use "new" to create a cv::FileStorage instance.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>

namespace basics
{
using namespace std;

class Config
{

public:
  // set a new config file
  static void setParameterFile(const std::string &filename);

  // access the parameter values
  template <typename T>
  static T get(const std::string &key);

  template <typename T>
  static std::vector<T> getVector(const std::string &key);

  static bool getBool(const std::string &key);

  ~Config(); // close the file when deconstructing

private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

  /* @brief: Get content by key. If key doesn't exist, throw runtime error.
   */
  static cv::FileNode get_(const std::string &key);

  Config() // private constructor makes a singleton
  {
  }

};

// Get a content of cv::FileNode. Convert to type T.
template <typename T>
T Config::get(const std::string &key) 
{
  cv::FileNode content = Config::get_(key);
  return static_cast<T>(content);
}

// Get a content of cv::FileNode. Convert to type vector<T>.
template <typename T>
std::vector<T> Config::getVector(const std::string &key) 
{
  cv::FileNode content = Config::get_(key);
  std::vector<T> res;
  content >> res;
  return res;
}

} // namespace basics

#endif
