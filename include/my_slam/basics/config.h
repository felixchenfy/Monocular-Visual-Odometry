/* @brief A singleton configuration class for reading .yaml file.
 *    Modified from: https://github.com/gaoxiang12/slambook/blob/master/project/0.4/include/myslam/config.h
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "my_slam/common_include.h"

namespace my_slam
{
namespace basics
{

class Config
{

public:

  // Set a new config file
  static void setParameterFile(const std::string &filename);

  // Get a content by key.
  template <typename T>
  static T get(const std::string &key);

  // Get a vector of content by key.
  template <typename T>
  static std::vector<T> getVector(const std::string &key);

  // Get a content by key.
  // The content is of type string ("true", "false"). It's then converted to bool and returned.
  static bool getBool(const std::string &key);

  ~Config(); // Close the file when deconstructing

private:
  static std::shared_ptr<Config> config_;
  cv::FileStorage file_;

  /* @brief: Get content by key. 
   *    If key doesn't exist, throw runtime error.
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
} // namespace my_slam

#endif
