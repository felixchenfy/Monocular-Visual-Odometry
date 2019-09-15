/* @brief: Read contents from .yaml file.
 */

#ifndef MY_SLAM_YAML_H
#define MY_SLAM_YAML_H

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <assert.h>

namespace my_slam
{
namespace basics
{

class Yaml
{

public:
    /* @brief: Constructor: constructed by reading a .yaml file.
     */
    Yaml(const std::string &filename);

    /* @brief: Constructor: constructed from existing cv::FileNode.
     */
    Yaml(const cv::FileNode &fn);

    /* @brief: Constructor: constructed from existing cv::FileStorage.
     */
    Yaml(const cv::FileStorage &fs);

    /* @brief: Release file memory.
     */
    ~Yaml();

    // Get a content of cv::FileNode. Convert to Yaml type.
    Yaml get(const std::string &key) const;

    // Get a content of cv::FileNode. Convert to type T.
    template <typename T>
    T get(const std::string &key) const;

    // Get a content of cv::FileNode. Convert to type vector<T>.
    template <typename T>
    std::vector<T> get_vec(const std::string &key) const;

    // Get a string content of cv::FileNode. Convert it to type bool.
    bool getBool(const std::string &key)
    {
        std::string val = static_cast<std::string>(get_(key));
        if (val == "true" || val == "True")
            return true;
        else
            return false;
    }

    /* @brief: Release file_storage_.
     */
    void Release();

private:
    /* @brief: The member variable that stores yaml data 
     *         is either file_node_ or file_storage_.
     */
    const bool is_file_node_;
    cv::FileNode file_node_;
    cv::FileStorage file_storage_;

    /* @brief: Get content by key.
     */
    cv::FileNode get_(const std::string &key) const;
};

// Get a content of cv::FileNode. Convert to type T.
template <typename T>
T Yaml::get(const std::string &key) const
{
    cv::FileNode content = get_(key);
    return static_cast<T>(content);
}

// Get a content of cv::FileNode. Convert to type vector<T>.
template <typename T>
std::vector<T> Yaml::get_vec(const std::string &key) const
{
    cv::FileNode content = get_(key);
    std::vector<T> res;
    content >> res;
    return res;
}

} // namespace basics
} // namespace my_slam
#endif