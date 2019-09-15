#include "my_slam/basics/yaml.h"

namespace my_slam
{
namespace basics
{

/* @brief: Constructor: constructed by reading a .yaml file.
 */
Yaml::Yaml(const std::string &filename) : file_storage_(filename, cv::FileStorage::READ),
                                          is_file_node_(false)
{
    if (!file_storage_.isOpened())
    {
        std::cout << "Error reading config file: " << filename << std::endl;
        exit(EXIT_FAILURE);
    }
}

/* @brief: Constructor: constructed from existing cv::FileNode.
 */
Yaml::Yaml(const cv::FileNode &fn) : file_node_(fn),
                                     is_file_node_(true)
{
}

/* @brief: Constructor: constructed from existing cv::FileStorage.
 */
Yaml::Yaml(const cv::FileStorage &fs) : file_storage_(fs),
                                        is_file_node_(false)
{
}

/* @brief: Release file_storage_.
 */
void Yaml::Release()
{
    if (!is_file_node_ && file_storage_.isOpened())
        file_storage_.release();
}

/* @brief: Release file memory.
 */
Yaml::~Yaml()
{
    this->Release();
}

// Get a content of cv::FileNode. Convert to Yaml type.
Yaml Yaml::get(const std::string &key) const
{
    cv::FileNode content = get_(key);
    return Yaml(content);
}

/* @brief: Get content by key.
 */
cv::FileNode Yaml::get_(const std::string &key) const
{
    cv::FileNode content;
    if (is_file_node_)
        content = file_node_[key];
    else
        content = file_storage_[key];
    if (content.empty())
    {
        std::cout << "Error: key '" << key << "' doesn't exist" << std::endl;
        exit(EXIT_FAILURE);
    }
    return content;
}

} // namespace basics
} // namespace my_slam