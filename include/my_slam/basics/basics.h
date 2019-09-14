/* @brief Some basic common functions.
 */

#ifndef MY_SLAM_BASICS_H
#define MY_SLAM_BASICS_H

#include "my_slam/common_include.h"

namespace my_slam
{
namespace basics
{

// Convert a string of "true" or "false" to bool.
bool str2bool(const string &s);

// Convert int to string, and fill it with zero before the number to make it specified width
string int2str(int num, int width, char char_to_fill = '0');

// Convert string into a vector of doubles
vector<double> str2vecdouble(const string &pointLine);

// Please sort v1 and v2 first, then this functions returns the intersection of two vector.
vector<int> getIntersection(vector<int> v1, vector<int> v2);

} // namespace basics
} // namespace my_slam

#endif