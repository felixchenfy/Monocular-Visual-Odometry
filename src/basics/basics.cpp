

#include "my_slam/basics/basics.h"
#include <iterator>
#include <assert.h>
#include <chrono>
#include <ctime>
#include <sys/types.h> //mkdir
#include <sys/stat.h>  //mkdir
#include <unistd.h> // access
#include <stdlib.h> // access

namespace my_slam
{
namespace basics
{
bool str2bool(const string &s)
{
    if (s == "true" || s == "True")
    {
        return true;
    }
    else if (s == "false" || s == "False")
    {
        return false;
    }
    else
    {
        cout << "my Warning: wrong string to convert to bool" << endl;
        assert(0);
    }
}

string int2str(int num, int width, char char_to_fill)
{
    std::stringstream ss;
    ss << std::setw(width) << std::setfill(char_to_fill) << num;
    return ss.str();
}

vector<double> str2vecdouble(const string &pointLine)
{
    std::istringstream iss(pointLine);

    return vector<double>{
        std::istream_iterator<double>(iss),
        std::istream_iterator<double>()};
}

vector<int> getIntersection(vector<int> v1, vector<int> v2)
{
    int comb_len = v1.size() + v2.size();
    vector<int> v(comb_len);
    vector<int>::iterator it;
    it = std::set_intersection(
        v1.begin(), v1.end(),
        v2.begin(), v2.end(),
        v.begin());
    v.resize(it - v.begin());
    return v;
}

bool makedirs(const string &dir)
{
    const char *sPathName = dir.c_str();
    char DirName[256];
    strcpy(DirName, sPathName);
    int i, len = strlen(DirName);
    if (DirName[len - 1] != '/')
        strcat(DirName, "/");

    len = strlen(DirName);

    for (i = 1; i < len; i++)
    {
        if (DirName[i] == '/')
        {
            DirName[i] = 0;
            if (access(DirName, F_OK) != 0)
            {
                if (mkdir(DirName, 0755) == -1)
                {
                    perror("Error makedirs.");
                    return false;
                }
            }
            DirName[i] = '/';
        }
    }
    return true;
}


} // namespace basics
} // namespace my_slam