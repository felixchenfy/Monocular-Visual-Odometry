

#include "my_basics/basics.h"
#include <string>
#include <vector>
#include <iterator>
#include <sstream>
#include <assert.h>

namespace my_basics
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
    stringstream ss;
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

} // namespace my_basics