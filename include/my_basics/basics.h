
/*This is script is currently not used*/

#include <vector>
#include <algorithm>
using namespace std;

// Please sort v1 and v2 first, then this functions returns the intersection of two vector.
vector<int> getIntersection(vector<int> v1, vector<int> v2)
{
    int comb_len = v1.size() + v2.size();
    std::vector<int> v(comb_len);
    std::vector<int>::iterator it;
    it = std::set_intersection(
        v1.begin(), v1.end(),
        v2.begin(), v2.end(),
        v.begin());
    v.resize(it - v.begin());
    return v;
}