#include <iostream>
#include <my_common/config.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string filename = "config/default.yaml";
    int level_pyramid;
    string key = "level_pyramid";

    // test 1
    FileStorage fs(filename, FileStorage::READ);
    if (fs.isOpened() == false)
    {
        std::cerr << "Parameter file " << filename << " does not exist." << std::endl;
        return 1;
    }
    level_pyramid = fs[key];
    cout << "Read yaml by this cpp: level_pyrimid = " << level_pyramid << endl;

    // test 2
    my::Config::setParameterFile(filename);
    level_pyramid = my::Config::get<int>(key);
    cout << "Read yaml by config.h: level_pyrimid = " << level_pyramid << endl;

    return 1;
}