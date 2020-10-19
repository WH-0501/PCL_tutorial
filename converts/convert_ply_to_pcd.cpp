#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

using namespace pcl;
using namespace pcl::io;

int main (int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "usage: xxx <input> <output>" << std::endl;
        exit(-1);
    }
    std::string input = argv[1];
    std::string output = argv[2];

    pcl::PLYReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read<pcl::PointXYZ>(input, *cloud);
    pcl::io::savePCDFile(output, *cloud );

    return 0;
}
