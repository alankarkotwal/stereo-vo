/***********************************************************************************\
* Stereo Visual Odometry using registration with Gaussian mixtures					*
* Alankar Kotwal <alankar.kotwal@iitb.ac.in>, Anand Kalvit <anandiitb12@gmail.com>	*
* Visual Odometry node, estimates transformation between pointclouds				*
\***********************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

int main(int argc, char *argv[]) {

	if (argc < 3) {
		cerr << "Usage: " << argv[0] << " POINTCLOUD1 POINTCLOUD2\n";
		return -1;
	}
	
	PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);

	if (io::loadPCDFile<PointXYZRGB>(argv[1], *cloud1) == -1 || io::loadPCDFile<PointXYZRGB>(argv[2], *cloud2) == -1) {
		return (-1);
	}
	
	// Pointset matching here
	
	return 0;
	
}
