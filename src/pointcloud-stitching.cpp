/***********************************************************************************\
* Stereo Visual Odometry using registration with Gaussian mixtures					*
* Alankar Kotwal <alankar.kotwal@iitb.ac.in>, Anand Kalvit <anandiitb12@gmail.com>	*
* Pointcloud stitching using the transformation estimated by pointset-matching		*
\***********************************************************************************/

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {

	if (argc < 4) {
		cerr << "Usage: " << argv[0] << " JSON_DATA_FILE JSON_LEFT_CALIB_FILE JSON_RIGHT_CALIB_FILE\n";
		return -1;
	}
	
}
