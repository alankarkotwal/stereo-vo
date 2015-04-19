/***********************************************************************************\
* Stereo Visual Odometry using registration with Gaussian mixtures					*
* Alankar Kotwal <alankar.kotwal@iitb.ac.in>, Anand Kalvit <anandiitb12@gmail.com>	*
* Visual Odometry node, estimates transformation between pointclouds				*
\***********************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#define PI 3.14159265
#define w 0.9
#define GAUSS_VAR 100
#define MAX_ITER 20

using namespace std;
using namespace pcl;
using namespace Eigen;

float GaussKernel(Vector3f, float);

int main(int argc, char *argv[]) {

	if (argc < 5) {
		cerr << "Usage: " << argv[0] << " POINTCLOUD_STATIC POINTCLOUD_MOVING OUTRFILE OUTTFILE OUTAFILE\n";
		return -1;
	}
	
	PointCloud<PointXYZRGB>::Ptr s(new PointCloud<PointXYZRGB>); // Reference cloud
	PointCloud<PointXYZRGB>::Ptr m(new PointCloud<PointXYZRGB>); // Template cloud

	if (io::loadPCDFile<PointXYZRGB>(argv[1], *s) == -1 || io::loadPCDFile<PointXYZRGB>(argv[2], *m) == -1) {
		return (-1);
	}
	
	const int N = s -> size();
	const int M = m -> size();
	
	// Pointset matching here
	// Initiate rotation quaternion
	/*float ax, ay;
	Quaternion<float> q;  q = AngleAxis<float>(angle_in_radian, Vector3f(ax,ay,az));
	Vector3f t;
	t << 0, 0, 0;
	float a = 1;
	
	float ObjFn = 0;
	
	for(int iter=0; iter<MAX_ITER; iter++) {
	
		for(int i=0; i<M; i++) {
			
			// Find the rotated, translated Vector3f
			
			for(int j=0; j<N; j++) {
			
				Vector3f diff;
				diff << s->points[j].x, s->points[j].y, s->points[j].z;
				diff = mTrans - diff;
				ObjFn = ObjFn + GaussKernel(diff, GAUSS_VAR);
				
			}
			
		}
		
	}
	
	// Write transformation to file here
	/*ofstream Rfile;
	ofstream Tfile;
	ofstream afile;
	Rfile.open(argv[3]);
	Tfile.open(argv[4]);
	afile.open(argv[5]);
	
	stringstream Rstream, Tstream, astream;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			Rstream << R(i, j) << endl;
		}
	}
	for(int j=0; j<3; j++) {
			Tstream << t(j) << endl;
	}
	astream << a;
	
	Rfile.write(Rstream.str().c_str(), Rstream.str().length());
	Tfile.write(Tstream.str().c_str(), Tstream.str().length());
	afile.write(astream.str().c_str(), astream.str().length());
	Rfile.close();
	Tfile.close();
	afile.close();
	return 0;*/
	
}
