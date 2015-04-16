/***********************************************************************************\
* Stereo Visual Odometry using registration with Gaussian mixtures					*
* Alankar Kotwal <alankar.kotwal@iitb.ac.in>, Anand Kalvit <anandiitb12@gmail.com>	*
* Convert stereo images and stereo calibration data into pointclouds				*
\***********************************************************************************/

#include <cstdlib>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;
using namespace pcl;

int main(int argc, char *argv[]) {

	if (argc < 3) {
		cerr << "Usage: " << argv[0] << " FILE_LIST Q_FILE\n";
		return -1;
	}
	
	std::vector<Mat> left_images, right_images;
	ifstream file;
	file.open(argv[1]);
	
	string line;
	while(getline(file, line)) {
		left_images.push_back(imread("../images/"+line+"_left.jpg"));
		right_images.push_back(imread("../images/"+line+"_right.jpg"));
	}
	
	file.close();
	file.open(argv[2]);	
	
	Mat Q = Mat::zeros(4, 4, CV_32F);
	for(int i=0; i<4; i++) {
		for(int j=0; j<4; j++) {
			getline(file, line);
			Q.at<float>(i, j) = stof(line);
		}
	}
	
	StereoSGBM sgbm;
	sgbm.SADWindowSize = 5;
	sgbm.numberOfDisparities = 192;
	sgbm.preFilterCap = 4;
	sgbm.minDisparity = -64;
	sgbm.uniquenessRatio = 1;
	sgbm.speckleWindowSize = 150;
	sgbm.speckleRange = 2;
	sgbm.disp12MaxDiff = 10;
	sgbm.fullDP = true;
	sgbm.P1 = 600;
	sgbm.P2 = 2400;
	
	pcl::PCDWriter w;
	
	for(int i=0; i<left_images.size(); i++) {
	
		Mat disparities;
		Mat reprojections;
		PointCloud<PointXYZRGB> pc;
		Mat grayLeft, grayRight;
		
		cvtColor(left_images[i], grayLeft, CV_BGR2GRAY);
		cvtColor(right_images[i], grayRight, CV_BGR2GRAY);
		
		sgbm(grayLeft, grayRight, disparities);
		reprojectImageTo3D(disparities, reprojections, Q, true, -1);
		
		for (int k=0; k < left_images[i].rows; k++) {
			for (int j=0; j < left_images[i].cols; j++) {
				PointXYZRGB p;
				p.z = reprojections.at<Vec3f>(k, j)[2];
				if(p.z < 400 && p.z > 0) {
					p.x = reprojections.at<Vec3f>(k, j)[0];
					p.y = reprojections.at<Vec3f>(k, j)[1];
					cv::Vec3b bgr(left_images[i].at<cv::Vec3b>(k, j));
					p.b = bgr[0];
					p.g = bgr[1];
					p.r = bgr[2];
					pc.push_back(p);
				}
			}
		}
		
		stringstream filename;
		filename << "../pointclouds/pair0" <<i<< ".pcd";
		w.writeASCII(filename.str(), pc);
	}
	
}
