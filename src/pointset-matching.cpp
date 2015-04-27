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

#define PI 3.14159265
#define GAUSS_VAR 100
#define MAX_ITER 3
#define MAX_OBJ_FN 1

using namespace std;
using namespace pcl;
using namespace Eigen;

int main(int argc, char *argv[]) {

	if (argc < 5) {
		cerr << "Usage: " << argv[0] << " POINTCLOUD_STATIC POINTCLOUD_MOVING OUTRFILE OUTTFILE\n";
		return -1;
	}
	
	PointCloud<PointXYZRGB>::Ptr s(new PointCloud<PointXYZRGB>); // Reference cloud
	PointCloud<PointXYZRGB>::Ptr m(new PointCloud<PointXYZRGB>); // Moving cloud

	if (io::loadPCDFile<PointXYZRGB>(argv[1], *s) == -1 || io::loadPCDFile<PointXYZRGB>(argv[2], *m) == -1) {
		return (-1);
	}
	
	const int N = s -> size();
	const int M = m -> size();
	
	MatrixXf MPoints(M, 3);
	for(int i=0; i<M; i++) {
		MPoints(i, 0) = m->points[i].x;
		MPoints(i, 1) = m->points[i].y;
		MPoints(i, 2) = m->points[i].z;
	}
	
	// Pointset matching here
	Vector4f rot(1, 0, 0, 0);
	Vector3f trans(0, 0, 0);
	
	float objFn = 1000000000000;
	MatrixXf G = MatrixXf::Zero(M, 3); // Initialise zeros
	
	VectorXf OneM(M);
	OneM.setOnes();
	
	VectorXf OneD(3);
	OneD.setOnes();
	
	float stepSize = 0.01;
	int iter = 0;
	
	while(iter < MAX_ITER && objFn > MAX_OBJ_FN) { // Termination criterion
	
		// Calculate the G matrix and evaluate the objective function
		objFn = 0;
		G = MatrixXf::Zero(M, 3);
		Quaternionf rotQ(rot(0), rot(1), rot(2), rot(3));
		Matrix3f rotM = rotQ.toRotationMatrix();
		
		for(int i=0; i<M; i++) {
		
			for(int j=0; j<3; j++) {

				for(int n=0; n<M; n++) {
				
					// Calculate <i-th transformed vector from the moving set>-<n-th vector from the stationary set>
					Vector3f diff;
					diff << m->points[i].x, m->points[i].y, m->points[i].z;
					diff = rotQ * diff + trans;
					
					diff << diff(0)-s->points[n].x, diff(1)-s->points[n].y, diff(2)-s->points[n].z;
					
					float pref = ((exp(-(pow(diff(0), 2) + pow(diff(1), 2) + pow(diff(2), 2))/(2*GAUSS_VAR)))/(GAUSS_VAR));
					G(i, j) += pref * (diff(0)*rotM(0, j) + diff(1)*rotM(1, j) + diff(2)*rotM(2, j));
					objFn += pref;
				
				}
				
			}
		
		}
		cout << objFn << endl;
		
		// Calculate gradient wrt trans
		Vector3f transGrad = G.transpose()*OneM;
		
		// Calculate gradients wrt rot
		Matrix3f temp = G.transpose()*MPoints;
		Matrix3f temp1, temp2, temp3, temp4;
		temp1 <<  2*rot(0)*temp(0, 0), -2*rot(3)*temp(0, 1),  2*rot(2)*temp(0, 2), 
				  2*rot(3)*temp(1, 0),  2*rot(0)*temp(1, 1), -2*rot(1)*temp(1, 2),
				 -2*rot(2)*temp(2, 0),  2*rot(1)*temp(2, 1),  2*rot(0)*temp(2, 2);
				 
		temp2 <<  2*rot(1)*temp(0, 0),  2*rot(2)*temp(0, 1),  2*rot(3)*temp(0, 2), 
				  2*rot(2)*temp(1, 0), -2*rot(1)*temp(1, 1), -2*rot(0)*temp(1, 2),
				  2*rot(3)*temp(2, 0),  2*rot(0)*temp(2, 1), -2*rot(1)*temp(2, 2);
				 
		temp3 << -2*rot(2)*temp(0, 0),  2*rot(1)*temp(0, 1),  2*rot(0)*temp(0, 2), 
				  2*rot(1)*temp(1, 0),  2*rot(2)*temp(1, 1),  2*rot(3)*temp(1, 2),
				 -2*rot(0)*temp(2, 0),  2*rot(3)*temp(2, 1), -2*rot(2)*temp(2, 2);
				 
		temp4 << -2*rot(3)*temp(0, 0), -2*rot(0)*temp(0, 1),  2*rot(1)*temp(0, 2), 
				  2*rot(0)*temp(1, 0), -2*rot(3)*temp(1, 1),  2*rot(2)*temp(1, 2),
				  2*rot(1)*temp(2, 0),  2*rot(2)*temp(2, 1),  2*rot(3)*temp(2, 2);
		Vector4f rotGrad;
		rotGrad << OneD.transpose()*temp1*OneD, OneD.transpose()*temp2*OneD, OneD.transpose()*temp3*OneD, OneD.transpose()*temp4*OneD;
		
		// Perform the update
		trans = trans - stepSize*transGrad;
		rot = rot - stepSize*rotGrad;
		rot = rot/(sqrt(pow(rot(0), 2) + pow(rot(1), 2) + pow(rot(2), 2) + pow(rot(3), 2)));
		
		iter = iter + 1;
	
	}
	
	// Write transformation to file here
	Quaternionf rotQ(rot(0), rot(1), rot(2), rot(3));
	Matrix3f rotM = rotQ.toRotationMatrix();
	
	ofstream Rfile;
	ofstream Tfile;
	Rfile.open(argv[3]);
	Tfile.open(argv[4]);
	
	stringstream Rstream, Tstream, astream;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			Rstream << rotM(j, i) << endl;
		}
	}
	for(int j=0; j<3; j++) {
			Tstream << trans(j) << endl;
	}
	
	Rfile.write(Rstream.str().c_str(), Rstream.str().length());
	Tfile.write(Tstream.str().c_str(), Tstream.str().length());
	Rfile.close();
	Tfile.close();
	
	cout << "Done!" << endl;

	return 0;
	
}
