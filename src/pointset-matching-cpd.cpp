/***********************************************************************************\
* Stereo Visual Odometry using registration with Gaussian mixtures					*
* Alankar Kotwal <alankar.kotwal@iitb.ac.in>, Anand Kalvit <anandiitb12@gmail.com>	*
* Visual Odometry node, estimates transformation between pointclouds				*
\***********************************************************************************/

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#define PI 3.14159265
#define w 0.9
#define MAX_ITER 20

using namespace std;
using namespace pcl;
using namespace Eigen;

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
	
	MatrixXf SPoints(N, 3);
	MatrixXf MPoints(M, 3);
	VectorXf OneM(M);
	VectorXf OneN(N);
	
	for(int i=0; i<N; i++) {
		SPoints(i, 0) = s->points[i].x;
		SPoints(i, 1) = s->points[i].y;
		SPoints(i, 2) = s->points[i].z;
		OneN(i) = 1;
	}
	for(int i=0; i<M; i++) {
		MPoints(i, 0) = m->points[i].x;
		MPoints(i, 1) = m->points[i].y;
		MPoints(i, 2) = m->points[i].z;
		OneM(i) = 1;
	}
	
	// Pointset matching here
	Matrix3f R = Matrix3f::Identity();
	Vector3f t;
	t << 0, 0, 0;
	float a = 1;
	
	float sigSq = 0;
	for(int i=0; i<M; i++) {
		for(int j=0; j<N; j++) {
		
			sigSq += pow((s->points[j].x - m->points[i].x), 2) + pow((s->points[j].y - m->points[i].y), 2) + pow((s->points[j].z - m->points[i].z), 2);
		
		}
	}
	sigSq = sigSq/(3*N*M);
	
	for(int iter=0; iter<MAX_ITER; iter++) { // Term criterion?
		
		cout << "Iteration number " << iter+1 << endl;
		
		float denTerm2 = pow(2*PI*sigSq, 1.5)*w*M/((1-w)*N);
		
		MatrixXf P(M, N);
		for(int j=0; j<N; j++) {
		
			float denTerm1 = 0;
			for(int i=0; i<M; i++) {
			
				Vector3f mTrans;
				mTrans << m->points[i].x, m->points[i].y, m->points[i].z;
				mTrans = R*mTrans + t;
				P(i, j) = exp(-(pow((s->points[j].x - mTrans(0)), 2) + pow((s->points[j].y - mTrans(1)), 2) + pow((s->points[j].z - mTrans(2)), 2))/(2*sigSq));
				denTerm1 += P(i, j);
			
			}
			for(int i=0; i<M; i++) {
			
				P(i, j) = P(i, j)/(denTerm1 + denTerm2);
			
			}
			
		}
		
		// Solver with the current parameters
		
		float Np = OneM.transpose()*P*OneN;
		Vector3f MuS;
		Vector3f MuM;		
	 	MuS = (SPoints.transpose()*P.transpose()*OneM)/Np;
		MuM = (MPoints.transpose()*P*OneN)/Np;
		
		MatrixXf SCap(N, 3);
		MatrixXf MCap(N, 3);
		SCap = SPoints - OneN*MuS.transpose();
		MCap = MPoints - OneM*MuM.transpose();
		
		Matrix3f A = SCap.transpose()*P.transpose()*MCap;
		
		JacobiSVD<Matrix3f> svd(A, ComputeFullU | ComputeFullV);
		Matrix3f UVt = svd.matrixU()*svd.matrixV().transpose();
		Matrix3f C;
		C << 1, 0, 0, 0, 1, 0, 0, 0, UVt.determinant();
		R = svd.matrixU()*C*svd.matrixV().transpose();
		
		VectorXf POne(M);
		VectorXf PtOne(N);
		POne = P*OneN;
		PtOne = P.transpose()*OneM;
		
		MatrixXf PDiag = MatrixXf::Identity(M, M);
		MatrixXf PtDiag = MatrixXf::Identity(N, N);
		for(int i=0; i<M; i++) {
			PDiag(i, i) = POne(i);
		}
		for(int i=0; i<N; i++) {
			PtDiag(i, i) = PtOne(i);
		}
		a = ((A.transpose()*R).trace())/((MCap.transpose()*PDiag*MCap).trace());
		t = MuS - a*R*MuM;
		sigSq = (1/(Np*3))*(SCap.transpose()*PtDiag*SCap).trace() - a*(A.transpose()*R).trace();
	
	}
	
	// Write transformation to file here
	ofstream Rfile;
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
	return 0;
	
}
