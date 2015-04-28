#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

// This is the main function
int main (int argc, char** argv)
{

	// Load file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	pcl::io::loadPCDFile (argv[1], *source_cloud);

	Eigen::Affine3f transform = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform.translation() << 2.5, 0.0, 0.0;

	// The same rotation matrix as before; tetha radians arround Z axis
	transform.rotate (Eigen::AngleAxisf (0.1, Eigen::Vector3f::UnitZ()));

	// Print the transformation
	std::cout << transform.matrix() << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
	// You can either apply transform_1 or transform; they are the same
	pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

	pcl::PCDWriter w;
	std::stringstream filename;
	filename << "../pointclouds/transformed/transformed.pcd";
	w.writeASCII(filename.str(), *transformed_cloud);

	return 0;
}
