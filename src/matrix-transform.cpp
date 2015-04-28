#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

// This function displays the help
void
showHelp(char * program_name)
{
	std::cout << std::endl;
	std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
	std::cout << "-h:	Show this help." << std::endl;
}

// This is the main function
int
main (int argc, char** argv)
{

	/*// Load file | Works with PCD and PLY files
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

	if (file_is_pcd) {
		if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)	{
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp (argv[0]);
			return -1;
		}
	} else {
		if (pcl::io::loadPLYFile (argv[filenames[0]], *source_cloud) < 0)	{
			std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
			showHelp (argv[0]);
			return -1;
		}
	}

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
	float theta = M_PI/4; // The angle of rotation in radians
	transform_1 (0,0) = cos (theta);
	transform_1 (0,1) = -sin(theta);
	transform_1 (1,0) = sin (theta);
	transform_1 (1,1) = cos (theta);
	//		(row, column)

	// Define a translation of 2.5 meters on the x axis.
	transform_1 (0,3) = 2.5;

	// Print the transformation
	printf ("Method #1: using a Matrix4f\n");
	std::cout << transform_1 << std::endl;

	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// Define a translation of 2.5 meters on the x axis.
	transform_2.translation() << 2.5, 0.0, 0.0;

	// The same rotation matrix as before; tetha radians arround Z axis
	transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

	// Print the transformation
	printf ("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform_2);

	pcl::PCDWriter w;
	std::stringstream filename;
	filename << "pointclouds/aligned.pcd";
	std::cout << "Saving a pointcloud to " << filename.str() << "...\n";
	w.writeASCII(filename.str(), aligned);*/

	return 0;
}
