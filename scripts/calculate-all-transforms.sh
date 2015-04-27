#!/bin/bash

#####################################################################################
# Stereo Visual Odometry using registration with Gaussian mixtures					#
# Alankar Kotwal <alankar.kotwal@iitb.ac.in>, Anand Kalvit <anandiitb12@gmail.com>	#
# Bash file to generate all transforms using standard test data						#
#####################################################################################

# Identity
./pointset-matching ../pointclouds/sparse/pair00.pcd ../pointclouds/sparse/pair00.pcd ../data/tf00R.transform ../data/tf00t.transform
./pointset-matching ../pointclouds/sparse/pair11.pcd ../pointclouds/sparse/pair11.pcd ../data/tf11R.transform ../data/tf11t.transform
./pointset-matching ../pointclouds/sparse/pair22.pcd ../pointclouds/sparse/pair22.pcd ../data/tf22R.transform ../data/tf22t.transform
./pointset-matching ../pointclouds/sparse/pair33.pcd ../pointclouds/sparse/pair33.pcd ../data/tf33R.transform ../data/tf33t.transform
./pointset-matching ../pointclouds/sparse/pair44.pcd ../pointclouds/sparse/pair44.pcd ../data/tf44R.transform ../data/tf44t.transform

# Pairwise
./pointset-matching ../pointclouds/sparse/pair00.pcd ../pointclouds/sparse/pair01.pcd ../data/tf01R.transform ../data/tf01t.transform
./pointset-matching ../pointclouds/sparse/pair01.pcd ../pointclouds/sparse/pair02.pcd ../data/tf12R.transform ../data/tf12t.transform
./pointset-matching ../pointclouds/sparse/pair02.pcd ../pointclouds/sparse/pair03.pcd ../data/tf23R.transform ../data/tf23t.transform
./pointset-matching ../pointclouds/sparse/pair03.pcd ../pointclouds/sparse/pair04.pcd ../data/tf34R.transform ../data/tf34t.transform
