#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

struct MyPointType
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float normal_x;
  float normal_y;
  float normal_z;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, normal_x, normal_x)
                                   (float, normal_y, normal_y)
                                   (float, normal_z, normal_z)
)


int main (int argc, char** argv)
{  
  std::string in_path;
  std::string out_name;
  if (argv[1]){
    in_path = argv[1];
    std::cout << "Input PCD path = " << in_path << std::endl;
  }
  else {
    std::cout << "---------- Insert an input path -------------"  << std::endl;
    return(0);
  }

  if (argv[2]){
    out_name = argv[2];
    std::cout << "Output PCD file name = " << out_name << std::endl;
  }
  else {
    std::cout << "---------- No output path inserted, it will be stored as out.pcd -------------"  << std::endl;
    out_name = "out.pcd";  
  }

  pcl::PointCloud<MyPointType>::Ptr cloud_with_normals (new pcl::PointCloud<MyPointType>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  pcl::io::loadPCDFile (in_path, *cloud);

  // estimate normals
    // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  cloud_with_normals->resize(cloud->size());
  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);
  // Compute the features
  ne.compute (*normals);

  // Other solution to compute normals
  /*pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  ne.compute(*normals);*/

  std::cout << "Points: " << cloud->size() << "  Normals" << normals->size()<< std::endl;
    
  for (int i = 0; i < normals->size(); ++i){    
      cloud_with_normals->points[i].x = cloud->points[i].x;
      cloud_with_normals->points[i].y = cloud->points[i].y;
      cloud_with_normals->points[i].z = cloud->points[i].z;
      cloud_with_normals->points[i].normal_x = normals->points[i].normal_x;
      cloud_with_normals->points[i].normal_y = normals->points[i].normal_y;
      cloud_with_normals->points[i].normal_z = normals->points[i].normal_z;        
  }

  std::cout << "Saving ... " << std::endl;
  pcl::io::savePCDFileASCII (out_name, *cloud_with_normals);
  std::cout << "Saved into " << out_name << std::endl;
  return (0);
}


