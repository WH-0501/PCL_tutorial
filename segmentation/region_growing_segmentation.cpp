#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

//#define PI 3.14159

int
main (int argc, char** argv)
{
  std::cout << argc << std::endl;
  std::string pcd_file = "../test.pcd";
  if(argc == 2) 
  {
    pcd_file = argv[1];
  }
  std::cout << "Load pcd file: " << pcd_file << std::endl;
  // load PointCloud data 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> (pcd_file, *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }
  pcl::PCDWriter writer;
  
  // filter ground and the point that too high 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.4, 2.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  // compute normal 
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud_filtered);
  normal_estimator.setKSearch (50);
  //normal_estimator.setViewPoint(-1, -1, -1); // set viewpoint 
  normal_estimator.compute (*normals);
  
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  // set the max and min points size in a cluster  
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud_filtered);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  
  /* 该算法中最重要的参数 */
  // 设置邻近点与当前种子点的法线角度差
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI); // 0.052 弧度 - 2.9793806° ≈ 3°
  // 设置曲率阈值
  reg.setCurvatureThreshold (1);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  for (int i =0; i < clusters.size(); i++) {
    std::cout << i << "th cluster has " << clusters[i].indices.size () << " points."  << cloud_filtered->at(clusters[0].indices[i])
    << cloud_filtered->points[clusters[0].indices[i]] << endl;
  } 
  
  std::cout << "These are the indices of the points of the initial" <<
  std::endl << "cloud that belong to the first cluster:" << std::endl;
    
  int cnt = 0;
  // for remove method 2
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0, min_z = 0.0, max_z = 0.0;
  for (std::vector<pcl::PointIndices>::const_iterator clu_it = clusters.begin (); clu_it != clusters.end (); ++clu_it)
  {
    cnt++;
    double angle_sum = 0.0;
    std::cout << "================= " << cnt << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator idx_it = clu_it->indices.begin (); idx_it != clu_it->indices.end (); ++idx_it)
    {
      //std::cout << (*idx_it) << "Point - " << cloud_filtered->points[*idx_it] << " - " << normals->points[*idx_it] << std::endl;
      // print the max/min value of the normal 
      //max_x = cloud_filtered->points[*idx_it].x > max_x ? cloud_filtered->points[*idx_it].x:max_x;
      //min_x = cloud_filtered->points[*idx_it].x > min_x ? min_x:cloud_filtered->points[*idx_it].x;
      //max_y = cloud_filtered->points[*idx_it].y > max_y ? cloud_filtered->points[*idx_it].y:max_y;
      //min_y = cloud_filtered->points[*idx_it].y > min_y ? min_y:cloud_filtered->points[*idx_it].y;
      //max_z = cloud_filtered->points[*idx_it].z > max_z ? cloud_filtered->points[*idx_it].z:max_z;
      //min_z = cloud_filtered->points[*idx_it].z > min_z ? min_z:cloud_filtered->points[*idx_it].z;
      
      // Calculate the angle between normal and vector (0,0,1).
      double norm = sqrt(normals->points[*idx_it].normal_x * normals->points[*idx_it].normal_x
                   + normals->points[*idx_it].normal_y * normals->points[*idx_it].normal_y
                   + normals->points[*idx_it].normal_z * normals->points[*idx_it].normal_z);
      double angle = acos(normals->points[*idx_it].normal_z / norm) ; 
      if (angle > M_PI && angle <= M_PI)
      {
        angle = M_PI - angle;
      }
      angle_sum += angle; 
      std::cout << "Point: " << *idx_it << " Angle is: " << angle << std::endl;
      
      cloud_cluster->points.push_back (cloud_filtered->points[*idx_it]);
    }
    std::cout << "Average angle is : " << angle_sum / clu_it->indices.size() << std::endl;
    //std::cout << cnt << "th cluster, thresold: min( " << min_x << "," << min_y << "," << min_z << "); max( " 
    //  << max_x << "," << max_y << "," << max_z << ")" << std::endl;
    cloud_cluster->width = static_cast<uint32_t> (cloud_cluster->points.size ());
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    // write PCD file 
    std::stringstream ss;
    ss << "./cloud_cluster_" << cnt << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
    
    // remove the slope that below 15 degree
	if ((angle_sum / clu_it->indices.size()) <= 0.2617994) // angle / 180 * M_PI
    {
      std::cout << "Erase this cluster (" << clu_it->indices.size() << ")points" << std::endl;
      int erase_counter = 0;
      while (erase_counter < clu_it->indices.size()) {
        //std::cout << "===>" << clu_it->indices[erase_counter] << std::endl;
        //cloud->erase(cloud->begin() + clu_it->indices[erase_counter]);
        erase_counter++;
        
        // for remove method 2
        inliers->indices.push_back(clu_it->indices[erase_counter]);
      }
    }
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_new (new pcl::PointCloud<pcl::PointXYZ>);
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_filtered_new);
  // save point to PCD file without slope
  writer.write<pcl::PointXYZ> ("without_slope.pcd", *cloud_filtered_new, false); 
  
  /*
  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {  
    //std::cout << clusters[0].indices[counter] << ", ";
    std::cout << clusters[0].indices[counter] << "point -- " << normals->points[clusters[0].indices[counter]] << std::endl;
    counter++;
  }
  std::cout << std::endl;
  */
  
  /* remove point that doesn't need */  
  //int erase_counter = 0;
  //while (erase_counter < clusters[0].indices.size()) {
  //  cloud_filtered->erase(cloud_filtered->begin() + clusters[0].indices[erase_counter]);
  //  erase_counter++;
  //}
  
  std::cout << "PCL Viewer..." << std::endl;
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}
