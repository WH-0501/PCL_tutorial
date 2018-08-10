#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h> // ģ�Ͷ���
#include <pcl/segmentation/sac_segmentation.h> // ���ڲ���һ���Էָ����ͷ�ļ�
#include <pcl/visualization/pcl_visualizer.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer viewer ("plane model segmentation example");
  
  /* 1. ���ɵ������� */
  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0; // �߶Ⱦ�Ϊ 1.0������ͬһƽ��
  }

  // Set a few outliers ���ü�������ƽ���ϵĵ�
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
                        << cloud->points[i].y << " "
                        << cloud->points[i].z << std::endl;
  // �����ָ�ʱ����Ҫ��ģ��ϵ������coefficients���洢�ڵ�ĵ��������϶���inliers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE); // ����ģ������Ϊ plane
  seg.setMethodType (pcl::SAC_RANSAC); // �����������һ���Է���
  /* ���þ��뷧ֵ�����뷧ֵ�����˵㱻��Ϊ�Ǿ��ڵ�ʱ�������������
   * ��ʾ�㵽����ģ�͵ľ������ֵ
   */
  seg.setDistanceThreshold (0.01); 
  
  seg.setInputCloud (cloud);
  // �ָ���̣��洢�ָ������㼸��inliers���洢ƽ��ģ�͵�ϵ��coefficients
  seg.segment (*inliers, *coefficients); 

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                               << cloud->points[inliers->indices[i]].y << " "
                                               << cloud->points[inliers->indices[i]].z << std::endl;

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 255, 255);
  viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
  
  return (0);
}