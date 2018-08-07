/*************************************************************************
	File Name: merge_pcd.cpp
	Author: Wang Bang
	Mail: wbang8933@163.com
	Date: 2018-08-06
    Description: read and combine all pcd files in the specified dir 
                 to a pcd file without registration
 ************************************************************************/
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

using namespace std;

int main(int argc, char **argv)
{
  if ( argc < 3 ) 
  {
    PCL_ERROR("Usage: xxx dir output_name\n");
    return -1;
  }
  
  char *dirname = argv[1];
  char *outfile = argv[2];
  
  DIR *dir;
  struct dirent* file_info;
  pcl::PointCloud<pcl::PointXYZI> cloud, final_result;
  char pcd_name[1024];
  
  if ( NULL == dirname )
  {
    PCL_ERROR("Invalied directory name\n");
    return false;
  }
  
  struct stat st;
  lstat( dirname, &st );
  if ( !S_ISDIR( st.st_mode ))
  {
    PCL_ERROR("%s is not a vaild directory\n", dirname);
    return false;
  }
  
  dir = opendir( dirname );
  if ( NULL == dir )
  {
    PCL_ERROR("Open %s failed!\n", dirname);
    return false;
  }
  
  PCL_INFO( "Start read pcd...\n" );
  
  while( (file_info = readdir(dir)) != NULL )
  {
    PCL_INFO( "===>:%s\n",file_info->d_name );
    if ( NULL == file_info->d_name || strlen(file_info->d_name) == 0)
    {
      continue;
    }
    PCL_INFO( "%s\n", file_info->d_name ); 
    if ( strcmp( file_info->d_name, "." ) == 0 ||
        strcmp( file_info->d_name, ".." ) == 0 )
    {
        continue;
    }
    PCL_INFO( "file name:%s\n", file_info->d_name );
    // get the suffix name
    char *ext = strrchr( file_info->d_name, '.' );
    if ( NULL == ext )
    {
      PCL_ERROR( "Cannot get extend name!\n" );
      continue;
    }
    PCL_INFO("EXT name:%s\n", ext);
    if ( strcmp( ext, ".pcd" ) )
    {
      PCL_ERROR( " This is not a pcd file!" );
      continue;
    }
    sprintf(pcd_name, "%s%s", dirname, file_info->d_name);
    pcl::io::loadPCDFile<pcl::PointXYZI>( pcd_name, cloud );
    PCL_INFO( "Point cloud loaded, point size= %d\n", cloud.points.size() );
    
    final_result += cloud;
    memset(pcd_name, 0, sizeof(pcd_name));
  }
  
  pcl::io::savePCDFileASCII ( outfile, final_result);
  closedir(dir);

  return 0;
}