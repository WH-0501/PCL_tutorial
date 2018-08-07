### Overview

velodyne_convert.cpp </br>
    参考 https://github.com/bostondiditeam/kitti </br>
    将从 kitti 下载的 [velodyne 二进制数据(*.bin)](https://s3.eu-central-1.amazonaws.com/avg-kitti/data_road_velodyne.zip)转换为 pcd 文件 </br>


### categories

  uu  - urban unmarked </br>
  um  - urban marked </br>
  umm - urban multiple marked lanes </br>


### Building

    sudo make build 
    make


### Usage

    ./velodyne_convert <velodyne_bin_dir>