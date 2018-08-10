### Overview

pcd_registration.cpp </br>
    http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration </br>
    与示例中的程序不同之处在于,该该程序根据pcd索引列表文件来依次读取pcd文件并进行配准

merge_pcd.cpp </br>
    简单的读取指定目录下的所有pcd文件并合并为一个pcd文件。该程序不包含配准操作。

### Building

    sudo make build 
    make

### Usage

    merge_pcd <source_dir> <output_file>
    pcd_registration <pcd_index_list_file>