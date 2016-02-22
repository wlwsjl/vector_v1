
#include "range_sensor_filters/point_cloud_footprint_filter.h"
#include "range_sensor_filters/laser_scan_footprint_filter.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "filters/filter_base.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(range_sensor_filters, LaserScanFootprintFilter, range_sensor_filters::LaserScanFootprintFilter, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_DECLARE_CLASS(range_sensor_filters, PointCloudFootprintFilter, range_sensor_filters::PointCloudFootprintFilter, filters::FilterBase<sensor_msgs::PointCloud2>)
