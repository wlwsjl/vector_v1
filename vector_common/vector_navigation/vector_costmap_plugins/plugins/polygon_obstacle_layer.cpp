

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <costmap_2d/costmap_2d.h>
#include <boost/foreach.hpp>


#include <vector_costmap_plugins/polygon_obstacle_layer.h>
#include <vector_costmap_plugins/geometry_tools.h>

PLUGINLIB_EXPORT_CLASS(vector_costmap_plugins::PolygonObstacleLayer, costmap_2d::Layer)

namespace vector_costmap_plugins
{

    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;
    using costmap_2d::FREE_SPACE;

    PolygonObstacleLayer::PolygonObstacleLayer() : dsrv_(NULL) {}

    PolygonObstacleLayer::~PolygonObstacleLayer(){
        if (dsrv_){
            delete dsrv_;
        }
    }
    
    void PolygonObstacleLayer::onInitialize(){
        ros::NodeHandle nh_("~/" + name_), g_nh_;
        current_ = true;
        marked_ = false;
        configured_ = 0;
        has_updated_data_ = false;
        
        boundary_subscriber_ = g_nh_.subscribe("vector/costmap_layers/bounding_box_polygon", 10, &PolygonObstacleLayer::updateBoundaryPolygon,this);
        obstacles_subscriber_ = g_nh_.subscribe("vector/costmap_layers/obstacle_polygons", 10, &PolygonObstacleLayer::updateObstaclePolygons,this);
        
        ros::Rate r(10);
        while ((0x3 != configured_) && (g_nh_.ok()))
        {
            ros::spinOnce();
            r.sleep();
        }
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh_);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                    &PolygonObstacleLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void PolygonObstacleLayer::matchSize(){
    
        if (!layered_costmap_->isRolling())
        {
            Costmap2D* master = layered_costmap_->getCostmap();
            resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
                      master->getOriginX(), master->getOriginY());
        }
    }


    void PolygonObstacleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level){
        enabled_ = config.enabled;
    }
    

    void PolygonObstacleLayer::updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped){

        ROS_INFO_STREAM("Boundary Polygon Recieved");
        
        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_stamped.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_stamped.header.frame_id);
            return;
        }
        
        //clear existing boundary, if any
        boundary_polygon_.points.clear();

        //Transform all points of boundary polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        geometry_msgs::Point32 tmp_pnt;
        in.header = polygon_stamped.header;
        BOOST_FOREACH(geometry_msgs::Point32 point, polygon_stamped.polygon.points){
            in.point.x = point.x;
            in.point.y = point.y;
            in.point.z = point.z;
            tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
            tmp_pnt.x = out.point.x;
            tmp_pnt.y = out.point.y;
            tmp_pnt.z = out.point.z;
            boundary_polygon_.points.push_back(tmp_pnt);
        }

        //if empty boundary provided, set to whole map
        if(boundary_polygon_.points.empty()){
            geometry_msgs::Point32 temp;
            temp.x = getOriginX();
            temp.y = getOriginY();
            boundary_polygon_.points.push_back(temp);
            temp.y = getSizeInMetersY();
            boundary_polygon_.points.push_back(temp);
            temp.x = getSizeInMetersX();
            boundary_polygon_.points.push_back(temp);
            temp.y = getOriginY();
            boundary_polygon_.points.push_back(temp);
        }

        updateOrigin(0,0);
        

        //Find map size and origin by finding min/max points of polygon
        double min_x = std::numeric_limits<double>::infinity();
        double min_y = std::numeric_limits<double>::infinity();
        double max_x = -std::numeric_limits<double>::infinity();
        double max_y = -std::numeric_limits<double>::infinity();

        BOOST_FOREACH(geometry_msgs::Point32 point, boundary_polygon_.points){
            min_x = std::min(min_x,(double)point.x);
            min_y = std::min(min_y,(double)point.y);
            max_x = std::max(max_x,(double)point.x);
            max_y = std::max(max_y,(double)point.y);
        }

        //resize the costmap to polygon boundaries, don't change resolution
        int size_x, size_y;
        worldToMapNoBounds(max_x - min_x, max_y - min_y, size_x, size_y);
        layered_costmap_->resizeMap(size_x, size_y, layered_costmap_->getCostmap()->getResolution(), min_x, min_y);
        matchSize();
        
        configured_ |= 0x1;
        has_updated_data_ = true;
        marked_ = false;
    }
    
    void PolygonObstacleLayer::updateObstaclePolygons(jsk_recognition_msgs::PolygonArray polygon_array)
    {
        
        ROS_INFO_STREAM("Obstacle Polygon Array Recieved");
        
        //error if no transform available between polygon and costmap
        if(!tf_listener_.waitForTransform(layered_costmap_->getGlobalFrameID(), polygon_array.header.frame_id,ros::Time::now(),ros::Duration(10))) {
            ROS_ERROR_STREAM("Couldn't transform from "<<layered_costmap_->getGlobalFrameID()<<" to "<< polygon_array.header.frame_id);
            return;
        }
        
        //clear existing boundary, if any
        obstacle_polygons_.clear();
        
        //Transform all points of each polygon into costmap frame
        geometry_msgs::PointStamped in, out;
        geometry_msgs::Point32 tmp_pnt;
        geometry_msgs::Polygon tmp_poly;
        int index = 0;        
        
        BOOST_FOREACH(geometry_msgs::PolygonStamped poly, polygon_array.polygons){
            in.header = poly.header;
            tmp_poly.points.clear();
            BOOST_FOREACH(geometry_msgs::Point32 point, poly.polygon.points){
                in.point.x = point.x;
                in.point.y = point.y;
                in.point.z = point.z;
                tf_listener_.transformPoint(layered_costmap_->getGlobalFrameID(),in,out);
                tmp_pnt.x = out.point.x;
                tmp_pnt.y = out.point.y;
                tmp_pnt.z = out.point.z;
                tmp_poly.points.push_back(tmp_pnt);
            }
            obstacle_polygons_.push_back(tmp_poly);
        }
        
        marked_ = false;
        configured_ |= 0x2;
    }
    
    void PolygonObstacleLayer::activate()
    {
        onInitialize();
    }

    void PolygonObstacleLayer::deactivate()
    {
        boundary_subscriber_.shutdown();
        obstacles_subscriber_.shutdown();
    }

    void PolygonObstacleLayer::reset()
    {
        marked_ = false;
        configured_ = 0;
        deactivate();
        activate();
    }        
    
    
    void PolygonObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y){

        //check if layer is enabled and configured with a boundary
        if ((false == enabled_) || (0x3!=configured_)|| (false == has_updated_data_)){ 
            return; 
        }
        
        ROS_INFO_STREAM("Bounds Updated");
        //update the whole costmap
        *min_x = getOriginX();
        *min_y = getOriginY();
        *max_x = getSizeInMetersX()+getOriginX();
        *max_y = getSizeInMetersY()+getOriginY();
        has_updated_data_ = false;

    }

    void PolygonObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        //check if layer is enabled and configured with a boundary
        if ((false == enabled_) || (0x3!=configured_)){ 
            return; 
        }
        ROS_INFO_STREAM("Costs Updated");
        
        geometry_msgs::Polygon tmp_poly;
        int i,j,k,x_1,y_1,x_2,y_2;

        //draw lines between each point in polygon
        MarkCell marker(costmap_, LETHAL_OBSTACLE);

        //circular iterator
        for(i=0, j=boundary_polygon_.points.size()-1; i < boundary_polygon_.points.size(); j = i++){
            worldToMapEnforceBounds(boundary_polygon_.points[i].x, boundary_polygon_.points[i].y, x_1,y_1);
            worldToMapEnforceBounds(boundary_polygon_.points[j].x, boundary_polygon_.points[j].y, x_2,y_2);

            raytraceLine(marker,x_1,y_1,x_2,y_2);
        }
        
        
        for (k=0;k<obstacle_polygons_.size();k++){
        
            tmp_poly = obstacle_polygons_[k];
            
            for(i=0, j=tmp_poly.points.size()-1; i < tmp_poly.points.size(); j = i++){
                worldToMapEnforceBounds(tmp_poly.points[i].x, tmp_poly.points[i].y, x_1,y_1);
                worldToMapEnforceBounds(tmp_poly.points[j].x, tmp_poly.points[j].y, x_2,y_2);

                raytraceLine(marker,x_1,y_1,x_2,y_2);
            }
         }            
        //update the master grid from the internal costmap
        mapUpdateKeepObstacles(master_grid, min_i, min_j, max_i, max_j);


    }

    void PolygonObstacleLayer::mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){

        unsigned char* master = master_grid.getCharMap();
        unsigned int span = master_grid.getSizeInCellsX();

        for (int j = min_j; j < max_j; j++)
        {
            unsigned int it = span*j+min_i;
            for (int i = min_i; i < max_i; i++)
            {
                //only update master grid if local costmap cell is lethal/higher value, and is not overwriting a lethal obstacle in the master grid
                if(master[it] != LETHAL_OBSTACLE && (costmap_[it] == LETHAL_OBSTACLE || costmap_[it] > master[it])){
                    master[it] = costmap_[it];
                }
                it++;
            }
        }
        marked_ = true;
    }
}
