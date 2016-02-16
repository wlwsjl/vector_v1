#ifndef BOUNDED_EXPLORE_LAYER_H_
#define BOUNDED_EXPLORE_LAYER_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/Polygon.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <vector>

namespace vector_costmap_plugins
{

/**
 * @brief costmap_2d layer plugin that holds the state for a bounded frontier exploration task.
 * Manages the boundary polygon, superimposes the polygon on the overall exploration costmap,
 * and processes costmap to find next frontier to explore.
 */
class PolygonObstacleLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
    PolygonObstacleLayer();
    ~PolygonObstacleLayer();
    
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void matchSize();

private:

    /**
     * @brief Load polygon boundary to draw on map with each update
     * @param polygon_stamped polygon boundary
     * @return void
     */
    void updateBoundaryPolygon(geometry_msgs::PolygonStamped polygon_stamped);
    
    /**
     * @brief Load a polygon array containing "keep out" zones to draw on map with each update
     * @param polygon_array polygon obstacle array
     * @return void
     */
    void updateObstaclePolygons(jsk_recognition_msgs::PolygonArray polygon_array);

    /**
     * @brief Update the map with exploration boundary data
     * @param master_grid Reference to master costmap
     */
    void mapUpdateKeepObstacles(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
    ros::Subscriber boundary_subscriber_;
    ros::Subscriber obstacles_subscriber_;
    geometry_msgs::Polygon boundary_polygon_;
    std::vector<geometry_msgs::Polygon> obstacle_polygons_;
    tf::TransformListener tf_listener_;
    bool marked_, has_updated_data_;
    int configured_;
};

}
#endif
