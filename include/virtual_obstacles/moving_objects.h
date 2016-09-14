#ifndef MOVING_OBJECTS_H_
#define MOVING_OBJECTS_H_

// std lib includes
#include <iostream>
#include <string>
#include <vector>
#include <map>

// ros includes
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>

// project intern includes
#include <virtual_obstacles/moving_object_msg.h>
#include <virtual_obstacles/GenericPluginConfig.h>


namespace virtual_obstacles
{

class MovingObjects : public costmap_2d::Layer, public costmap_2d::Costmap2D
{
public:
  MovingObjects();

  virtual void onInitialize();

  virtual void updateBounds(double origin_x, double origin_y, double origin_yaw,
                  double* min_x, double* min_y, double* max_x, double* max_y);

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                  int min_j, int max_i, int max_j);

  virtual void matchSize();

  bool isDiscretized()
  {
    return true;
  }

private:
  /*
   * Typdefs and structs
   */
  struct Coord2D {
    double x;
    double y;
    double theta;
  };

  struct State {
    ros::Time last_msg;
    Coord2D position;
    Coord2D velocity;
    std::vector<geometry_msgs::PoseStamped> route;
    //std::list<Coord2D> route;
  };

  typedef std::map<std::string, State> Dict;
  typedef Dict::const_iterator DictIt;
  typedef std::vector<geometry_msgs::PoseStamped>::iterator RouteIt;

  struct UpdateBoundingBox {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
  };

  /*
   * Privat methods
   */
  void reconfigureCB(virtual_obstacles::GenericPluginConfig &config, uint32_t level);

  void drawCircle(double circle_radius, double center_x, double center_y);

  void drawSimpleRectangle(double start_x, double start_y, double end_x, double end_y);

  void drawPoint(double x, double y);

  void msgMovingObjectCB(const virtual_obstacles::moving_object_msg::ConstPtr& msg);

  void deleteOldDataSets();

  void placeObstalcesInMap(double origin_x, double origin_y, double origin_yaw);

  /*
   * class wide variables
   */

  // dyn reconfigure
  dynamic_reconfigure::Server<virtual_obstacles::GenericPluginConfig> *dsrv_;
  double_t update_zone_enlargment;
  std::string own_unique_name;
  double_t obstacle_life_time;

  double_t max_obstical_scalling_distance;
  double_t min_obstical_scalling_distance;
  double_t max_velocity_of_objects;

  // msg system
  ros::NodeHandle nh;
  ros::Subscriber sub;

  // moving objects store
  Dict moving_objects_store;

  // store min/max x and y to update
  UpdateBoundingBox update_area;

}; // end class
} // end namespace
#endif
