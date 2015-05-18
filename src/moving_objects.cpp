#include <pluginlib/class_list_macros.h>

#include <virtual_obstacles/moving_objects.h>

PLUGINLIB_EXPORT_CLASS(virtual_obstacles::MovingObjects, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace virtual_obstacles
{

/* #####################################################################
 * initalisation and config handling
 * #####################################################################
 */
 
/* *********************************************************************
 * constructor
 */
MovingObjects::MovingObjects()
{
  update_zone_enlargment = 0.5;
  own_unique_name = "";
  obstacle_life_time = 3.0;
}

/* *********************************************************************
 * init function
 */
void MovingObjects::onInitialize()
{
  nh = ros::NodeHandle("~/" + name_);

  // register msg handling
  sub = nh.subscribe("/robot_state_publisher", 100, &MovingObjects::msgMovingObjectCB, this);
  
  // set plugin vars
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  // register dynamic reconfigure service
  dsrv_ = new dynamic_reconfigure::Server<virtual_obstacles::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<virtual_obstacles::GenericPluginConfig>::CallbackType cb = boost::bind(
      &MovingObjects::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

/* *********************************************************************
 * matchSize
 * 
 * match size between master grid and local grid
 */
void MovingObjects::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

/* *********************************************************************
 * reconfigureCB
 * 
 * dynamic reconfigure callback
 */
void MovingObjects::reconfigureCB(virtual_obstacles::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  update_zone_enlargment = config.update_zone_enlargment;
  own_unique_name = config.own_unique_name;
  obstacle_life_time = config.obstacle_life_time;
}

/* #####################################################################
 * Costmap creation and update
 * #####################################################################
 */

/* *********************************************************************
 * updateBounds
 * 
 * update obstacles and bounds 
 */
void MovingObjects::updateBounds(double origin_x, double origin_y,
      double origin_yaw, double* min_x, double* min_y, double* max_x,
      double* max_y)
{
  if (!enabled_)
    return;
  
  // ros spine once to process new messages
  ros::spinOnce();
  
  // cleanup old datasets in moving_objects_store
  deleteOldDataSets();
  
  // reset map
  for (int ind = 0; ind < (size_y_*size_x_); ind++)
    costmap_[ind] = NO_INFORMATION;
    
  // reset update bound
  update_area.min_x  = *min_x;
  update_area.max_x  = *max_x;
  update_area.min_y  = *min_y;
  update_area.max_y  = *max_y;
  
  // draw obstacles from moving obstacles store
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  
  double vx = 0.0;
  double vy = 0.0;
  double vres = 0.0;
  
  for (DictIt it(moving_objects_store.begin()); it != moving_objects_store.end(); it++)
  {
    std::cout << "Element: " << it->first << " = " << moving_objects_store[it->first].last_msg << std::endl;
    
    // if obstacles unique_name same as own unique_name, ignore it
    if (it->first == own_unique_name)
      continue;
    
    x = moving_objects_store[it->first].position.x;
    y = moving_objects_store[it->first].position.y;
    yaw = moving_objects_store[it->first].position.theta;
    
    vx = moving_objects_store[it->first].velocity.x;
    vy = moving_objects_store[it->first].velocity.y;
    vres = std::sqrt( std::pow(vx, 2) + std::pow(vy, 2) );
    
    
    drawCircle(0.18, x, y);
    drawPoint(x + cos(yaw)*vres*1, y + sin(yaw)*vres*1);
    drawPoint(x + cos(yaw)*vres*2, y + sin(yaw)*vres*2);
    drawPoint(x + cos(yaw)*vres*3, y + sin(yaw)*vres*3);
    
  }
  
  // update bounds
  *min_x = std::min(*min_x, update_area.min_x - update_zone_enlargment);
  *min_y = std::min(*min_y, update_area.min_y - update_zone_enlargment);
  *max_x = std::max(*max_x, update_area.max_x + update_zone_enlargment);
  *max_y = std::max(*max_y, update_area.max_y + update_zone_enlargment);
}

/* *********************************************************************
 * updateCosts
 * 
 * updates the master grid in the area defined in updateBounds
 */
void MovingObjects::updateCosts(costmap_2d::Costmap2D& master_grid,
      int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  
  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

/* #####################################################################
 * Message handling functions
 * #####################################################################
 */

void MovingObjects::msgMovingObjectCB(const virtual_obstacles::moving_object_msg::ConstPtr& msg)
{
  // Debug
  //ROS_INFO("I heard from  [%s]:", msg->unique_name.c_str());
  //std::cout << "Position: " << msg->pose.x << " / " << msg->pose.y << " / " << msg->pose.theta << std::endl;
  //std::cout << "Velocity: " << msg->velocity.linear.x << " / " << msg->velocity.linear.y << " / " << msg->velocity.angular.z << std::endl;
  
  // Save Msg data in Object Store
  State s;
  s.last_msg = ros::Time::now();
  
  s.position.x = msg->pose.x;
  s.position.y = msg->pose.y;
  s.position.theta = msg->pose.theta;
  
  s.velocity.x = msg->velocity.linear.x;
  s.velocity.y = msg->velocity.linear.y;
  s.velocity.theta = msg->velocity.angular.z;
  
  moving_objects_store[msg->unique_name.c_str()] = s;
  
}

/* #####################################################################
 * Drawing helper functions
 * #####################################################################
 */

/* *********************************************************************
 * drawCircle
 * 
 * draw a circle on local grid (costmap_)
 */
void MovingObjects::drawCircle(double circle_radius, double center_x, double center_y)
{
  double tol = 0.01;
  
  unsigned int pixle_center_x;
  unsigned int pixle_center_y;
  
  unsigned int pixle_tmp_x;
  unsigned int pixle_tmp_y;
  
  worldToMap(center_x, center_y, pixle_center_x, pixle_center_y);
  worldToMap(center_x + circle_radius, center_y, pixle_tmp_x, pixle_tmp_y);
  
  int max_distance = pixle_tmp_x - pixle_center_x;
  
  
  for (int cx = -max_distance; cx <= max_distance; cx++)
  {
    for (int cy = -max_distance; cy <= max_distance; cy++)
    {
      float distance_to_centre = sqrt(pow(((double)cx/(double)max_distance), 2) + pow(((double)cy/(double)max_distance), 2));
      
      if (distance_to_centre < 1.0)
        setCost(pixle_center_x + cx, pixle_center_y + cy, LETHAL_OBSTACLE);
    }
  }
  
  // update "update area"
  update_area.min_x = std::min( update_area.min_x, center_x - circle_radius);
  update_area.max_x = std::max( update_area.max_x, center_x + circle_radius);
  update_area.min_y = std::min( update_area.min_y, center_y - circle_radius);
  update_area.max_y = std::max( update_area.max_y, center_y + circle_radius);
}

/* *********************************************************************
 * drawSimpleRectangle
 * 
 * draw a simple rectangle on local grid (costmap_), simple means no
 * rotation
 */
void MovingObjects::drawSimpleRectangle(double start_x, double start_y, double end_x, double end_y)
{

  unsigned int pixle_start_x;
  unsigned int pixle_start_y;
  unsigned int pixle_end_x;
  unsigned int pixle_end_y;
  
  worldToMap(start_x, start_y, pixle_start_x, pixle_start_y);
  worldToMap(end_x, end_y, pixle_end_x, pixle_end_y);
  
  for (int i = pixle_start_x; i < pixle_end_x; i++)
  {
    setCost(i, pixle_start_y, LETHAL_OBSTACLE);
    setCost(i, pixle_end_y, LETHAL_OBSTACLE);
  }

  for (int i = pixle_start_y; i < pixle_end_y; i++)
  {
    setCost(pixle_start_x, i, LETHAL_OBSTACLE);
    setCost(pixle_end_x, i, LETHAL_OBSTACLE);
  }
  
  // update "update area"
  update_area.min_x = std::min( update_area.min_x, std::min(start_x, end_x));
  update_area.max_x = std::max( update_area.max_x, std::max(start_x, end_x));
  update_area.min_y = std::min( update_area.min_y, std::min(start_y, end_y));
  update_area.max_y = std::max( update_area.max_y, std::max(start_y, end_y));
}

/* *********************************************************************
 * drawPoint
 * 
 * draw a Point on local grid (costmap_)
 */
void MovingObjects::drawPoint(double x, double y)
{
  unsigned int pixle_x;
  unsigned int pixle_y;
  
  worldToMap(x, y, pixle_x, pixle_y);

  setCost(pixle_x, pixle_y, LETHAL_OBSTACLE);

  // update "update area"
  update_area.min_x = std::min( update_area.min_x, x);
  update_area.max_x = std::max( update_area.max_x, x);
  update_area.min_y = std::min( update_area.min_y, y);
  update_area.max_y = std::max( update_area.max_y, y);
}

/* #####################################################################
 * Other helper functions
 * #####################################################################
 */

/* *********************************************************************
 * deleteOldDataSets
 * 
 * delete old datasets from moving_objects_store
 */
void MovingObjects::deleteOldDataSets()
{
  for (DictIt it(moving_objects_store.begin()); it != moving_objects_store.end(); it++)
  {
    if (ros::Time::now() - moving_objects_store[it->first].last_msg >= ros::Duration(obstacle_life_time))
      moving_objects_store.erase (it->first);
  }
}

} // end namespace
