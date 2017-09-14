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
  update_zone_enlargment = 1.0;
  own_unique_name = "";
  obstacle_life_time = 3.0;

  // ToDo: make this dyn reconfigure option
  max_obstical_scalling_distance = 2.0;
  min_obstical_scalling_distance = 1.0;
  
  dynamic_speed_max = 0.5;
  dynamic_speed_min = 0.1;
  dynamic_speed = 0.0;
  
  max_velocity_of_objects = 1.0;
}

/* *********************************************************************
 * init function
 */
void MovingObjects::onInitialize()
{
  nh = ros::NodeHandle("~/" + name_);

  // register msg handling
  sub = nh.subscribe("/moving_objects", 1, &MovingObjects::msgMovingObjectCB, this);

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
 * placeObstalcesInMap
 *
 * place obstalces in map
 */
void MovingObjects::placeObstalcesInMap(double origin_x, double origin_y,
      double origin_yaw)
{
  // draw obstacles from moving obstacles store
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vres = 0.0;

  double distance_to_origin = 0.0;
  double scale_factor = 0.0;
  
  double closest_moving_object = 1e3;
  double speed_scaling = 0.0;
  double calculated_speed = 0.0;
  
  // if own_unique_name set to empty string derive unique name from namespace
  if (own_unique_name == "")
  {
    own_unique_name = ros::this_node::getNamespace();
    own_unique_name.erase (own_unique_name.begin(), own_unique_name.begin()+2);
  }

  for (DictIt it(moving_objects_store.begin()); it != moving_objects_store.end(); it++)
  {
    // if obstacles unique_name same as own unique_name, ignore it
    if (it->first == own_unique_name)
      continue;

    // short names for position
    x = moving_objects_store[it->first].position.x;
    y = moving_objects_store[it->first].position.y;
    yaw = moving_objects_store[it->first].position.theta;

    // calculate distance to origin in meter for dynamic scaling
    distance_to_origin = std::sqrt( std::pow((x-origin_x), 2) + std::pow((y-origin_y), 2) );
    scale_factor = ((distance_to_origin-min_obstical_scalling_distance) / (max_obstical_scalling_distance-min_obstical_scalling_distance));
    scale_factor = std::max(0.0, scale_factor);
    scale_factor = std::min(1.0, scale_factor);
    scale_factor = 1.0 - scale_factor;
    
    // short names for velocity
    // ToDo: could use pointers here
    vx = moving_objects_store[it->first].velocity.x;
    vy = moving_objects_store[it->first].velocity.y;
    //vres = std::sqrt( std::pow(vx, 2) + std::pow(vy, 2) );
    
    
    // limit vx, vy, vres (i.e. repositioning robots)
    if (vx > max_velocity_of_objects)
      vx = max_velocity_of_objects;
    if (vy > max_velocity_of_objects)
      vy = max_velocity_of_objects;
    //if (vres > max_velocity_of_objects)
    //  vres = max_velocity_of_objects;
      
    // update closest_moving_object (used for max velocity)
    if (distance_to_origin < closest_moving_object)
    {
      closest_moving_object = distance_to_origin;
      speed_scaling = 1.0 - scale_factor;
    }

    // block moving obstacle center point without exception
    drawPoint(x, y);

    // block moving obstacle footprint, scaled by distance to robot
    // low distance -> full size
    // high distance -> small (to no) size
    // ToDo: save obstacle shape in Msg
    drawCircle(0.20*scale_factor, x, y);

    // block moving obstacle route (smart or simple way)
    if (moving_objects_store[it->first].route.size() > 0)
    {
      // block moving obstacle planed route
      for (RouteIt it2= moving_objects_store[it->first].route.begin();
        it2 != moving_objects_store[it->first].route.end(); it2++)
      {
        drawPoint(it2->pose.position.x, it2->pose.position.y);
        //drawCircle(0.05, it2->pose.position.x, it2->pose.position.y);
      }
    } else {
      //block moving obstacle way, based on velocity (simple way)
      for (int i = 0; i < 20; i++)
      {
        drawPoint(x + vx*i/10.0, y + vy*i/10.0);
        //drawCircle(0.05, x + vx*i/10.0, y + vy*i/10.0);
      }
    }
  }
  
  // ToDo: test
  // at time of writing there is/was no reconfigure API for C++
  // maybe this command is very very slow and we need to only run it in case of changes
  calculated_speed = (dynamic_speed_max - dynamic_speed_min) * speed_scaling + dynamic_speed_min;
  if (fabs(calculated_speed - dynamic_speed) >= 0.1)
  {
    dynamic_speed = calculated_speed;
    std::stringstream system_call_cmd;
    system_call_cmd << "rosrun dynamic_reconfigure dynparam set move_base/DWAPlannerROS \"{ max_vel_x: " << calculated_speed << "}\" &";
    //system(system_call_cmd.str().c_str());
    // Debug code:
    //std::cout << "set speed " << calculated_speed << std::endl;
  }
  else
  {
    // Debug code:
    //std::cout << "keep speed " << dynamic_speed << std::endl;
  }
  

}

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

  // place obstacles
  placeObstalcesInMap(origin_x, origin_y, origin_yaw);

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
  // Save Msg data in Object Store
  State s;
  s.last_msg = ros::Time::now();

  s.position.x = msg->pose.x;
  s.position.y = msg->pose.y;
  s.position.theta = msg->pose.theta;

  s.velocity.x = msg->velocity.linear.x;
  s.velocity.y = msg->velocity.linear.y;
  s.velocity.theta = msg->velocity.angular.z;

  s.route = msg->route;

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
  
  bool ret;

  ret = worldToMap(center_x, center_y, pixle_center_x, pixle_center_y);
  if (not ret)
  {
    ROS_WARN_STREAM("world to map failed, object outside map?!");
    return;
  }
  
  ret = worldToMap(center_x + circle_radius, center_y, pixle_tmp_x, pixle_tmp_y);
  if (not ret)
  {
    ROS_WARN_STREAM("world to map failed, object outside map?!");
    return;
  }

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
  
  bool ret;

  ret = worldToMap(start_x, start_y, pixle_start_x, pixle_start_y);
  if (not ret)
  {
    ROS_WARN_STREAM("world to map failed, object outside map?!");
    return;
  }
  
  ret = worldToMap(end_x, end_y, pixle_end_x, pixle_end_y);
  if (not ret)
  {
    ROS_WARN_STREAM("world to map failed, object outside map?!");
    return;
  }

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

  bool ret = worldToMap(x, y, pixle_x, pixle_y);
  if (not ret)
  {
    ROS_WARN_STREAM("world to map failed, object outside map?!");
    return;
  }
  
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
    {
      // only delete one item per round
      // save trouble with working on modified list
      moving_objects_store.erase (it->first);
      return;
    }
  }
}

} // end namespace
