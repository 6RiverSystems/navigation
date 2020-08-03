#ifndef COSTMAP_2D_TESTING_HELPER_H
#define COSTMAP_2D_TESTING_HELPER_H

#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/static_layer_with_inflation.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/obstruction_layer.h>
#include <costmap_2d/inflation_layer.h>

<<<<<<< HEAD
#include <iomanip>
=======
#include <sensor_msgs/point_cloud2_iterator.h>
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691

const double MAX_Z(1.0);

void setValues(costmap_2d::Costmap2D& costmap, const unsigned char* map)
{
  int index = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      costmap.setCost(j, i, map[index]);
    }
  }
}

char printableCost(unsigned char cost)
{
  switch (cost)
  {
  case costmap_2d::NO_INFORMATION: return '?';
  case costmap_2d::LETHAL_OBSTACLE: return 'L';
  case costmap_2d::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case costmap_2d::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap(costmap_2d::Costmap2D& costmap)
{
<<<<<<< HEAD
  // change printf() to std::cerr inorder to visualize the testing map
  std::cerr<< "map:\n";
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      std::cerr <<  std::setw(3) << std::setfill(' ') << int(costmap.getCost(j, i)) << " ";
=======
  printf("map:\n");
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      printf("%4d", int(costmap.getCost(j, i)));
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
    }
    std::cerr << "\n\n";
  }
}

unsigned int countValues(costmap_2d::Costmap2D& costmap, unsigned char value, bool equal = true)
{
  unsigned int count = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value))
      {
        count+=1;
      }
    }
  }
  return count;
}

void addStaticLayer(costmap_2d::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  costmap_2d::StaticLayer* slayer = new costmap_2d::StaticLayer();
<<<<<<< HEAD
  layers.addPlugin(boost::shared_ptr<costmap_2d::CostmapLayer>(slayer));
  slayer->initialize(&layers, "static", &tf);
}

void addStaticLayerWithInflation(costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  costmap_2d::StaticLayerWithInflation* slayer = new costmap_2d::StaticLayerWithInflation();
  layers.addPlugin(boost::shared_ptr<costmap_2d::CostmapLayer>(slayer));
=======
  layers.addPlugin(boost::shared_ptr<costmap_2d::Layer>(slayer));
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
  slayer->initialize(&layers, "static", &tf);
}

costmap_2d::ObstacleLayer* addObstacleLayer(costmap_2d::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  costmap_2d::ObstacleLayer* olayer = new costmap_2d::ObstacleLayer();
  olayer->initialize(&layers, "obstacles", &tf);
<<<<<<< HEAD
  layers.addPlugin(boost::shared_ptr<costmap_2d::CostmapLayer>(olayer));
  return olayer;
}

costmap_2d::ObstructionLayer* addObstructionLayer(costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  costmap_2d::ObstructionLayer* olayer = new costmap_2d::ObstructionLayer();
  olayer->initialize(&layers, "obstructions", &tf);
  layers.addPlugin(boost::shared_ptr<costmap_2d::CostmapLayer>(olayer));
=======
  layers.addPlugin(boost::shared_ptr<costmap_2d::Layer>(olayer));
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
  return olayer;
}

void addObservation(costmap_2d::ObstacleLayer* olayer, double x, double y, double z = 0.0,
<<<<<<< HEAD
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z, double min_raytrace_range = 0.0){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = x;
  cloud.points[0].y = y;
  cloud.points[0].z = z;
=======
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z){
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  // set minimum raytrace
  costmap_2d::Observation obs(p, cloud, 100.0, 100.0, min_raytrace_range);  // obstacle range = raytrace range = 100.0
  olayer->addStaticObservation(obs, true, true);
}

<<<<<<< HEAD
void addObservation(costmap_2d::ObstructionLayer* olayer, double x, double y, double z = 0.0,
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z, double min_raytrace_range = 0.0){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = x;
  cloud.points[0].y = y;
  cloud.points[0].z = z;

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  // set minimum raytrace
  costmap_2d::Observation obs(p, cloud, 100.0, 100.0, min_raytrace_range);  // obstacle range = raytrace range = 100.0
  olayer->addStaticObservation(obs, true, true);
}

// costmap_2d::InflationLayer* addInflationLayer(costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
// {
//   costmap_2d::InflationLayer* ilayer = new costmap_2d::InflationLayer();
//   ilayer->initialize(&layers, "inflation", &tf);
//   boost::shared_ptr<costmap_2d::Layer> ipointer(ilayer);
//   layers.addPlugin(ipointer);
//   return ilayer;
// }

std::vector<geometry_msgs::Point> setRadii(costmap_2d::LayeredCostmap& layers, double length, double width, double inflation_radius)
{
  std::vector<geometry_msgs::Point> polygon;
  geometry_msgs::Point p;
  p.x = width;
  p.y = length;
  polygon.push_back(p);
  p.x = width;
  p.y = -length;
  polygon.push_back(p);
  p.x = -width;
  p.y = -length;
  polygon.push_back(p);
  p.x = -width;
  p.y = length;
  polygon.push_back(p);
  layers.setFootprint(polygon);

  ros::NodeHandle nh;
  nh.setParam("/inflation_tests/inflation/inflation_radius", inflation_radius);

  return polygon;
=======
costmap_2d::InflationLayer* addInflationLayer(costmap_2d::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  costmap_2d::InflationLayer* ilayer = new costmap_2d::InflationLayer();
  ilayer->initialize(&layers, "inflation", &tf);
  boost::shared_ptr<costmap_2d::Layer> ipointer(ilayer);
  layers.addPlugin(ipointer);
  return ilayer;
>>>>>>> 4dca4370b914bf8b13eb766c98a1137063826691
}


#endif  // COSTMAP_2D_TESTING_HELPER_H
