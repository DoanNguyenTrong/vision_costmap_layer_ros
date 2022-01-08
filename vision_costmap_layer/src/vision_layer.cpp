#include <geometry_msgs/Pose.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

#include <XmlRpcException.h>

#include "vision_costmap_layer/vision_layer.hpp"


PLUGINLIB_EXPORT_CLASS(vision_costmap_layer::VisionLayer, costmap_2d::Layer);

static const std::string tag {"[Vision-LAYER] "};

namespace vision_costmap_layer {


VisionLayer::VisionLayer() :
    _dsrv(nullptr)
{}


VisionLayer::~VisionLayer()
{
    if (_dsrv) {
        _dsrv = nullptr;
    }
}


void VisionLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;

    nh.param<std::string>("base_link", _base_frame, "base_link");
    nh.param<std::string>("global_frame", _map_frame, "odom");
    nh.param("one_zone", _one_zone_mode, true);
    nh.param("clear_obstacle", _clear_obstacles, true);

    // Dynamic reconfiguration
    _dsrv = std::make_shared<dynamic_reconfigure::Server<VisionLayerConfig>>(nh);
    dynamic_reconfigure::Server<VisionLayerConfig>::CallbackType cb = boost::bind(&VisionLayer::reconfigureCb, this, _1, _2);
    _dsrv->setCallback(cb);

    // save resolution
    _costmap_resolution = layered_costmap_->getCostmap()->getResolution();

    // set initial bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // reading the defined topics out of the namespace of this plugin!
    std::string param {"zone_topics"};
    topicConfigure(nh, param);
    param = "obstacle_topics";
    topicConfigure(nh, param);

    // compute map bounds for the current set of areas and obstacles.
    computeMapBounds();
}


void VisionLayer::topicConfigure(ros::NodeHandle &nh, const std::string &param)
{
    XmlRpc::XmlRpcValue param_yaml;

    if (nh.getParam(param, param_yaml)) {
        // Valid & Array of string
        if ((param_yaml.valid() == false) || (param_yaml.getType() != XmlRpc::XmlRpcValue::TypeArray)) {
            ROS_ERROR_STREAM(tag << "invalid list of topic names. Must be a non-empty list of strings");
            throw std::runtime_error("invalid list of topic names. Must be a non-empty list of strings");
        }

        if (param_yaml.size() == 0) {
            ROS_WARN_STREAM(tag << "Empty list of topic names. Ignoring " << param);
        }

        // Loop through all strings and find params
        for (std::size_t i = 0; i < param_yaml.size(); ++i) {
            if (param_yaml[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_WARN_STREAM(tag << param_yaml[i] << " is not a string! Ignored");
            } else {
                std::string topic_name(param_yaml[i]);
                
                if ((!topic_name.empty()) && (topic_name.at(0) != '/')) {
                    topic_name = "/" + topic_name;
                }

                if (param.compare("zone_topics") == 0) {
                    _subs.push_back(nh.subscribe(topic_name, 100, &VisionLayer::zoneCallback, this));
                    ROS_INFO_STREAM(tag << "subscribed to topic " << _subs.back().getTopic().c_str());
                } else if (param.compare("obstacle_topics") == 0) {
                    _subs.push_back(nh.subscribe(topic_name, 100, &VisionLayer::obstaclesCallback, this));
                    ROS_INFO_STREAM(tag << "subscribed to topic " << _subs.back().getTopic().c_str());
                } else{
                    ROS_WARN_STREAM(tag << "Unuse param for querry " << param << ": " << topic_name);
                }

            }
        }
    } else {
        ROS_ERROR_STREAM(tag << "cannot read " << param << " from parameter server");
    }
}


bool VisionLayer::robotInZone(const Polygon &zone)
{
    if (!_one_zone_mode) {
        ROS_WARN_STREAM(tag << "could be applied only for one_zone_mode");
        return true;
    }

    geometry_msgs::Point point = getRobotPoint();
    std::size_t i, j;
    std::size_t size = zone.size();
    bool result = false;

    for (i = 0, j = size - 1; i < size; j = ++i) {
        if (((zone[i].y > point.y) != (zone[j].y > point.y)) &&
            (point.x < (zone[j].x - zone[i].x) * (point.y - zone[i].y) / (zone[j].y - zone[i].y) + zone[i].x)) {
            result = !result;
        }
    }

    return result;
}


void VisionLayer::reconfigureCb(VisionLayerConfig &config, uint32_t level)
{
    enabled_ = config.enabled;
    _one_zone_mode = config.one_zone;
    _clear_obstacles = config.clear_obstacles;
    _base_frame = config.base_frame;
    _map_frame = config.map_frame;
}


void VisionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                double *min_x, double *min_y, double *max_x, double *max_y)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> l(_data_mutex);

    if (_obstacle_points.empty() && _zone_polygons.empty() && _obstacle_polygons.empty()) {
        return;
    }

    *min_x = std::min(*min_x, _min_x);
    *min_y = std::min(*min_y, _min_y);
    *max_x = std::max(*max_x, _max_x);
    *max_y = std::max(*max_y, _max_y);
}


void VisionLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> l(_data_mutex);

    // set costs of zone polygons
    for (int i = 0; i < _zone_polygons.size(); ++i) {
        setPolygonCost(master_grid, _zone_polygons[i], costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, false);
    }

    // set costs of obstacle polygons
    for (int i = 0; i < _obstacle_polygons.size(); ++i) {
        setPolygonCost(master_grid, _obstacle_polygons[i], costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, true);
    }

    // set cost of obstacle points
    for (int i = 0; i < _obstacle_points.size(); ++i) {
        unsigned int mx;
        unsigned int my;
        if (master_grid.worldToMap(_obstacle_points[i].x, _obstacle_points[i].y, mx, my)) {
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
}


void VisionLayer::computeMapBounds()
{
    std::lock_guard<std::mutex> l(_data_mutex);

    // reset bounds
    _min_x = _min_y = _max_x = _max_y = 0;

    // iterate zone polygons
    for (int i = 0; i < _zone_polygons.size(); ++i) {
        for (int j = 0; j < _zone_polygons.at(i).size(); ++j) {
            double px = _zone_polygons.at(i).at(j).x;
            double py = _zone_polygons.at(i).at(j).y;
            _min_x = std::min(px, _min_x);
            _min_y = std::min(py, _min_y);
            _max_x = std::max(px, _max_x);
            _max_y = std::max(py, _max_y);
        }
    }

    // iterate obstacle polygons
    for (int i = 0; i < _obstacle_polygons.size(); ++i) {
        for (int j = 0; j < _obstacle_polygons.at(i).size(); ++j) {
            double px = _obstacle_polygons.at(i).at(j).x;
            double py = _obstacle_polygons.at(i).at(j).y;
            _min_x = std::min(px, _min_x);
            _min_y = std::min(py, _min_y);
            _max_x = std::max(px, _max_x);
            _max_y = std::max(py, _max_y);
        }
    }

    // iterate obstacle points
    for (int i = 0; i < _obstacle_points.size(); ++i) {
        double px = _obstacle_points.at(i).x;
        double py = _obstacle_points.at(i).y;
        _min_x = std::min(px, _min_x);
        _min_y = std::min(py, _min_y);
        _max_x = std::max(px, _max_x);
        _max_y = std::max(py, _max_y);
    }
}

void VisionLayer::setPolygonCost(costmap_2d::Costmap2D &master_grid, const Polygon &polygon, unsigned char cost,
                                  int min_i, int min_j, int max_i, int max_j, bool fill_polygon)
{
    std::vector<PointInt> map_polygon;
    for (unsigned int i = 0; i < polygon.size(); ++i) {
        PointInt loc;
        master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
        map_polygon.push_back(loc);
    }

    std::vector<PointInt> polygon_cells;

    // get the cells that fill the polygon
    rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

    // set the cost of those cells
    for (unsigned int i = 0; i < polygon_cells.size(); ++i) {
        int mx = polygon_cells[i].x;
        int my = polygon_cells[i].y;
        // check if point is outside bounds
        if (mx < min_i || mx >= max_i)
            continue;
        if (my < min_j || my >= max_j)
            continue;
        master_grid.setCost(mx, my, cost);
    }
}


void VisionLayer::polygonOutlineCells(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells)
{
    for (unsigned int i = 0; i < polygon.size() - 1; ++i) {
        raytrace(polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y, polygon_cells);
    }
    if (!polygon.empty()) {
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        raytrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
    }
}


void VisionLayer::raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt> &cells)
{
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (; n > 0; --n) {
        cells.push_back(pt);

        if (error > 0) {
            pt.x += x_inc;
            error -= dy;
        } else {
            pt.y += y_inc;
            error += dx;
        }
    }
}


void VisionLayer::rasterizePolygon(const std::vector<PointInt> &polygon, std::vector<PointInt> &polygon_cells, bool fill)
{
    // this implementation is a slighly modified version of Costmap2D::convexFillCells(...)

    //we need a minimum polygon of a traingle
    if (polygon.size() < 3)
        return;

    //first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    if (!fill)
        return;

    //quick bubble sort to sort points by x
    PointInt swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1) {
        if (polygon_cells[i].x > polygon_cells[i + 1].x) {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if (i > 0)
                --i;
        } else
            ++i;
    }

    i = 0;
    PointInt min_pt;
    PointInt max_pt;
    int min_x = polygon_cells[0].x;
    int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

    //walk through each column and mark cells inside the polygon
    for (int x = min_x; x <= max_x; ++x) {
        if (i >= (int)polygon_cells.size() - 1)
            break;

        if (polygon_cells[i].y < polygon_cells[i + 1].y) {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        } else {
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }

        i += 2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x) {
            if (polygon_cells[i].y < min_pt.y)
                min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y)
                max_pt = polygon_cells[i];
            ++i;
        }

        PointInt pt;
        //loop though cells in the column
        for (int y = min_pt.y; y < max_pt.y; ++y) {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}


void VisionLayer::zoneCallback(const custom_msgs::ZoneConstPtr &zone_msg)
{
    if (zone_msg->area.form.size() > 2) {
        Polygon vector_to_add;
        for (int i = 0; i < zone_msg->area.form.size(); ++i) {
            vector_to_add.push_back(zone_msg->area.form[i]);
        }

        if (!robotInZone(vector_to_add)) {
            ROS_WARN_STREAM(tag << "Robot point is not the navigation zone");
            return;
        }

        if (_one_zone_mode) {
            _zone_polygons.clear();
        }
        _zone_polygons.push_back(vector_to_add);

        computeMapBounds();
    } else {
        ROS_ERROR_STREAM(tag << "A zone Layer needs to be a polygon with minimun 3 edges");
    }
}


void VisionLayer::obstaclesCallback(const custom_msgs::ObstaclesConstPtr &obstacles_msg)
{
    if (_clear_obstacles) {
        _obstacle_polygons.clear();
        _obstacle_points.clear();
    }

    for (int i = 0; i < obstacles_msg->list.size(); ++i) {
        Polygon vector_to_add;
        if (obstacles_msg->list[i].form.size() == 1) {
            if (obstacles_msg->list[i].form[0].z == 0.0) {
                ROS_INFO_STREAM(tag << "Adding a Point");
                _obstacle_points.push_back(obstacles_msg->list[i].form[0]);
            } else if (obstacles_msg->list[i].form[0].z > 0.0) {
                ROS_INFO_STREAM(tag << "Adding a Circle");
                // Loop over 36 angles around a circle making a point each time
                int N = 36;
                geometry_msgs::Point pt;
                for (int j = 0; j < N; ++j) {
                    double angle = j * 2 * M_PI / N;
                    pt.x = obstacles_msg->list[i].form[0].x + cos(angle) * obstacles_msg->list[i].form[0].z;
                    pt.y = obstacles_msg->list[i].form[0].y + sin(angle) * obstacles_msg->list[i].form[0].z;
                    vector_to_add.push_back(pt);
                }
                _obstacle_polygons.push_back(vector_to_add);
            }
        } else if (obstacles_msg->list[i].form.size() == 2) {
            auto current_msg = obstacles_msg->list[i].form;
            ROS_INFO_STREAM(tag << "Adding a Line {"<<current_msg[0].x<< ", "<< current_msg[0].y << "}-{"<<current_msg[1].x<< ","<<current_msg[1].y<<"}");

            geometry_msgs::Point point_A = obstacles_msg->list[i].form[0];
            geometry_msgs::Point point_B = obstacles_msg->list[i].form[1];
            vector_to_add.push_back(point_A);
            vector_to_add.push_back(point_B);

            // calculate the normal vector for AB
            geometry_msgs::Point point_N;
            point_N.x = point_B.y - point_A.y;
            point_N.y = point_A.x - point_B.x;

            // get the absolute value of N to normalize and get
            // it to the length of the costmap resolution
            double abs_N = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
            point_N.x = point_N.x / abs_N * _costmap_resolution;
            point_N.y = point_N.y / abs_N * _costmap_resolution;

            // calculate the new points to get a polygon which can be filled
            geometry_msgs::Point point;
            point.x = point_A.x + point_N.x;
            point.y = point_A.y + point_N.y;
            vector_to_add.push_back(point);

            point.x = point_B.x + point_N.x;
            point.y = point_B.y + point_N.y;
            vector_to_add.push_back(point);

            _obstacle_polygons.push_back(vector_to_add);
        } else {
            ROS_INFO_STREAM(tag << "Adding a Polygon");
            for (int j = 0; j < obstacles_msg->list[i].form.size(); ++j) {
                vector_to_add.push_back(obstacles_msg->list[i].form[j]);
            }
            _obstacle_polygons.push_back(vector_to_add);
        }
    }
    computeMapBounds();
}


geometry_msgs::Point VisionLayer::getRobotPoint()
{
    tf::TransformListener tfListener;
    geometry_msgs::PoseStamped current_robot_pose, current_robot_pose_base;
    geometry_msgs::Point robot_point;
    geometry_msgs::TransformStamped current_transform_msg;
    tf::StampedTransform current_transform_tf;
    try {
        ros::Time now = ros::Time(0);
        tfListener.waitForTransform(_map_frame, _base_frame, now, ros::Duration(1.0));
        now = ros::Time::now();
        tfListener.getLatestCommonTime(_map_frame, _base_frame, now, nullptr);
        current_robot_pose_base.header.stamp = now;
        current_robot_pose_base.header.frame_id = _base_frame;
        current_robot_pose_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        tfListener.transformPose(_map_frame, current_robot_pose_base, current_robot_pose);
        robot_point.x = current_robot_pose.pose.position.x;
        robot_point.y = current_robot_pose.pose.position.y;
        robot_point.z = 0.0;
    } catch (tf::TransformException &ex) {
        ROS_DEBUG_STREAM(tag << "Can't get robot pose: " << ex.what());
    }
    return robot_point;
}
} // namespace vision_costmap_layer