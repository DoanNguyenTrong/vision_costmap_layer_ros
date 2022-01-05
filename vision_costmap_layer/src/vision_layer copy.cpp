#include  "vision_costmap_layer/vision_layer.hpp"

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

PLUGINLIB_EXPORT_CLASS(vision_layer::VisionLayer, costmap_2d::Layer)


static const std::string tag {"[VisionLayer] "};


namespace vision_layer{

VisionLayer::VisionLayer():dsrv_(nullptr){
};

VisionLayer::~VisionLayer(){
    if (dsrv_){
        dsrv_ = nullptr;
    }
};



void VisionLayer::onInitialize(){
    ros::NodeHandle nh("~/" + name_);
    
    current_ = true;
    base_frame_ = "base_link";
    map_frame_  = "map_frame";
    one_zone_mode_          = true;
    clear_obstacles_        = true;



    dsrv_ = std::make_shared<dynamic_reconfigure::Server<vision_costmap_layer::VisionLayerConfig>>(nh);
    dynamic_reconfigure::Server<vision_costmap_layer::VisionLayerConfig>::CallbackType 
                    cb = boost::bind(&VisionLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    
    costmap_resolution_ = layered_costmap_->getCostmap()->getResolution();

    min_x_ = min_y_ = max_x_ = max_y_ = 0.;

    std::string param {"zone_topics"};
    parseTopicsFromYaml(nh, param);
    param = "obstacle_topics";
    parseTopicsFromYaml(nh, param);
    param = "forms";
    parseFormListFromYaml(nh, param);

    computeMapBounds();

    ROS_INFO_STREAM(tag << "layer is initialized: [points: " 
                        << form_points_.size() 
                        << "][polygons: "
                        << form_polygon_.size() << "]");

}


void VisionLayer::parseTopicsFromYaml(ros::NodeHandle &nh, const std::string &param){
    XmlRpc::XmlRpcValue param_yaml;
    //@error: Old method of getting param 
    if (nh.getParam(param, param_yaml)){
        if ( !param_yaml.valid() || param_yaml.getType() != XmlRpc::XmlRpcValue::TypeArray){
            ROS_ERROR_STREAM(tag << "invalid topic name list. Must be non-empty list of string");
            throw std::runtime_error("Invalid topic name list. Must be non-empty list of string!");
        }
        if (param_yaml.size() == 0){
            ROS_WARN_STREAM(tag << "empty topic name list. Vision layer will have no effect on costmap!");
        }

        for (std::size_t i = 0; i < param_yaml.size(); i++){
            if (param_yaml[i].getType() != XmlRpc::XmlRpcValue::TypeString){
                ROS_WARN_STREAM(tag << "invalid topic name list: element " << i << "is not a string. Ignoring ...");
            }
            else{
                std::string topic_name(param_yaml[i]);
                ROS_INFO_STREAM(tag << "parseTopicsFromYaml() Topic name: " << topic_name);
                if (topic_name.empty() && topic_name.at(0) != '/'){
                    topic_name = "/" + topic_name;
                }

                if (param.compare("zone_topics") == 0){
                    subs_.push_back(nh.subscribe(topic_name, 100, &VisionLayer::zoneCallback, this));
                    
                }
                else if (param.compare("obstacle_topics") == 0){
                    subs_.push_back(nh.subscribe(topic_name, 100, &VisionLayer::obstacleCallback, this));
                }
                ROS_INFO_STREAM(tag << "subscribed to topic " << subs_.back().getTopic().c_str());
            }
        }
    } else{
        ROS_ERROR_STREAM(tag << "cannot read " << param << " from parameter server.");
    }

}

void VisionLayer::parseFormListFromYaml(ros::NodeHandle &nh, const std::string &param){
    XmlRpc::XmlRpcValue param_yaml;
    //@error: Old method of getting param 
    if (nh.getParam(param, param_yaml)){
        if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray){
            for (std::size_t i = 0; i < param_yaml.size(); i++ ){
                geometry_msgs::Point point;
                Polygon vector_to_add;
                if (param_yaml[i].getType() == XmlRpc::XmlRpcValue::TypeArray){
                    // Point
                    if(param_yaml[i].size() == 1){
                        try{
                            convert(param_yaml[i][0], point);
                        } catch (...){
                            continue;
                        }
                        form_points_.push_back(point);
                    }
                    // Polygon
                    else if (param_yaml[i].size() == 2 ){
                        if (param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble ||
                            param_yaml[i][0].getType() == XmlRpc::XmlRpcValue::TypeInt){
                            try{
                                convert(param_yaml[i], point);
                            } catch (...){
                                continue;
                            }
                            form_points_.push_back(point);
                        } else{
                            vector_to_add.reserve(3);
                            geometry_msgs::Point a, b;
                            try {
                                convert(param_yaml[i][0], a);
                                convert(param_yaml[i][1], b);
                            } catch (...){
                                continue;
                            }
                            vector_to_add.push_back(a);
                            vector_to_add.push_back(b);
                            // calculate normal vector for AB
                            geometry_msgs::Point n;
                            n.x = b.y - a.y;
                            n.y = b.x = a.x;
                            double abs_n = sqrt(pow(n.x, 2) + pow(n.y, 2));
                            n.x = n.x / abs_n * costmap_resolution_;
                            n.y = n.y / abs_n * costmap_resolution_;

                            point.x = a.x + n.x;
                            point.y = a.y + n.y;
                            vector_to_add.push_back(point);
                            point.x = b.x + n.x;
                            point.y = b.y + n.y;
                            vector_to_add.push_back(point);
                            form_polygon_.push_back(vector_to_add);

                        }
                    }
                    // Polygon
                    else if (param_yaml[i].size() >= 3){
                        vector_to_add.reserve(param_yaml.size());
                        for (std::size_t j = 0; j < param_yaml[i].size(); j++){
                            try{
                                convert(param_yaml[i][j], point);
                            }
                            catch(...){
                                continue;
                            }
                            vector_to_add.push_back(point);
                        }
                        if (!vector_to_add.empty()){
                            form_polygon_.push_back(vector_to_add);
                        }
                    }
                    
                } else{
                    ROS_ERROR_STREAM(tag << param << "with index #" << i << " is corrupted!");
                    ROS_ERROR_STREAM(param_yaml[i]);
                }
            }
        } else {
            ROS_ERROR_STREAM(tag << param << " struct is corrupted!");  
        }
    } else{
        ROS_ERROR_STREAM(tag << " could not read " << param << "from paramerter server.");
    }
}

void VisionLayer::convert(const XmlRpc::XmlRpcValue &val, geometry_msgs::Point &point){
    try {
        // check if there are 2 values for a coordinate
        if (val.getType() == XmlRpc::XmlRpcValue::TypeArray && val.size() == 2){
            // LAMBDAS func.
            auto toDouble  = [](const XmlRpc::XmlRpcValue &val)->double {
                auto val_cpy  = val;
                if (val_cpy.getType() == XmlRpc::XmlRpcValue::TypeInt){
                    return int(val_cpy);
                }
                return val_cpy; // if not double, an exception is thrown
            };
            point.x = toDouble(val[0]);
            point.y = toDouble(val[1]);
        }
        else{
            ROS_ERROR_STREAM(tag << " a point has to contain two double values");
            throw std::runtime_error("a point has to contain two double values");
        }
    }catch (const XmlRpc::XmlRpcException &ex){
        ROS_ERROR_STREAM(tag << "could not convert point: [" << ex.getMessage() << "]");
        throw std::runtime_error("could not convert point: [" + ex.getMessage() + "]");
    }
}



bool VisionLayer::robotInZone(const Polygon &zone){
    if (!one_zone_mode_){
        ROS_WARN_STREAM(tag << "could not be applied, only for one_zone_mode");
        return true;
    }
    geometry_msgs::Point point = getRobotPoint();
    std::size_t i, j;
    std::size_t size = zone.size();
    
    bool result = false;
    for (i = 0, j = size - 1; i < size; j=++i){
        if ( ((zone[i].y > point.y) != (zone[j].y > point.y)) && 
             (point.x < (zone[j].x - zone[i].x) * (point.y - zone[i].y)/(zone[j].y - zone[i].y) + zone[i].x )){
            result != result;
        }
    }
    return result;
}


void VisionLayer::reconfigureCB(vision_costmap_layer::VisionLayerConfig &config, uint32_t level){
    enabled_            = config.enabled;
    one_zone_mode_      = config.one_zone;
    clear_obstacles_    = config.clear_obstacles;
    base_frame_         = config.base_frame;
    map_frame_          = config.map_frame;

}



void VisionLayer::updateBounds( double robot_x, double robot_y, double robot_yaw, 
                                double *min_x, double *min_y, 
                                double *max_x, double *max_y){
    if (!enabled_){
        return;
    }
    std::lock_guard<std::mutex> locker(data_mutex_);
    
    if (obstacle_points_.empty() && zone_polygon_.empty() && obstacle_polygon_.empty()){
        return;
    }

    *min_x = std::min(*min_x, min_x_);
    *min_y = std::min(*min_y, min_y_);
    *max_x = std::max(*max_x, max_x_);
    *max_y = std::max(*max_y, max_y_);
}


void VisionLayer::updateCosts(costmap_2d::Costmap2D &master_grid,
                                int min_i, int min_j,
                                int max_i, int max_j){
    if (!enabled_){
        return;
    }

    std::lock_guard<std::mutex> locker(data_mutex_);

    // set costs of zone polygons
    for (int i = 0; i < zone_polygon_.size(); i++){
        setPolygonCost(master_grid, zone_polygon_[i], costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, false);
    }

    // set costs of obstacle polygons
    for (int i = 0; i < obstacle_polygon_.size(); i++){
        setPolygonCost(master_grid, obstacle_polygon_[i],costmap_2d::LETHAL_OBSTACLE, min_i, min_j, max_i, max_j, true);
    }

    // set costs of obstacle points
    for (int i = 0; i < obstacle_points_.size(); i++){
        unsigned int mx, my;
        if (master_grid.worldToMap(obstacle_points_[i].x, obstacle_points_[i].y, mx, my)){
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }
}


void VisionLayer::computeMapBounds(){
    std::lock_guard<std::mutex> locker(data_mutex_);
    
    // reset bounds
    min_x_ = min_y_ = max_x_ = max_y_ = 0;

    // iterate zone polygons
    for (int i = 0; i < zone_polygon_.size(); i++){
        for (int j = 0; j < zone_polygon_.at(i).size(); j++){
            double px = zone_polygon_.at(i).at(j).x;
            double py = zone_polygon_[i][j].y;
            min_x_ = std::min(px, min_x_);
            min_y_ = std::min(py, min_y_);
            max_x_ = std::max(px, max_x_);
            max_y_ = std::max(py, max_y_);
        }
    }

    // iterate obstacle polygons
    for (int i = 0; i < obstacle_polygon_.size(); i ++){
        for (int j = 0; j < obstacle_polygon_[i].size(); j++){
            double px = obstacle_polygon_[i][j].x;
            double py = obstacle_polygon_[i][j].y;
            min_x_ = std::min(px, min_x_);
            min_y_ = std::min(py, min_y_);
            max_x_ = std::max(px, max_x_);
            max_y_ = std::max(py, max_y_);
        }
    }

    // iterate obstacle points

    for (int i = 0; i < obstacle_points_.size(); i ++){
        double px = obstacle_points_[i].x;
        double py = obstacle_points_[i].y;
        min_x_ = std::min(px, min_x_);
        min_y_ = std::min(py, min_y_);
        max_x_ = std::max(px, max_x_);
        max_y_ = std::max(py, max_y_);
        
    }
}

void VisionLayer::setPolygonCost(   costmap_2d::Costmap2D &master_grid, 
                                    const Polygon &polygon,
                                    unsigned char cost,
                                    int min_i, int min_j,
                                    int max_i, int max_j,
                                    bool fill_polygon){
        std::vector<PointInt2D> map_polygon;
        for (int i = 0; i < polygon.size(); i++){
            PointInt2D loc;
            master_grid.worldToMapNoBounds(polygon[i].x, polygon[i].y, loc.x, loc.y);
            map_polygon.push_back(loc);

        }

        // get the cells that fill the polygon
        std::vector<PointInt2D> polygon_cells;
        rasterizePolygon(map_polygon, polygon_cells, fill_polygon);

        // set the cost of those cells
        for (int i = 0; i < polygon_cells.size(); i++){
            int mx = polygon_cells[i].x;
            int my = polygon_cells[i].y;

            // if point is out of bound
            if (mx < min_i || mx >= max_i){
                continue;
            }
            if (my < min_j || my >= max_j){
                continue;
            }
            master_grid.setCost(mx, my, cost);
        }
    }


void VisionLayer::polygonOutlineCells(const std::vector<PointInt2D> &polygon,
                                        std::vector<PointInt2D> &polygon_cells){
    for (unsigned int i = 0; i < polygon.size(); i++){
        rayTrace(polygon[i].x, polygon[i].y, polygon[i+1].x, polygon[i+1].y, polygon_cells);
    }
    if (!polygon.empty()){
        unsigned int last_index = polygon.size() - 1;
        rayTrace(polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y, polygon_cells);
    }
}

void VisionLayer::rayTrace(int x0, int y0, int x1, int y1, std::vector<PointInt2D> &cells){
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    PointInt2D pt;
    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0)? 1:-1;
    int y_inc = (y1 > y0)? 1:-1;
    int error = dx - dy;
    dx *=2; dy*=2;
    for (; n> 0; n--){
        cells.push_back(pt);
        if (error > 0){
            pt.x += x_inc;
            error -= dy;
        }
        else{
            pt.y+= y_inc;
            error += dx;
        }
    }
}



void VisionLayer::rasterizePolygon(const std::vector<PointInt2D> &polygon,
                                    std::vector<PointInt2D> &polygon_cells, 
                                    bool fill){
    // Modify of Costmap2D::convexFillCells()
    if (polygon.size() < 3){
        return;
    }

    // get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);

    if (!fill) return;

    PointInt2D swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1){
        if (polygon_cells[i].x > polygon_cells[i + 1].x){
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i+1];
            polygon_cells[i+1] = swap;

            if (i > 0) i--;
        }else i++;
    }

    i = 0;
    PointInt2D min_pt, max_pt;
    int min_x = polygon_cells[0].x;
    int max_x = polygon_cells[(int)polygon_cells.size() - 1].x;

    // walk through each column and mark cells inside the polygon
    for (int x = min_x; x <=max_x; x++){
        if (i >= (int)polygon_cells.size() - 1){
            break;
        }
        if (polygon_cells[i].y < polygon_cells[i+1].y){
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i+1];
        }else{
            min_pt = polygon_cells[i+1];
            max_pt = polygon_cells[i];
        }
        i +=2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x){
            if (polygon_cells[i].y < min_pt.y){
                min_pt = polygon_cells[i];
            }
            else if (i < polygon_cells[i].y > max_pt.y){
                max_pt = polygon_cells[i];
            }
            i++;
        }


        // loop through cells in the column
        PointInt2D pt;
        for (int y = min_pt.y; y< max_pt.y; y++){
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}

void VisionLayer::zoneCallback(const custom_msgs::ZoneConstPtr &zone_msg){
    if (zone_msg->zone.points.size() > 2){
        Polygon vector_to_add;
        for (int i = 0; i < zone_msg->zone.points.size(); i++){
            vector_to_add.push_back(zone_msg->zone.points[i]);
        }
        if (!robotInZone(vector_to_add)){
            ROS_WARN_STREAM(tag << "Robot point is not in the navigation zone.");
            return;
        }
        if (one_zone_mode_){
            zone_polygon_.clear();
        }
        zone_polygon_.push_back(vector_to_add);

        computeMapBounds();
    }
    else{
        ROS_ERROR_STREAM(tag << "A zone layer needs to be a polygon with minimum 3 edges");
    }
}


void VisionLayer::obstacleCallback(const custom_msgs::ObstaclesConstPtr &obstacle_msg){
    if (clear_obstacles_){
        obstacle_polygon_.clear();
        obstacle_points_.clear();
    }

    for (int i = 0; i < obstacle_msg->obstacles.size();i++){
        Polygon vector_to_add;
        
        custom_msgs::Form current_obstacle = obstacle_msg->obstacles[i];
        
        // Point & Circle
        if (current_obstacle.points.size() == 1){
            if (current_obstacle.points[0].z == 0.0){
                ROS_INFO_STREAM(tag << "Adding a point: {"
                                    << current_obstacle.points[0].x 
                                    << "," 
                                    << current_obstacle.points[0].y 
                                    << "}");
                // ADD TO POINTS
                obstacle_points_.push_back(current_obstacle.points[0]);
            }
            else if (current_obstacle.points[0].z > 0.0){
                ROS_INFO_STREAM(tag << "Adding a circle.");
                // loop over 36 angles around a circle making a point each time
                int N = 36;
                geometry_msgs::Point pt;
                for (int j = 0; j < N; j++){
                    double angle = j * 2 * M_PI / N;
                    pt.x = current_obstacle.points[0].x + cos(angle) * current_obstacle.points[0].z;
                    pt.y = current_obstacle.points[0].y + sin(angle) * current_obstacle.points[0].z;
                    vector_to_add.push_back(pt);
                }

                // ADD TO POLYGONS
                obstacle_polygon_.push_back(vector_to_add);
            }
        }
        // Line
        else if (obstacle_msg->obstacles[i].points.size() == 2){
            ROS_INFO_STREAM(tag << "Adding a line.");
            geometry_msgs::Point first_point = current_obstacle.points[0];
            geometry_msgs::Point second_point= current_obstacle.points[1];
            vector_to_add.push_back(first_point);
            vector_to_add.push_back(second_point);

            // normal vector for AB
            // @warn Dumb implementation of interpolation for line
            geometry_msgs::Point point_N;
            point_N.x = first_point.y - second_point.y;
            point_N.y = first_point.x - second_point.x;


            double norm = sqrt(pow(point_N.x, 2) + pow(point_N.y, 2));
            point_N.x = point_N.x / norm * costmap_resolution_;
            point_N.y = point_N.y / norm * costmap_resolution_;

            geometry_msgs::Point point;
            point.x = first_point.x + point_N.x;
            point.y = first_point.y + point_N.y;
            vector_to_add.push_back(point);
            point.x = second_point.x + point_N.x;
            point.y = second_point.y + point_N.y;
            vector_to_add.push_back(point);

            // ADD TO POLYGONS
            obstacle_polygon_.push_back(vector_to_add);

        }
        else{
            ROS_INFO_STREAM(tag << "Adding a Polygon");
            for (int j = 0; j < current_obstacle.points.size(); j ++){
                vector_to_add.push_back(current_obstacle.points[i]);
            }
            // Add to POLYGONS
            obstacle_polygon_.push_back(vector_to_add);
        }
        
    }computeMapBounds();
}

geometry_msgs::Point VisionLayer::getRobotPoint(){
    tf::TransformListener           tf_listener;
    geometry_msgs::PoseStamped      current_robot_pose, current_robot_pose_base;
    geometry_msgs::Point            robot_point;
    geometry_msgs::TransformStamped current_tf_msg;
    tf::StampedTransform            current_tf;
    try{
        ros::Time now = ros::Time(0);
        tf_listener.waitForTransform(map_frame_, base_frame_, now, ros::Duration(1.0));
        now = ros::Time(0);
        tf_listener.getLatestCommonTime(map_frame_, base_frame_, now, nullptr);
        current_robot_pose_base.header.stamp = now;
        current_robot_pose_base.header.frame_id = base_frame_;
        current_robot_pose_base.pose.orientation = tf::createQuaternionMsgFromYaw(0);

        tf_listener.transformPose(map_frame_, current_robot_pose_base, current_robot_pose);
        robot_point.x = current_robot_pose.pose.position.x;
        robot_point.y = current_robot_pose.pose.position.y;
        robot_point.z = 0.0;
    }
    catch( tf::TransformException &ex){
        ROS_DEBUG_STREAM(tag << "Cant get robot pose: "<< ex.what());
    }
    return robot_point;
}


} //namespace