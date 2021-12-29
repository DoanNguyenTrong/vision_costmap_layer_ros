#pragma once

#include    <memory>
#include    <mutex>
#include    <string>
#include    <vector>

#include    <XmlRpcValue.h>

#include    <ros/ros.h>
#include    <costmap_2d/layered_costmap.h>
#include    <costmap_2d/layer.h>
#include    <costmap_2d/costmap_2d.h>

#include    <costmap_2d/GenericPluginConfig.h>
#include    <dynamic_reconfigure/server.h>

#include    <custom_msgs/Obstacles.h>
#include    <custom_msgs/Zone.h>



namespace vision_layer{

// Define VisionLayer
class VisionLayer: public costmap_2d::Layer, public costmap_2d::Costmap2D{
public:
    // Constructor
    VisionLayer();
    virtual void onInitialize();
    // @brief Define the area that will need to be update. 
    //        Calculate the point (mark_x_, mark_y_) then expand the min/max bounds to assure it includes the new point.
    virtual void updateBounds(  double , double , double , 
                                double *, double *, 
                                double *, double *);
    // @brief Calculate which grid cell our point is in using worldToMap.
    //        Then set the cost of that cell.
    virtual void updateCosts(costmap_2d::Costmap2D &, 
                                int , int , 
                                int , int );
    
    bool isDiscretized(){
        return true;
    }
    virtual void matchSize();

private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &, uint32_t );
    
    // double mark_x_, mark_y_;
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};

}