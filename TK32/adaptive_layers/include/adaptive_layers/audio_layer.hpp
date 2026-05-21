#ifndef AUDIO_LAYER_HPP_
#define AUDIO_LAYER_HPP_

#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <mutex>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include <map>
#include <iostream>

namespace adaptive_layers
{

    class AudioLayer : public nav2_costmap_2d::CostmapLayer
    {      
        public:
        AudioLayer();
        void onInitialize() override;
        void updateBounds(double robot_x, double robot_y, double robot_yaw, 
                                  double * min_x,double * min_y,double * max_x,double * max_y) override;
        void updateCosts(nav2_costmap_2d::Costmap2D & master_grid,
                                 int min_i, int min_j, int max_i, int max_j) override;
        void reset() override;
        bool isClearable() override {return false;}
        void matchSize() override;

        private:
        //variables
        
        double param_1_;
        bool need_resize;
        std::vector<uint8_t> audio_costs_;
        std::mutex audio_mutex_;

        // functions

        void audioMapCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr audio_sub_;
        // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr layerSizePub_;

    };

}  // namespace adaptive_layers

#endif