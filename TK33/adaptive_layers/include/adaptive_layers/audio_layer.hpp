#ifndef AUDIO_LAYER_HPP_
#define AUDIO_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

namespace adaptive_layers
{

    class AudioLayer : public nav2_costmap_2d::CostmapLayer
    {
                
        protected:
        rclcpp::Logger logger_{rclcpp::get_logger("adaptive_layers.AudioLayer")};

        public:
        AudioLayer();

        void onInitialize() override;

        void updateBounds(double, double, double, double*, double*, double*, double*) override;

        void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;

        void matchSize() override;

        void reset() override;

        bool isClearable() override;
        };

}  // namespace adaptive_layers

#endif