#include "adaptive_layers/audio_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace adaptive_layers
{

    AudioLayer::AudioLayer() {}

    void AudioLayer::onInitialize()
    {
        auto node = node_.lock();  // lifecycle node
        logger_ = node->get_logger();
        RCLCPP_INFO(logger_, "AudioLayer initialized");

    }

    void AudioLayer::matchSize()
    
    {
        CostmapLayer::matchSize();
        auto * master = layered_costmap_->getCostmap();
        resizeMap(
            master->getSizeInCellsX(),
            master->getSizeInCellsY(),
            master->getResolution(),
            master->getOriginX(),
            master->getOriginY()
        );
    }


    void AudioLayer::updateBounds(double, double, double, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        // Update entire map
        *min_x = 0.0;
        *min_y = 0.0;
        *max_x = layered_costmap_->getCostmap()->getSizeInMetersX();
        *max_y = layered_costmap_->getCostmap()->getSizeInMetersY();
        
        //RCLCPP_WARN(logger_, "x = %.2f", *max_x);

    }

    void AudioLayer::updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
    {
        
        if (!enabled_) {
            return;
        }

        uint8_t cost = 0;
        for (int j = min_j; j < max_j; j++)
        {
            for (int i = min_i; i < max_i; i++)
            {
                // Audio: create a simple gradient
                cost = std::min<uint8_t>(100, (i + j) % 100);
                master.setCost(i, j, cost);
            }
        }
    }
  
    void AudioLayer::reset()
    {
        // Can be empty for now
    }

    bool AudioLayer::isClearable()
    {
        return true;
    }

}  // namespace adaptive_layers

PLUGINLIB_EXPORT_CLASS(adaptive_layers::AudioLayer, nav2_costmap_2d::Layer)