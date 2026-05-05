#include "adaptive_layers/audio_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace adaptive_layers
{

    AudioLayer::AudioLayer() {}

    void AudioLayer::onInitialize()
    {
        auto node = node_.lock();  // lifecycle node
        RCLCPP_INFO(node->get_logger(), "AudioLayer initialized");
    }

    void AudioLayer::matchSize()
    {
        nav2_costmap_2d::CostmapLayer::matchSize();
    }

    void AudioLayer::updateBounds(double, double, double, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        // Update entire map
        *min_x = std::numeric_limits<double>::lowest();
        *min_y = std::numeric_limits<double>::lowest();
        *max_x = std::numeric_limits<double>::max();
        *max_y = std::numeric_limits<double>::max();
    }

    void AudioLayer::updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
    {
        for (int j = min_j; j < max_j; j++)
        {
            for (int i = min_i; i < max_i; i++)
            {
                // Audio: create a simple gradient
                uint8_t cost = (i + j) % 255;

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