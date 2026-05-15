#include "adaptive_layers/audio_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace adaptive_layers
{

    // uint8_t costValue(int i, int j)
    // {
    //     // Nav2 possible cost values
    //     // 0    :        Completely freeFully traversable
    //     // 1–252:        Increasing traversal cost
    //     // 253  :        Inside robot’s inscribed radius (Practically forbidden)
    //     // 254  :        Collision guaranteed
    //     // 255  :        Unknown
        
    //     return static_cast<uint8_t>(0.0 * (static_cast<double>(std::rand()) / RAND_MAX));
    // }


    AudioLayer::AudioLayer() {}

    void AudioLayer::onInitialize()
    {
        CostmapLayer::onInitialize();  // <-- REQUIRED
        auto node = node_.lock();  // lifecycle node
        logger_ = node->get_logger();
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + ".enabled", enabled_);
        declareParameter("param_1", rclcpp::ParameterValue(0.0));
        node->get_parameter(name_ + ".param_1", param_1_);
        
        audio_sub_ = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "~/audio_costs",
            rclcpp::QoS(1),
            std::bind(&AudioLayer::audioMapCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            logger_,
            "AudioLayer initialized, enabled=%s",
            enabled_ ? "true" : "false");


    }

    void AudioLayer::matchSize()
    {
        CostmapLayer::matchSize();
        
        std::lock_guard<std::mutex> lock(audio_mutex_);

        size_t size = getSizeInCellsX() * getSizeInCellsY();
        audio_costs_.assign(size, 0);  // default FREE_SPACE

    }


    void AudioLayer::updateBounds(double, double, double, double *min_x, double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_) {
            RCLCPP_WARN_ONCE(logger_, "Audio Layer is not enabled!!");
            return;
        }
        *min_x = std::min(*min_x, getOriginX());
        *min_y = std::min(*min_y, getOriginY());
        *max_x = std::max(
            *max_x,
            getOriginX() + getSizeInCellsX() * getResolution());
        *max_y = std::max(
            *max_y,
            getOriginY() + getSizeInCellsY() * getResolution());
    }

    void AudioLayer::updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
    {
        
        if (!enabled_) {
            RCLCPP_WARN_ONCE(logger_, "Audio Layer is not enabled!!");
            return;
        }
        
        std::lock_guard<std::mutex> lock(audio_mutex_);

        unsigned int size_x = getSizeInCellsX();

        for (int j = min_j; j < max_j; j++) {
            for (int i = min_i; i < max_i; i++) {
            unsigned int index = j * size_x + i;
            uint8_t cost = audio_costs_[index];
            setCost(i, j, cost);
            }
        }

        //RCLCPP_INFO(logger_,"Param_1=%f",param_1_);
        // Merge into master costmap
        // Use one of these:
            // updateWithMax()               Most common (obstacles dominate)
            // updateWithOverwrite()         Semantic or absolute layers
            // updateWithTrueOverwrite()     Full replacement (rare)
            // updateWithAddition()          Risk / preference accumulation

        
        updateWithMax(master, min_i, min_j, max_i, max_j);

    }
  
    void AudioLayer::reset()
    {
        // Can be empty for now
    }

    bool AudioLayer::isClearable()
    {
        return true;
    }

    
    void AudioLayer::audioMapCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(audio_mutex_);

        if (msg->data.size() != audio_costs_.size()) {
            RCLCPP_WARN_THROTTLE(
            logger_, *node_.lock()->get_clock(), 2000,
            "Audio cost array size mismatch: got %zu, expected %zu",
            msg->data.size(), audio_costs_.size());
            return;
        }

        audio_costs_ = msg->data;
    }


    

}  // namespace adaptive_layers

PLUGINLIB_EXPORT_CLASS(adaptive_layers::AudioLayer, nav2_costmap_2d::Layer)