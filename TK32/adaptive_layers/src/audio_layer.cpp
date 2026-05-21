#include "adaptive_layers/audio_layer.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace adaptive_layers{

    void printCostHistogram(const std::vector<uint8_t>& costs, const std::string& name){
        std::map<unsigned char, int> hist;
        for (int i = 0; i < (int)costs.size(); ++i) {
            hist[costs[i]]++;
        }
        std::cout << "\nHistogram: " << name << std::endl;
        for (const auto& kv : hist)
            std::cout << "  cost " << (int)kv.first << " : " << kv.second << std::endl;
    }

    AudioLayer::AudioLayer(){}

    void AudioLayer::onInitialize(){
        auto node = node_.lock();  // lifecycle node
        declareParameter("enabled", rclcpp::ParameterValue(true));
        node->get_parameter(name_ + ".enabled", enabled_);
        declareParameter("param_1", rclcpp::ParameterValue(0.0));
        node->get_parameter(name_ + ".param_1", param_1_);

        need_resize = true;
        
        matchSize();

        audio_sub_ = node->create_subscription<std_msgs::msg::UInt8MultiArray>(
            "~/audio_costs",
            rclcpp::QoS(1),
            std::bind(&AudioLayer::audioMapCallback, this, std::placeholders::_1));
        //layerSizePub_ = this->create_publisher<std_msgs::msg::Float32>("my_topic", 10);
        RCLCPP_INFO(logger_, "AudioLayer initialized, enabled=%s", enabled_ ? "true" : "false");
    }

    void AudioLayer::matchSize(){
        need_resize = true;
        CostmapLayer::matchSize();
        resetMaps();      
        unsigned int size_x = getSizeInCellsX();
        unsigned int size_y = getSizeInCellsY();
        RCLCPP_INFO(
            logger_,
            "AudioLayer resized to: %u x %u",
            size_x,
            size_x);
        std::lock_guard<std::mutex> lock(audio_mutex_);
        audio_costs_.assign(size_x * size_y, 0);  // default FREE_SPACE
        need_resize = false;
        // TODO: Publish msg to the costmaps manager 

    }

    void AudioLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, 
                                  double *min_x, double *min_y, double *max_x, double *max_y){
        if (!enabled_) {
            RCLCPP_WARN_ONCE(logger_, "Audio Layer is not enabled!!");
            return;
        }

        if(need_resize){
            matchSize();
            return;
        }
        // Lock the modification of this layer and the master layer
        std::lock_guard<std::mutex> lock(audio_mutex_);

        // setting the whole map as a cost map for this layer while respecting other layers contribution
        *min_x = std::min(*min_x, getOriginX());
        *min_y = std::min(*min_y, getOriginY());
        *max_x = std::max(*max_x, getOriginX() + getSizeInCellsX() * getResolution());
        *max_y = std::max(*max_y, getOriginY() + getSizeInCellsY() * getResolution());
    }

    void AudioLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, 
                                 int min_i, int min_j, int max_i, int max_j){
        
        // Dont update the cost unless the layer is enabled and resized
        if (!enabled_ || need_resize) {
            RCLCPP_WARN_ONCE(logger_, "Audio Layer is not enabled or the bounds are not ready!!");
            return;
        }

        // Lock the modification of this layer and the master layer
        std::lock_guard<std::mutex> lock(audio_mutex_);

        // Get the local (audio) costmap and sizes
        unsigned char * map = getCharMap();
        unsigned int size = getSizeInCellsX() * getSizeInCellsY();

        // Update the local (audio) costmap
        for (unsigned int i = 0; i < size; ++i) {
            map[i] = audio_costs_[i];   // value in [0..252]
        }

        // Merge into master costmap. Use one of these:
            // updateWithMax()               Most common (obstacles dominate)
            // updateWithOverwrite()         Semantic or absolute layers
            // updateWithTrueOverwrite()     Full replacement (rare)
            // updateWithAddition()          Risk / preference accumulation
        updateWithMax(master_grid, min_i, min_j, max_i, max_j);

        // informing the master layer that the audio layer has new values.
        current_ = true;
    }

    void AudioLayer::reset(){
        RCLCPP_WARN_ONCE(logger_, "Audio Layer has been reset!!");
        std::lock_guard<std::mutex> lock(audio_mutex_);
        resetMaps();
        unsigned char * map = getCharMap();
        memset(map, 0, getSizeInCellsX() * getSizeInCellsY());
        need_resize = true;
    }
    
    void AudioLayer::audioMapCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg){
        if (msg->data.size() != audio_costs_.size()) {
            RCLCPP_WARN_THROTTLE(
            logger_, *node_.lock()->get_clock(), 2000,
            "Audio cost array size mismatch: got %zu, expected %zu",
            msg->data.size(), audio_costs_.size());
            need_resize = true;
            return;
        }
        std::lock_guard<std::mutex> lock(audio_mutex_);

        for (size_t i = 0; i < audio_costs_.size(); ++i) {
            audio_costs_[i] = msg->data[i];
        }
        //printCostHistogram(audio_costs_, "received cost");
    }
}  // namespace adaptive_layers
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(adaptive_layers::AudioLayer, nav2_costmap_2d::Layer)