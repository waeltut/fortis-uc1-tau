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

        // Matching QoS for init map
        rclcpp::QoS qos_init(rclcpp::KeepLast(1));
        qos_init.reliable();
        qos_init.transient_local();

        // Subsrciber
        init_audio_sub_ = node->create_subscription<fortis_interfaces::msg::MapCost>(
            "~/init_audio_costs",
            qos_init,
            std::bind(&AudioLayer::initAudioMapCallback, this, std::placeholders::_1));
        audio_sub_ = node->create_subscription<fortis_interfaces::msg::MapCost>(
            "~/audio_costs",
            1,
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

        if (!update_required_) {
            return;
        }

        if(need_resize){
            matchSize();
            return;
        }
        // Lock the modification of this layer and the master layer
        std::lock_guard<std::mutex> lock(audio_mutex_);

        // setting the whole map as a cost map for this layer while respecting other layers contribution
        double resolution = getResolution();
        *min_x = std::min(*min_x, new_min_i_*resolution);
        *min_y = std::min(*min_y, new_min_j_*resolution);
        *max_x = std::max(*max_x, (new_min_i_+new_width_) * resolution);
        *max_y = std::max(*max_y, (new_min_j_+new_height_) * resolution);

        // Audio has been modified
        update_required_ = false;
    }

    void AudioLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, 
                                 int min_i, int min_j, int max_i, int max_j){
        
        // Dont update the cost unless the layer is enabled and resized
        if (!enabled_ || need_resize) {
            RCLCPP_WARN_ONCE(logger_, "Audio Layer is not enabled or the bounds are not ready!!");
            return;
        }

        //RCLCPP_INFO(logger_, "min_i: %d, min_j: %d, max_i: %d, max_j: %d", min_i, min_j, max_i, max_j);

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

    void AudioLayer::audioMapCallback(const fortis_interfaces::msg::MapCost::SharedPtr msg){
        // This thing to check size
        /*if (msg->data.size() != audio_costs_.size()) {
            RCLCPP_WARN_THROTTLE(
            logger_, *node_.lock()->get_clock(), 2000,
            "Audio cost array size mismatch: got %zu, expected %zu",
            msg->data.size(), audio_costs_.size());
            need_resize = true;
            return;
        }*/

        // Lock :)
        std::lock_guard<std::mutex> lock(audio_mutex_);

        // Message-reading -hours
        //double resolution = getResolution();
        new_min_i_ = msg->i;
        new_min_j_ = msg->j;
        new_width_ = msg->width;
        new_height_ = msg->height;

        // Loopedy-loop
        for (int j = 0; j < new_height_; ++j) {
            for (int i = 0; i < new_width_; ++i) {
                uint8_t value = msg->data[i + j * new_width_];
                // Adding or removing
                if (value == 255) {    // Removing (want to avoid a rectangular remove)
                    audio_costs_[(i + new_min_i_) + (j + new_min_j_) * getSizeInCellsX()] = 0;
                } else if (value != 0) {    // Adding (just replace with new values)
                    audio_costs_[(i + new_min_i_) + (j + new_min_j_) * getSizeInCellsX()] = value;
                }
                
            }
        }
        update_required_ = true;
    }
    
    void AudioLayer::initAudioMapCallback(const fortis_interfaces::msg::MapCost::SharedPtr msg){
        if (map_init_) {
            return;
        }

        if (msg->data.size() != audio_costs_.size()) {
            RCLCPP_WARN_THROTTLE(
            logger_, *node_.lock()->get_clock(), 2000,
            "Audio cost array size mismatch: got %zu, expected %zu",
            msg->data.size(), audio_costs_.size());
            need_resize = true;
            return;
        }
        std::lock_guard<std::mutex> lock(audio_mutex_);

        // First update should update everything
        new_min_i_ = getOriginX();
        new_min_j_ = getOriginY();
        new_width_ = getSizeInCellsX();
        new_height_ = getSizeInCellsY();

        for (size_t i = 0; i < audio_costs_.size(); ++i) {
            audio_costs_[i] = msg->data[i];
        }
        //printCostHistogram(audio_costs_, "received cost");
        map_init_ = true;
        update_required_ = true;
    }
}  // namespace adaptive_layers
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(adaptive_layers::AudioLayer, nav2_costmap_2d::Layer)