#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class ControleNode : public rclcpp::Node {
public:
    ControleNode() : Node("controle_node") {
        moteur_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/Moteur", 10,
            std::bind(&ControleNode::moteur_callback, this, std::placeholders::_1)
        );

        direction_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/Direction", 10,
            std::bind(&ControleNode::direction_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Contrôle Node initialisé !");
    }

private:
    void moteur_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Moteur: %.2f", msg->data);
    }

    void direction_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Direction: %.2f", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moteur_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr direction_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControleNode>());
    rclcpp::shutdown();
    return 0;
}
