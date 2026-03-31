#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define SERVO_PIN 26       //PIN physique 12 
#define ESC_PIN   1      //PIN physique 32
#define PWM_RANGE 255    
#define NEUTRE 128
#define V_MAX 180 //MAXMAMX 220
#define V_MIN NEUTRE+35


int mapValue(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class ControleNode : public rclcpp::Node {
public:
    ControleNode() : Node("controle_node"){
        moteur_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/Moteur", 10,
            std::bind(&ControleNode::moteur_callback, this, std::placeholders::_1)
        );

        direction_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/Direction", 10,
            std::bind(&ControleNode::direction_callback, this, std::placeholders::_1)
        );

        // Appeler update_pwm toutes les 100 ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControleNode::update_pwm, this)
        );

        //setup PWM
        wiringPiSetup();
        pinMode (ESC_PIN, PWM_OUTPUT); 
        pwmSetMode(PWM_MODE_MS);      
        pwmSetRange(PWM_RANGE);       
        pwmSetClock(153);   
        
        pwmWrite (ESC_PIN, 128);
        delay(1000);

        softPwmCreate(SERVO_PIN, 0, PWM_RANGE);

        RCLCPP_INFO(this->get_logger(), "Contrôle Node initialisé !");
    }

    ~ControleNode() override {
        // Remettre les PWM à zéro à la fermeture
        softPwmWrite(SERVO_PIN, 127);
        pwmWrite(ESC_PIN, 60);
        RCLCPP_INFO(this->get_logger(), "PWM remis à zéro (arrêt du node)");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    void moteur_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        moteur_PWM_ = msg->data; // valeur entre -1 et 1
        if(moteur_PWM_<=-1) { // si -1, on coupe
            moteur_PWM_ = NEUTRE-15;
        }else{ 
            moteur_PWM_ = mapValue(static_cast<int>(moteur_PWM_*100) , 0, 100, V_MIN, V_MAX);
        } 
        RCLCPP_INFO(this->get_logger(), "Moteur: %d", static_cast<int>(moteur_PWM_));
    }

    void direction_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        servo_moteur_PWM_ = msg->data; //-1 et 1
        servo_moteur_PWM_ = mapValue(static_cast<int>(servo_moteur_PWM_*100), -100, 100, 30, 90);
        RCLCPP_INFO(this->get_logger(), "Direction: %d", static_cast<int>(servo_moteur_PWM_));
    }

    void update_pwm() {
        int servoPulse = (servo_moteur_PWM_ * 20 / 180) + 5; 
        softPwmWrite(SERVO_PIN, servoPulse);
        pwmWrite (ESC_PIN, moteur_PWM_);
    }

    _Float32 moteur_PWM_ = NEUTRE;
    _Float32 servo_moteur_PWM_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moteur_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr direction_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControleNode>());
    rclcpp::shutdown();
    return 0;
}
