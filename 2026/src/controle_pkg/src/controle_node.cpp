#include "rclcpp/rclcpp.hpp"
// Inclusion du message Ackermann standard
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <algorithm>

#define SERVO_PIN 26       // PIN physique 12 
#define ESC_PIN   1        // PIN physique 32
#define PWM_RANGE 255    
#define NEUTRE 128
#define V_MAX 160          // MAXIMUM 220
#define V_MIN (NEUTRE + 20)

// Utilisation de floats pour le mapping afin de ne pas perdre la précision des radians/vitesses
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

class ControleNode : public rclcpp::Node {
public:
    ControleNode() : Node("controle_node") {
        
        // --- Configuration de la QoS (Strictement identique au Publisher Python) ---
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.transient_local();
        qos.reliable();

        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/ackermann_cmd", qos,
            std::bind(&ControleNode::ackermann_callback, this, std::placeholders::_1)
        );

        // Appeler update_pwm toutes les 100 ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControleNode::update_pwm, this)
        );

        // Setup PWM
        wiringPiSetup();
        pinMode(ESC_PIN, PWM_OUTPUT); 
        pwmSetMode(PWM_MODE_MS);      
        pwmSetRange(PWM_RANGE);       
        pwmSetClock(153);   
        
        pwmWrite(ESC_PIN, NEUTRE);
        delay(1000);

        softPwmCreate(SERVO_PIN, 60, PWM_RANGE); // Initialisation au centre (60)

        RCLCPP_INFO(this->get_logger(), "Contrôle Node Ackermann Initialisé ! En attente de /ackermann_cmd...");
    }

    ~ControleNode() override {
        // Sécurité à la fermeture du nœud
        softPwmWrite(SERVO_PIN, 60);
        pwmWrite(ESC_PIN, 60); 
        RCLCPP_INFO(this->get_logger(), "PWM remis à zéro (arrêt du node)");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;

    int moteur_PWM_ = NEUTRE;
    int servo_moteur_PWM_ = 60; // Valeur intermédiaire par défaut (Centre)

    void ackermann_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        float speed = msg->drive.speed;               // Vitesse en m/s
        float steering_angle = msg->drive.steering_angle; // Angle en radians (+ = Gauche, - = Droite)

        // --- 1. Traitement de la Vitesse (ESC) ---
        if (speed <= 0.0f) {
            moteur_PWM_ = NEUTRE; // Arrêt total si consigne nulle ou négative
        } else {
            // On mappe la vitesse (0.0 à 0.6 m/s du script de course) vers la plage PWM moteur (V_MIN à V_MAX)
            moteur_PWM_ = static_cast<int>(mapFloat(speed, 0.0f, 0.6f, static_cast<float>(V_MIN), static_cast<float>(V_MAX)));
            moteur_PWM_ = std::clamp(moteur_PWM_, V_MIN, V_MAX);
        }

        // --- 2. Traitement de la Direction (Servo) ---
        // Ton code Python limite le braquage à max_steering = 0.61 rad
        // On mappe cette plage [-0.61, 0.61] vers ton ancienne échelle [10, 110] pour préserver ta formule de pulse
        servo_moteur_PWM_ = static_cast<int>(mapFloat(steering_angle, -0.61f, 0.61f, 10.0f, 110.0f));
        servo_moteur_PWM_ = std::clamp(servo_moteur_PWM_, 10, 110);

        RCLCPP_INFO(this->get_logger(), "Ackermann -> Vitesse: %.2f m/s (%d PWM) | Angle: %.2f rad (%d)", 
                    speed, moteur_PWM_, steering_angle, servo_moteur_PWM_);
    }

    void update_pwm() {
        // Application directe de ta formule de conversion de pulse de servo
        int servoPulse = (servo_moteur_PWM_ * 20 / 180) + 5; 
        softPwmWrite(SERVO_PIN, servoPulse);
        pwmWrite(ESC_PIN, moteur_PWM_);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControleNode>());
    rclcpp::shutdown();
    return 0;
}