#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cstdlib> // Pour system()

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <signal.h>
#include <csignal>

#define SERVO_PIN 1       //PIN physique 12 
#define ESC_PIN   26      //PIN physique 32
#define PWM_RANGE 2000   // 10µs precision: 19.2 MHz / 192 = 100 kHz, 100 kHz / 2000 = 50 Hz
#define NEUTRE 150       // 1.5 ms (standard neutral)
#define V_MAX 164        // 2.0 ms (full throttle)
#define V_MIN 160        // 1.0 ms 
// Servo range: 7 (Min) <-> 11 (Centre) <-> 16 (Max) with 200 range (20ms @ 50Hz)
#define SERVO_MIN 7      // Min servo position
#define SERVO_CENTER 11  // Center servo position
#define SERVO_MAX 16     // Max servo position
#define SERVO_RANGE 200  // Software PWM range for servo (0.1ms per step, 20ms total)

// Forward declaration
class ControleNode;

void signal_handler(int signum) {
    if (signum == SIGINT) {
        printf("\n[Signal Handler] Arrêt du moteur et du servo...\n");
        // Return to neutral positions immediately
        softPwmWrite(SERVO_PIN, SERVO_CENTER);  // Center servo
        pwmWrite(ESC_PIN, NEUTRE);              // Neutral ESC
        printf("[Signal Handler] Moteur et servo arrêtés !\n");
        exit(0);
    }
}


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

        // --- AJOUT : Force la configuration du PIN 12 (BCM) en mode PWM ---
        // Le flag -g indique qu'on parle du numéro GPIO BCM (12), pas wiringPi (26)
        // Force la configuration (avec chemin absolu)
        int res = system("/usr/bin/gpio -g mode 12 pwm");
        if (res != 0) {
            RCLCPP_ERROR(this->get_logger(), "Erreur lors de l'exécution de la commande gpio !");
        }

        // setup PWM
        // wiringPiSetup() retourne -1 en cas d'erreur
        if (wiringPiSetup() == -1) {
            RCLCPP_FATAL(this->get_logger(), "Échec de l'initialisation wiringPi ! Êtes-vous root ?");
            exit(1);
        }

        pinMode (ESC_PIN, PWM_OUTPUT); 
        pwmSetMode(PWM_MODE_MS);      
        pwmSetRange(PWM_RANGE);       
        pwmSetClock(192);   // For 50 Hz frequency
        
        pwmWrite (ESC_PIN, NEUTRE);

        // Initialize servo with software PWM (range 200 * 0.1ms = 20ms for 50Hz)
        softPwmCreate(SERVO_PIN, SERVO_CENTER, SERVO_RANGE);
        
        // Initialize servo to center position
        softPwmWrite(SERVO_PIN, SERVO_CENTER);

        RCLCPP_INFO(this->get_logger(), "Contrôle Node initialisé !");
        RCLCPP_INFO(this->get_logger(), "Servo range: %d (min) to %d (max), Center: %d", SERVO_MIN, SERVO_MAX, SERVO_CENTER);
    }

    ~ControleNode() override {
        // Return to neutral position before shutdown
        softPwmWrite(SERVO_PIN, SERVO_CENTER);  // Center servo position
        pwmWrite(ESC_PIN, NEUTRE);              // Neutral ESC position
        delay(500);
        RCLCPP_INFO(this->get_logger(), "PWM remis à position neutre (arrêt du node)");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    void moteur_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        moteur_PWM_ = msg->data; // valeur entre -1 et 1
        if(moteur_PWM_<=-1) { // si -1, on coupe
            moteur_PWM_ = NEUTRE;
        }else{ 
            moteur_PWM_ = mapValue(static_cast<int>(moteur_PWM_*100) , 0, 100, V_MIN, V_MAX);
        } 
        RCLCPP_INFO(this->get_logger(), "Moteur: %d", static_cast<int>(moteur_PWM_));
    }

    void direction_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // Convert from -1 to 1 range to servo range (7-16)
        float input_value = msg->data;
        
        // Clamp input to [-1, 1]
        if (input_value < -1.0f) input_value = -1.0f;
        if (input_value > 1.0f) input_value = 1.0f;
        
        // Map to servo range: -1 -> 7 (min), 0 -> 11 (center), 1 -> 16 (max)
        servo_moteur_PWM_ = mapValue(static_cast<int>(input_value * 100), -100, 100, SERVO_MIN, SERVO_MAX);
        
        RCLCPP_INFO(this->get_logger(), "Direction: %d (from input: %.2f)", static_cast<int>(servo_moteur_PWM_), input_value);
    }

    void update_pwm() {
        softPwmWrite(SERVO_PIN, static_cast<int>(servo_moteur_PWM_));
        pwmWrite (ESC_PIN, static_cast<int>(moteur_PWM_));
    }

    _Float32 moteur_PWM_ = NEUTRE;
    _Float32 servo_moteur_PWM_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moteur_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr direction_sub_;
};

int main(int argc, char * argv[]) {
    // Enregistrer le gestionnaire de signal SIGINT
    signal(SIGINT, signal_handler);
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControleNode>());
    rclcpp::shutdown();
    return 0;
}
