//Code AWAKE 2025
#include <iostream>
#include <iostream>
#include <pigpio.h>
#include <unistd.h> // pour sleep()

//chmod +x install.sh
//./install.sh

int main() {
    const int GPIO_PIN = 18;          // GPIO18 supporte hardware PWM
    const int PWM_FREQ = 20000;       // 20 kHz
    const int DUTY_CYCLE = 500000;    // 50% (sur 1 000 000)

    if (gpioInitialise() < 0) {
        std::cerr << "Erreur d'initialisation de pigpio." << std::endl;
        return 1;
    }

    // PWM
    gpioHardwarePWM(GPIO_PIN, PWM_FREQ, DUTY_CYCLE);
    std::cout << "PWM actif sur GPIO " << GPIO_PIN << " à " << PWM_FREQ << " Hz" << std::endl;

    sleep(5);

    // STOP
    gpioHardwarePWM(GPIO_PIN, 0, 0);
    gpioTerminate();

    return 0;
}
