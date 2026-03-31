#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define SERVO_PIN 1       //PIN physique 12 
#define ESC_PIN   26     //PIN physique 32
#define PWM_RANGE 255    

int main(void) {
    int angle, power;

    wiringPiSetup();

    // Pour régler la fréquence PWM sur le canal matériel (ESC_PIN)
    // wiringPi PWM hardware: fréquence = 19.2MHz / (clock * range)
    pinMode (ESC_PIN, PWM_OUTPUT); // Doit être appelé avant les réglages PWM
    pwmSetMode(PWM_MODE_MS);      // Mode Mark-Space recommandé pour ESC/servo
    pwmSetRange(PWM_RANGE);       // PWM_RANGE = 255
    pwmSetClock(153);             // 19.2MHz / (192 * 255) ≈ 392 Hz (adapter selon besoin)

    softPwmCreate(SERVO_PIN, 0, PWM_RANGE);    

    printf("Contrôle d'un servo et d'un moteur via ESC.\n");

    while (1) {
        printf("\nEntrez l'angle servo (30 à 110) et la puissance moteur (0 à 255), ou -1 pour quitter : ");
        printf("\nFormat : angle puissance → ex: 60 40\n> ");

        int result = scanf("%d %d", &angle, &power);

        if (result != 2) {
            printf("Entrée invalide. Essayez à nouveau.\n");
            while (getchar() != '\n'); // Vider le buffer
            continue;
        }

        if (angle == -1 || power == -1)
            break;

        if (angle < 30 || angle > 110) {
            printf("⚠️ Angle invalide. Utilisez entre 30 et 110.\n");
            continue;
        }

        if (power < 0 || power > 255) {
            printf("⚠️ Puissance invalide. Utilisez entre 0 et 255.\n");
            continue;
        }

        int servoPulse = (angle * 20 / 180) + 5;         // pour servo

        softPwmWrite(SERVO_PIN, servoPulse);
        // softPwmWrite(ESC_PIN, power);
        pwmWrite (ESC_PIN, power);

        delay(500); // temps pour que les composants réagissent
    }

    softPwmWrite(SERVO_PIN, 0);
    // softPwmWrite(ESC_PIN, 0);
    pwmWrite (ESC_PIN, 0);
    printf("Fin du programme. PWM désactivé.\n");

    return 0;
}
