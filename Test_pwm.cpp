// #include <wiringPi.h>
// #include <softPwm.h>
// #include <stdio.h>

// #define PWM_PIN1 1
// #define PWM_PIN2 26
// #define PWM_PIN3 23

// int main(void) {
//     int angle;

//     wiringPiSetup();
//     softPwmCreate(PWM_PIN1, 0, 200);  // plage de 0 à 200 (20ms)

//     softPwmCreate(PWM_PIN2, 0, 200); //moteur

//     while (1) {
//         printf("Entrez un angle (0 à 180, -1 pour quitter) : ");
//         scanf("%d", &angle);

//         if (angle == -1)
//             break;

//         if (angle < 30 || angle > 110) {
//             printf("Angle invalide.\n");
//             continue;
//         }

//         // Convertit l'angle en impulsion (entre 5 et 25 pour 0 à 180°)
//         int pulse = (angle * 20 / 180) + 5;

//         softPwmWrite(PWM_PIN1, pulse);
//         softPwmWrite(PWM_PIN2, pulse);

//         printf("Impulsion PWM : %d (pour %d°)\n", pulse, angle);
//         delay(500);
//     }

//     softPwmWrite(PWM_PIN1, 0);
//     softPwmWrite(PWM_PIN2, 0);
//     printf("Fin du programme\n");

//     return 0;
// }

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

#define SERVO_PIN 1      // Pin pour le servo moteur
#define ESC_PIN   26     // Pin pour l'ESC
#define PWM_RANGE 200    // softPwm plage : 0 à 200 = 20ms

int main(void) {
    int angle, power;

    wiringPiSetup();
    softPwmCreate(SERVO_PIN, 0, PWM_RANGE);
    softPwmCreate(ESC_PIN, 0, PWM_RANGE);

    printf("Contrôle d'un servo et d'un moteur via ESC.\n");

    while (1) {
        printf("\nEntrez l'angle servo (30 à 110) et la puissance moteur (0 à 100), ou -1 pour quitter : ");
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

        if (power < 0 || power > 100) {
            printf("⚠️ Puissance invalide. Utilisez entre 0 et 100.\n");
            continue;
        }

        int servoPulse = (angle * 20 / 180) + 5;         // pour servo
        int escPulse = (power * 20 / 100) + 5;           // pour ESC

        softPwmWrite(SERVO_PIN, servoPulse);
        softPwmWrite(ESC_PIN, power);

        printf("Servo : %d° → %d / ESC : %d%% → %d\n", angle, servoPulse, power, escPulse);

        delay(500); // temps pour que les composants réagissent
    }

    softPwmWrite(SERVO_PIN, 0);
    softPwmWrite(ESC_PIN, 0);
    printf("Fin du programme. PWM désactivé.\n");

    return 0;
}
