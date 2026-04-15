#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

// PIN 1 (WiringPi) = GPIO 18 (Physique 12) -> SERVO
#define SERVO_PIN 1      
// PIN 26 (WiringPi) = GPIO 12 (Physique 32) -> ESC
#define ESC_PIN   26     

// Range pour l'ESC (Hardware PWM) pour avoir une précision parfaite
#define PWM_RANGE 2000    

int main(void) {
    int angleInput, pwmValue;

    if (wiringPiSetup() == -1) {
        printf("Erreur d'initialisation wiringPi !\n");
        return 1;
    }

    // --- 1. CONFIGURATION ESC (HARDWARE PWM - Précis) ---
    pinMode(ESC_PIN, PWM_OUTPUT); 
    pwmSetMode(PWM_MODE_MS);      
    pwmSetClock(192);             
    pwmSetRange(PWM_RANGE);       

    // Armement ESC
    printf("Armement de l'ESC (Neutre 150)...\n");
    pwmWrite(ESC_PIN, 150); 
    delay(3000); 
    printf("ESC prêt.\n");

    // --- 2. CONFIGURATION SERVO (SOFTWARE PWM - Basique) ---
    // Range 200 * 0.1ms = 20ms (50Hz standard)
    softPwmCreate(SERVO_PIN, 0, 200); 

    while (1) {
        printf("\n--- CONTRÔLE (PLAGE CONTINUE) ---\n");
        printf("Servo : 7 (Min) <-> 11 (Centre) <-> 16 (Max)\n");
        printf("ESC   : 100 (Frein) <-> 150 (Neutre) <-> 200 (Gaz)\n");
        printf("Format : servo puissance (ex: 15 155) > ");
        
        int result = scanf("%d %d", &angleInput, &pwmValue);

        if (result != 2) {
            while (getchar() != '\n'); 
            continue;
        }

        if (angleInput == -1 || pwmValue == -1) break;

        // --- CONVERSION SERVO (PLAGE CONTINUE 7-16) ---
        
        // Sécurité bornes
        if (angleInput < 7) angleInput = 7;
        if (angleInput > 16) angleInput = 16;

        int servoCommand = angleInput;

        // --- SÉCURITÉ ESC ---
        if (pwmValue < 100) pwmValue = 100;
        if (pwmValue > 200) pwmValue = 200;

        printf("Commande -> Servo: %d | ESC: %d\n", servoCommand, pwmValue);

        softPwmWrite(SERVO_PIN, servoCommand);
        pwmWrite(ESC_PIN, pwmValue);
    }

    // Arrêt propre
    printf("Arrêt : Retour au neutre.\n");
    pwmWrite(ESC_PIN, 150);
    softPwmWrite(SERVO_PIN, 0); // Coupe le signal servo
    delay(1000);

    return 0;
}