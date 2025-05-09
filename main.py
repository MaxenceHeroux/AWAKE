# Code AWAKE 2025 en Python
import pigpio
import time

GPIO_PIN = 18         # GPIO18 supporte le PWM hardware
PWM_FREQ = 20000      # 20 kHz
DUTY_CYCLE = 500000   # 50% de 1 000 000

# Connexion au daemon pigpio
pi = pigpio.pi()
if not pi.connected:
    print("Erreur : impossible de se connecter à pigpiod.")
    exit(1)

# Activer le PWM
pi.hardware_PWM(GPIO_PIN, PWM_FREQ, DUTY_CYCLE)
print(f"PWM actif sur GPIO {GPIO_PIN} à {PWM_FREQ} Hz")

time.sleep(5)  # Attendre 5 secondes

# Arrêter le PWM
pi.hardware_PWM(GPIO_PIN, 0, 0)
pi.stop()

#sudo apt install pigpio python3-pigpio
#sudo pigpio
#python3 ton_fichier.py