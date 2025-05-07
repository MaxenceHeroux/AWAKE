#!/bin/bash

echo "🛠️ Mise à jour du système..."
sudo apt update && sudo apt upgrade -y

echo "Installation de pigpio..."
sudo apt install -y pigpio

echo "pigpio installé !"

echo "🚀 Démarrage du service pigpiod..."
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo "Service pigpiod actif. Installation terminée."