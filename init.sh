#!/bin/bash

# chmod +x init.sh
# ./init.sh

echo "Mise à jour du système"
sudo apt update && sudo apt upgrade -y 

echo "Installation des outils de base"
sudo apt install -y git build-essential

# Vérifie si gpio (wiringPi) est déjà installé
if ! command -v gpio &> /dev/null; then
  echo "📥 Installation de WiringPi depuis GitHub"
  git clone https://github.com/WiringPi/WiringPi.git
  cd WiringPi
  ./build
  cd ..
  rm -rf WiringPi
else
  echo "WiringPi déjà installé"
fi

echo "Test de WiringPi"
gpio -v
gpio readall