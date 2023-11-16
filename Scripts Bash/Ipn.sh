#!/bin/bash
#Author : Marc CHAMBRE
# Allows you to loop "ipn" to find out when the turtlebot is accessible via its IP address.

while true; do
    # Exécute la commande 'ip n' et récupère le résultat complet
    result=$(ip n)

    # Affiche le résultat sur une seule ligne en écrasant le résultat précédent
    echo -ne "\r$result"

    # Vérifie si "REACHABLE" est présent dans le résultat
    if [[ $result == *"REACHABLE"* ]]; then
        echo -e "\nLa condition est atteinte. Arrêt du script."
        break
    fi

    # Attends 0.1 seconde avant la prochaine itération
    sleep 0.1
done
