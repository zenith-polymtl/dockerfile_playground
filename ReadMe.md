# Dockerfile Playground

## 1. Configuration Initiale

### 1.1 Installation de Docker

Installer Docker en suivant la documentation officielle :
https://docs.docker.com/engine/install/ubuntu/

### 1.2 Configuration des Permissions Docker

Pour éviter d'utiliser `sudo` devant chaque commande Docker, ajoutez votre utilisateur au groupe docker :

```bash
# Créer le groupe docker (s'il n'existe pas déjà)
sudo groupadd docker

# Ajouter votre utilisateur au groupe docker
sudo usermod -aG docker $USER

# Redémarrer votre session WSL Ubuntu-22.04
# Fermez la connexion distante de VSCode et reconnectez-vous
```

### 1.3 Cloner le repo

```bash
git clone https://github.com/zenith-polymtl/dockerfile_playground.git dockerfile_playground
cd dockerfile_playground
```

## 2. Lancement du Docker

### Démarrer le Docker

```bash
# Mode interactif
docker compose up

# Mode détaché (en arrière-plan)
docker compose up -d

# Mode faut rebuilt après modif de dépendances
docker compose up --build
```

### Arrêter le Conteneur

```bash
docker compose down     # ou crtl+c dans le terminal si rouler docker compose up initialement
```

## 3. Accès au Conteneur

Pour ouvrir une session bash interactive dans le conteneur en cours d'exécution :

```bash
# Exécuter cette commande dans tous les terminaux différents de celui utilisé pour lancer Docker
docker exec -it dockerfile_playground-zenith-1 bash
```

## 4. Commandes Utiles

### Appel de Service ROS2

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}" 
# avant de lancer certains echo par exemple 
```

### Lancement de Nœuds ROS2

```bash
ros2 run mission $NOM_NODE
```

### Lancement de lauchfile ROS2

```bash
ros2 launch start_mission start.pi3.lauch.py 
```

### Changement dans le docker

```bash
colcon build    
# rentrer cette commande avant de relancer un node modifié ou un launch file
```

### Publication de Messages

```bash
# Format général
ros2 topic pub /$NOM_TOPIC $TYPE_MSG '$MESSAGE' -1

# Exemples
ros2 topic pub /topic std_msgs/String 'data: Hello World' -1
ros2 topic pub /go_approach std_msgs/String 'data: 99,90,-15' -1
```

### Gestion des Permissions de Fichiers

```bash
sudo chown -R avatar:avatar /home/avatar/dockerfile_playground/ros2_ws/install
```

### Pour supprimer ou créer des fichiers sans avoir les permissions : 

```bash
sudo rm -rf /home/avatar/dockerfile_playground/ros2_ws/data/*       # clear csv dans data par exemple
sudo touch /home/avatar/dockerfile_playground/ros2_ws/log/COLCON_IGNORE  # recrée le fichier COLCON_IGNORE après avoir delete les logs du dossier log
```

### Partir le fichier python de graph : 

```bash
python3 graph_crea_xyz.py             # Dans un terminal hors docker, ex : avatar@PcLaurent:~$
```

### Fermer une node : 

```bash
pkill -f abort_brake_node             # Forcer la fermeture d'une node
```

## 5. Notes Importantes

### Systèmes de Coordonnées

- **mapframe**: Référentiel global
- **baselink**: Référentiel local du drone (vitesse)
- **MavROS**: Système de coordonnées East-North-Up (ENU)
- **Zenmav**: Système de coordonnées North-East-Down (NED)

### Commandes de Contrôle

Les commandes locales sont envoyées en utilisant le système de coordonnées approprié pour chaque système.

### Before launching

Avant de lancer tous l'environnement ros2 et docker : 
    Dans Mission Planner
--> choisir le copteur
--> Simu se lance
--> Mettre en guided à la troisième ligne
--> Attendre que le GPS se set
--> ARM
--> Takeoff avec clique droit

## 6. Suggestion de terminaux

### Liste : 

Pas oublier de rouler le exec en point 3. dans ch. terminaux avant de rouler les commandes si dessous (sauf le compose)

```bash
docker compose up   #1.1
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}"   #1.2
ros2 topic echo /mavros/local_position/pose #2.1
ros2 launch start_mission start.pi3.lauch.py    #2.2
python3 graph_crea_pos_yaw.py   #3
```

## 7. Test de vol

ON OFA (connect to host) : 
i. Docker up
ii. Message request
iii. Lauch approach file except control node

ON UBUNTU : 
i. Docker up
ii. ros2 run control node
iii. begin approach or brake while drone took off

ON MP :
i. Modif paramètres
ii. Take off when armed by pilot

## 8. Questions ?
