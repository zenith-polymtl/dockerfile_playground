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
```

### Arrêter le Conteneur

```bash
docker compose down
```

## 3. Accès au Conteneur

Pour ouvrir une session bash interactive dans le conteneur en cours d'exécution :

```bash
# Exécuter cette commande dans un terminal différent de celui utilisé pour lancer Docker
docker exec -it dockerfile_playground-zenith-1 bash
```

## 4. Commandes Utiles

### Appel de Service ROS2

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}"
```

### Lancement de Nœuds ROS2

```bash
ros2 run mission $NOM_NODE
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

## 5. Notes Importantes

### Systèmes de Coordonnées

- **mapframe**: Référentiel global
- **baselink**: Référentiel local du drone (vitesse)
- **MavROS**: Système de coordonnées East-North-Up (ENU)
- **Zenmav**: Système de coordonnées North-East-Down (NED)

### Commandes de Contrôle

Les commandes locales sont envoyées en utilisant le système de coordonnées approprié pour chaque système.
