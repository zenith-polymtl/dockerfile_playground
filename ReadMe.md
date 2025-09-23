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
"""
ros2 topic pub --once /estimated_target_location geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w:
 1.0}}}'
 """
"""
ros2 topic pub --once /goal_pose_polar custom_interfaces/msg/TargetPosePolar \
"{r: 4.0, z: 5.0, theta: -1.2, relative: true, v_theta: 0.5, v_r: 1.0}"
"""


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

### Request de publication de messages

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}"   #1.2 si utile finalement
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

Pas oublier de rouler #A dans ch. terminaux avant de rouler les commandes si dessous (sauf le compose)

```bash
docker exec -it dockerfile_playground-zenith-1 bash  #A
docker compose up   #1.1
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}" #1.2
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

## 9. Planning

--> yaw non harcode, vérif yaw convention for whole pipe, (l.62 de graph_node et targets circle de target_publisher_node)

--> control with pos/vel/acc for smoothness

--> Lire doc caméra pour inté

# Utilisation de manual web control interface

Lors du colcon build, le node et l'interface sont construit à l'aide des templates.j2 et config.json dans mission/control_interfaces et de setup.py.

***IL EST DONC INUTILE DE MODIFIER DIRECTEMENT LE NODE web_manual_control_node.py***

Le fichier à modifier est donc config.json dans control_interfaces pour des modifications d'interfaces. Les subscribers n'ont pas été tester à l'instant mais devraient fonctionner.

## Customizing de l'interface avec le config.json

Il y a deux catégories primaires : sidebar et control_sections.

### sidebar
La sidebar va contenir title et un array buttons qui va contenir tous les boutons de celle-ci.

    "sidebar": {
        "title": "System Control",
        "buttons": [
        {
            "text": "Battery Changed",
            "class": "battery-btn",
            "command": "/battery_changed",
            "topicType": "String",
            "qosProfile": false,
            "data": "CHANGED"
        },
        {
            "text": "EMERGENCY ABORT",
            "class": "abort-btn",
            "command": "/abort_brake",
            "topicType": "String",
            "qosProfile": true,
            "data": "a.b."
        },
        {
            "text": "ARM & AUTHORIZE TAKEOFF",
            "class": "arm-btn",
            "topicType": "String",
            "qosProfile": false,
            "command": "/confirm_arming",
            "data": "ARM"
        }
        ]
    },

### control_sections
control_sections représente un array de sections, soit les carré visibles dans l'interface. Chaque objet de control_sections va avoir les attributs suivant title et un choix simple ou multiple de buttons, inputs, radio et sensors. Ces derniers sont des array avec des boutons, inputs, sensors et boutons radio (sélectionner un de ces boutons déselectionne l'autre).

    "control_sections": [
        {
        "title": "Vision Control",
        "buttons": [
                {
                "text": "Source Search",
                "topicType": "String",
                "qosProfile": false,
                "command": "/go_vision",
                "data": "SOURCE"
                },
                {
                "text": "Bucket Search",
                "topicType": "String",
                "qosProfile": false,
                "command": "/go_vision",
                "data": "BUCKET"
                }
            ]
        },
    ],
        
title est le titre de la section de l'interface

## Pour ce qui des boutons, inputs, radio et sensors:

### Boutons: 
Ce sont des boutons normaux qui send un message au topic. Ici,

text représente le texte écris sur le bouton, 
topicType représente le type de message (String, Int32, etc.), 
qosProfile ajoute l'option d'utiliser qos ou non avec un boolean,
command représente le topic et
data représente le message à envoyer 

    {
        "text": "Source Search",
        "topicType": "String",
        "qosProfile": false,
        "command": "/go_vision",
        "data": "SOURCE"
    },

### Radio
Ils sont pareil que les boutons mais si on est actif, les autres sont inactifs. Il s'agit plus d'une modification visuel que technique.

### Inputs
C'est un selecteur de valeur de type integer uniquement.
type sert de dire qu'il s'agit de nombres (va possiblement rajouter d'autre types plus tard)

id est utilisé pour l'interface pour avoir accès à la valeur DOIT ÊTRE UNIQUE
topicType représente le type de message (String, Int32, etc.), 
qosProfile ajoute l'option d'utiliser qos ou non avec un boolean,
command représente le topic et
data représente le message à envoyer
min est la valeur minimale permissible
value est la valeur par défaut
label est le texte sur l'interface qui identifie la section

    {
        "type": "number",
        "id": "bucketNumber",
        "command": "/bucket_number",
        "topicType": "Int32",
        "qosProfile": false,
        "min": "1",
        "value": "1",
        "label": "Buckets:"
    }
### Sensors (subscribers)
***UNTESTED AT THE MOMENT***

Les objets sensors fonctionne similairement à ceux de inputs :

id est utilisé pour l'interface pour avoir accès à la valeur DOIT ÊTRE UNIQUE
topicType représente le type de message (String, Int32, etc.), 
qosProfile ajoute l'option d'utiliser qos ou non avec un boolean,
command représente le topic et
value est la valeur par défaut
label est le texte sur l'interface qui identifie la section

    {
        "id": "torque",
        "label": "Motor Torque",
        "command": "/torque",
        "topicType": "Float32",
        "qosProfile": false,
        "value": "0.00 Nm"
    }
