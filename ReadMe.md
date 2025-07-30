## 1. Initial Setup

1. Install docker from docs:
   https://docs.docker.com/engine/install/ubuntu/

2. Modifier les permissions pour avoir accès à docker sans sudo:

S'ajouter au groupe docker sinon pour ne pas mettre sudo devant toutes ses commandes sinon est une solution en trois étapes :

a) Créer le groupe docker (si ce n’est pas déjà fait)
```bash
sudo groupadd docker
```
b) Ajouter ton utilisateur au groupe docker
```bash
sudo usermod -aG docker $USER
```
c) Fermer la remote connection de WSL Ubuntu-22.04 avec le bouton en bas à gauche de VSCode pour redémarrer.

3. Clone this repository and navigate into it.

```bash
git clone https://github.com/zenith-polymtl/dockerfile_playground.git dockerfile_playground
cd dockerfile_playground
```

## 2. Running the Container

### Launch Docker

Start the container :

```bash
docker compose up
```

Start the container in detached mode :

```bash
docker compose up -d          # ajouter sudo en préfixe si fails sur commande  
```

### Stop Docker

Shut down and remove the container:

```bash
docker compose down
```

### Access the Container Shell

Open an interactive bash session inside the running container:

```bash
# Faire ça dans chaque autres terminaux que celui ou on a parti le docker!
docker exec -it dockerfile_playground-zenith-1 bash
#source install/setup.bash # Plus besoin à partir de maintenant

ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}" 

# possible de ros2 "run mission $NOM_NODE ensuite" :)


__"### Permission de fichier, par exemple "__ 
sudo chown -R avatar:avatar /home/avatar/dockerfile_playground/ros2_ws/install

__"### Publier 1 message sur un topic "__ 
ros2 topic pub /topic std_msgs/String 'data: Hello World' -1
ros2 topic pub /go_approach std_msgs/String 'data: 99,90,-15 ' -1
# ros2 topic pub /$NOMTOPIC $TYPE_MSG '$MSG' -1

#mapframe : global baselink : local drone (vitesse)
# commande local
#MavROS : east north up
#ZenMav : north east down
