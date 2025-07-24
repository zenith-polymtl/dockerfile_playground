## 1. Initial Setup

1. Clone this repository and navigate into it.

2. Make the setup script executable and run it:

   ```bash
   chmod +x setup.sh
   ./setup.sh
   ```

   This will install any required dependencies on the host.




## 2. Running the Container

### Launch Docker

Start the container :

```bash
docker compose up
```

Start the container in detached mode :

```bash
docker compose up -d          # ajouter sudo en préfixe si fails sur commande  

"""
# S'ajouter au groupe docker sinon pour ne pas mettre sudo devant toutes ses commandes sinon est une solution en trois étapes :

# 1. Créer le groupe docker (si ce n’est pas déjà fait)
sudo groupadd docker

# 2. Ajouter ton utilisateur au groupe docker
sudo usermod -aG docker $USER                 # remplacer tout le $USER par ton username

# 3. Fermer la remote connection de WSL Ubuntu-22.04 avec le bouton en bas à gauche de VSCode pour redémarrer

"""
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
source install/setup.bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \"{message_id: 32, message_rate: 20.0}"

# possible de ros2 "run mission $NOM_NODE ensuite" :)


__"### Permission de fichier, par exemple "__ 
sudo chown -R avatar:avatar /home/avatar/dockerfile_playground/ros2_ws/install

__"### Publier 1 message sur un topic "__ 
ros2 topic pub /topic std_msgs/String 'data: Hello World' -1
ros2 topic pub /go_approach std_msgs/String 'data: 99,90,-15 ' -1
# ros2 topic pub /$NOMTOPIC $TYPE_MSG '$MSG' -1