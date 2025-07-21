Paquets ros2 pour la mission 2 de AEAC 2025. 

# Cheat sheet pour le développement ros2 

### 👉 Important : essentiel de sourcer l'install du ws:
Dans tous les nouveaux terminaux utilisés pour l'env ros2, exécuter :
   ```bash
   source install/setup.bash
   ```
   Si c'est le seul ws dans ta machine, et que tu comprends les risques de conflits avec les autres projets ros sur ta machine, tu peux mettre la ligne suivante dans ton .bashrc. (Remplacer /home/<user> par votre position absolue de ardu_ws)
   
   ```bash
   source /home/<user>/ardu_ws/install/setup.bash
   ```

### 👉 Début de simulation avez gazebo, ros2, rviz:
**Ouvrir un terminal** et exécuter la commande suivante :
   ```bash
   ros2 launch ardupilot_gz_bringup iris_runway.launch.py
   ```

## 👉 Début de mavros simulation sans gazebo, rviz, mais ros2:
   ```bash
   ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
    transport:=udp4 \
    synthetic_clock:=True \
    wipe:=True \
    model:=quad \
    speedup:=1 \
    slave:=0 \
    instance:=0 \
    defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm \
    sim_address:=127.0.0.1 \
    master:=tcp:127.0.0.1:5760 \
    sitl:=127.0.0.1:5501 \
    out:=udp:127.0.0.1:14550 \
    out:=udp:127.0.0.1:14551 \
    home:=50.0974520,-110.7357341,101.855,0
   ```
   Les deux connections udp sont les sortis de la simulation, ici distribuer à mavros et pymavlink, set up différent avec mission planner. (voir plus loin ... TODO, je comprends pas mais ça marche)

## 👉 Début de simulation de base (sans gazebo, ni rviz, ni ros2):
   ```bash
   sim_vehicle.py -v copter --console --map -w
   ```

   ou, pour la position de la compé:
   ```bash
   sim_vehicle.py -v ArduCopter --console --map -w --custom-location=50.0974520,-110.7357341,101.855,0
   ```

## 👉 Transfert de la simulation pour mavros et pymavlink:
   ```bash
   mavproxy.py --master=tcp:127.0.0.1:5762 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
   ```

## 👉 Début de mavros:
   ```bash
   ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14550 
   ```


## 👉 Début de l'ensembles des noeuds de mission, provient de start_mission package:
   ```bash
   ros2 launch start_mission start.launch.py
   ```

## 👉 Recontruire l'env. ROS2:
   ```bash
   colcon build
   ```
   
## 👉 Recontruire un package donné:
   ```bash
   colcon build --packages-select mission
   ```
Remplacer mission par le nom de votre package custom si ce n'est pas mission qui doit être recontruit.

   
## 👉 Run un noeud spécfique:
   ```bash
   ros2 run <package name> <node name>
   ```
   Remplacer les <xxx> !!


## Installation MAVROS:
   ```bash
   sudo apt update && sudo apt upgrade -y

   ```

   ```bash
   sudo apt install -y python3-pip python3-colcon-common-extensions \
                    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras \
                    ros-${ROS_DISTRO}-mavlink \
                    geographiclib-tools

   ```
   
   ```bash
   sudo apt update
   sudo apt install geographiclib-tools
   ```

   ```bash
   sudo geographiclib-get-geoids egm96-5
   sudo geographiclib-get-gravity egm2008
   sudo geographiclib-get-magnetic emm2015
   ```
   


