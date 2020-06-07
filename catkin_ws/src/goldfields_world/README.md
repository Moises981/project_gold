Para correr sin problemas el mundo de gazebo se debe añadir la ruta de la carpeta models del paquete
en el archivo .bashrc


Ejemplo:

export GAZEBO_MODEL_PATH=/home/playtec/catkin_ws/src/goldfields_world/models:${GAZEBO_MODEL_PATH}
