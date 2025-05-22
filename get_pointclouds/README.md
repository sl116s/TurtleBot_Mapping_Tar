# Practica1-Mapeado

## inicial el ros:
```
roscore
```

## Verificar si hay algunas modificacion:
```
catkin_make
export TURTLEBOT3_MODEL=waffle
source devel/setup.bash
```

## Launch el mundo:
```
roslaunch load_model init.launch
```

## La pantalla de la camera del robot:
```
rosrun listener listener
```

## iniciar nuestro codigo de la practica 1:
```
rosrun get_pointclouds get_pointclouds_node
```

## Controlar el robot:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
