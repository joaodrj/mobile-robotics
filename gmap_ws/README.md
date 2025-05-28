# Projeto SLAM com TurtleBot3 – GMapping + Movimento Autônomo

Este projeto faz parte da primeira entrega da disciplina de Robôs Móveis Autônomos.  
Foi implementado o mapeamento com SLAM 2D usando o algoritmo GMapping com o robô TurtleBot3 (modelo Burger) em simulação no Gazebo.

Além do SLAM, foi feito um script em Python para o robô se mover sozinho e evitar obstáculos com base nos dados do sensor LIDAR.

---

## 🎯 Objetivo

Fazer o robô mapear um ambiente desconhecido enquanto se move de forma autônoma, utilizando o algoritmo GMapping com dados de odometria e LIDAR.

---

## ⚙️ Como rodar o projeto

Abaixo estão os comandos utilizados para executar todo o projeto, com as respectivas descrições.

```bash
#Inicia a simulação no gazebo com o robô
cd ~/mobile-robotics/mobile-robotics/gmap_ws
export TURTLEBOT3_MODEL=burger	
roslaunch turtlebot3_gazebo turtlebot3_world.launch


#Roda o algoritmo SLAM com GMapping e visualizar o mapa no RViz
cd ~/mobile-robotics/mobile-robotics/gmap_ws
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping


#Execulta o script de movimento autônomo do robô
cd ~/mobile-robotics/mobile-robotics/gmap_ws
source devel/setup.bash
rosrun gmap_slam random_walk.py


#Salva o mapa
cd ~/mobile-robotics/mobile-robotics/gmap_ws
rosrun map_server map_saver -f mapa_gerado

