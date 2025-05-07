#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Parâmetros - ajustados para um ambiente com obstáculos próximos
        self.safe_distance = rospy.get_param("~safe_distance", 0.2)  
        self.linear_speed = rospy.get_param("~linear_speed", 0.15)
        self.angular_speed = rospy.get_param("~angular_speed", 0.3)
        
        # Controle de movimento
        self.twist = Twist()
        self.obstacle_detected = False
        self.laser_data_received = False
        
        # Variáveis para controle de movimento
        self.last_min_distance = float('inf')
        self.straight_line_counter = 0
        self.force_straight_movement = False
        self.force_straight_until = rospy.Time.now()
        
        # Inicializar com movimento para frente 
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
        
        rospy.loginfo("ObstacleAvoider FIXED node started - Parâmetros: safe_dist=%.2f, lin_speed=%.2f, ang_speed=%.2f", 
                     self.safe_distance, self.linear_speed, self.angular_speed)
        
        self.rate = rospy.Rate(10)  # 10Hz
        self.run()
    
    def scan_callback(self, scan_msg):
        self.laser_data_received = True
        
        # Logs sobre o scan apenas na primeira vez
        if not hasattr(self, '_scan_info_shown'):
            rospy.loginfo("Scan info: %d pontos (min: %.2f, max: %.2f)", 
                         len(scan_msg.ranges), scan_msg.range_min, scan_msg.range_max)
            
            # verificar valores do scan
            valid_values = [r for r in scan_msg.ranges if not np.isnan(r) and not np.isinf(r) 
                           and scan_msg.range_min < r < scan_msg.range_max]
            rospy.loginfo("Valores válidos no scan: %d de %d", len(valid_values), len(scan_msg.ranges))
            
            if len(valid_values) < 10:
                rospy.logwarn("POUCOS VALORES VÁLIDOS NO SCAN! Verificar configuração do laser.")
            
            self._scan_info_shown = True
        
        # Obtém o meio do scan (frente do robô)
        center_index = len(scan_msg.ranges) // 2
        
        # Define janelas para analisar a frente e laterais
        front_window = 40  # 40 pontos à frente (mais estreito para evitar falsos positivos)
        side_window = 60   # 60 pontos de cada lado
        
        # Extrair leituras
        front_ranges = scan_msg.ranges[center_index - front_window//2 : center_index + front_window//2]
        left_ranges = scan_msg.ranges[center_index - side_window - front_window//2 : center_index - front_window//2]
        right_ranges = scan_msg.ranges[center_index + front_window//2 : center_index + front_window//2 + side_window]
        
        # Função para filtrar valores válidos
        def filter_valid_ranges(ranges, min_dist=0.05):
            return [r for r in ranges if min_dist < r < scan_msg.range_max and not np.isnan(r) and not np.isinf(r)]
        
        # Filtrar valores válidos
        valid_front = filter_valid_ranges(front_ranges)
        valid_left = filter_valid_ranges(left_ranges)
        valid_right = filter_valid_ranges(right_ranges)
        
        # Se não houver valores válidos, considerar como caminho livre
        if not valid_front:
            self.obstacle_detected = False
            self.last_min_distance = 1.0  # Valor arbitrário "seguro"
            return
        
        # Calcular distâncias mínimas
        min_front = min(valid_front) if valid_front else float('inf')
        min_left = min(valid_left) if valid_left else float('inf')
        min_right = min(valid_right) if valid_right else float('inf')
        
        # Armazenar distância mínima frontal
        self.last_min_distance = min_front
        
        # Ignora leituras muito baixas (potenciais erros de medição)
        if min_front < 0.05:
            rospy.logwarn("Leitura frontal muito baixa (%.2fm) - possível erro de medição", min_front)
            return
        
        # Detecção de obstáculo
        self.obstacle_detected = min_front < self.safe_distance
        
        # Logs periódicos
        if not hasattr(self, '_last_log_time') or rospy.Time.now() - self._last_log_time > rospy.Duration(2.0):
            rospy.loginfo("Distâncias - Frente: %.2fm, Esquerda: %.2fm, Direita: %.2fm %s", 
                         min_front, min_left, min_right, 
                         "- OBSTÁCULO!" if self.obstacle_detected else "")
            self._last_log_time = rospy.Time.now()
    
    def run(self):
        last_obstacle_time = rospy.Time.now()
        
        # Força movimento inicial para frente por 3 segundos
        self.force_straight_until = rospy.Time.now() + rospy.Duration(3.0)
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            
            # Inicialmente, força movimento em linha reta
            if current_time < self.force_straight_until:
                rospy.loginfo("Forçando movimento em linha reta...")
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.rate.sleep()
                continue
            
            # Verifica recepção de dados do laser
            if not self.laser_data_received:
                rospy.logwarn("Não recebendo dados do laser! Verificar tópico /scan")
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.cmd_pub.publish(self.twist)
                self.rate.sleep()
                continue
            
            # Verifica se está preso em rotação
            if self.obstacle_detected:
                # Se está detectando obstáculo por muito tempo, tentar forçar movimento
                if current_time - last_obstacle_time > rospy.Duration(5.0):
                    rospy.loginfo("Detectando obstáculo por muito tempo. Tentando forçar movimento...")
                    self.force_straight_until = current_time + rospy.Duration(1.5)
                    last_obstacle_time = current_time
                    continue
                
                # Verificar se precisa alternar direção
                if not hasattr(self, '_last_rotation_direction') or \
                   current_time - self._last_direction_change > rospy.Duration(3.0):
                    # Alternar direção periodicamente
                    self._last_rotation_direction = 1 if not hasattr(self, '_last_rotation_direction') \
                                                  else -self._last_rotation_direction
                    self._last_direction_change = current_time
                
                # Girar para evitar obstáculo
                self.twist.linear.x = 0.0
                self.twist.angular.z = self._last_rotation_direction * self.angular_speed
                rospy.loginfo("Girando para evitar obstáculo a %.2fm (direção: %d)", 
                             self.last_min_distance, self._last_rotation_direction)
            else:
                # Caminho livre, seguir em frente
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                rospy.loginfo("Seguindo em frente, dist: %.2fm", self.last_min_distance)
                last_obstacle_time = current_time
                
                # Incrementa contador de movimento em linha reta
                self.straight_line_counter += 1
                
                # A cada 50 ciclos em linha reta, logar progresso
                if self.straight_line_counter % 50 == 0:
                    rospy.loginfo("Progresso: %d ciclos em linha reta", self.straight_line_counter)
            
            # Publica comando
            self.cmd_pub.publish(self.twist)
            
            # Log dos comandos enviados
            rospy.loginfo("COMANDO: linear.x=%.2f, angular.z=%.2f", self.twist.linear.x, self.twist.angular.z)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ObstacleAvoider()
    except rospy.ROSInterruptException:
        pass