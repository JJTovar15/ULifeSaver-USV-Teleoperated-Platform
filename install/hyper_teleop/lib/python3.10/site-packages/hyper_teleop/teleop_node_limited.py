#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

class TeleopNodeLimited(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Declarar parámetros con valores por defecto
        self.declare_parameter('trigger_axis', 5)
        self.declare_parameter('steer_axis', 0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('scale_linear', 0.6)
        self.declare_parameter('scale_angular', 1.0)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('min_linear_vel', 0.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('max_linear_accel', 0.5)   # m/s²
        self.declare_parameter('max_angular_accel', 1.0)  # rad/s²
        self.declare_parameter('publish_rate', 50.0)      # Hz
        self.declare_parameter('cmd_vel_timeout', 0.5)    # segundos
        
        # Obtener parámetros
        self.trigger_axis = self.get_parameter('trigger_axis').value
        self.steer_axis = self.get_parameter('steer_axis').value
        self.deadzone = self.get_parameter('deadzone').value
        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        publish_rate = self.get_parameter('publish_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        
        # Variables de estado
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.last_joy_time = self.get_clock().now()
        
        # Subscriber al joystick
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publisher de cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer para publicar a frecuencia constante
        timer_period = 1.0 / publish_rate  # segundos
        self.timer = self.create_timer(timer_period, self.publish_cmd_vel)
        self.dt = timer_period
        
        # Log de inicialización
        self.get_logger().info('Teleop node initialized')
        self.get_logger().info(f'  Trigger axis: {self.trigger_axis}')
        self.get_logger().info(f'  Steer axis: {self.steer_axis}')
        self.get_logger().info(f'  Deadzone: {self.deadzone:.3f}')
        self.get_logger().info(f'  Scale linear: {self.scale_linear:.2f}')
        self.get_logger().info(f'  Scale angular: {self.scale_angular:.2f}')
        self.get_logger().info(f'  Max linear vel: {self.max_linear_vel:.2f} m/s')
        self.get_logger().info(f'  Min linear vel: {self.min_linear_vel:.2f} m/s')
        self.get_logger().info(f'  Max angular vel: {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info(f'  Max linear accel: {self.max_linear_accel:.2f} m/s²')
        self.get_logger().info(f'  Max angular accel: {self.max_angular_accel:.2f} rad/s²')
    
    def apply_deadzone(self, value, deadzone):
        """
        Aplica deadzone a un valor del joystick
        
        Args:
            value: Valor del joystick (-1 a 1)
            deadzone: Zona muerta (0 a 1)
        
        Returns:
            Valor con deadzone aplicado y reescalado
        """
        if abs(value) < deadzone:
            return 0.0
        
        # Reescalar para que el rango [deadzone, 1.0] mapee a [0, 1.0]
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - deadzone) / (1.0 - deadzone)
        return sign * scaled
    
    def joy_callback(self, msg):
        """Procesa mensajes del joystick"""
        # Actualizar timestamp
        self.last_joy_time = self.get_clock().now()
        
        # Leer ejes del joystick
        # Trigger: normalmente va de -1 (no presionado) a 1 (totalmente presionado)
        # Lo normalizamos a 0-1
        trigger_raw = msg.axes[self.trigger_axis]
        trigger = (1.0 - trigger_raw) / 2.0  # Convierte de [-1,1] a [0,1]
        
        # Aplicar deadzone al trigger
        trigger = self.apply_deadzone(trigger * 2.0 - 1.0, self.deadzone)  # Convertir a [-1,1] para deadzone
        trigger = (trigger + 1.0) / 2.0  # Volver a [0,1]
        trigger = max(0.0, trigger)  # Asegurar que no sea negativo
        
        # Steer: va de -1 (izquierda) a 1 (derecha)
        steer = msg.axes[self.steer_axis]
        
        # Aplicar deadzone al steer
        steer = self.apply_deadzone(steer, self.deadzone)
        
        # Calcular velocidades objetivo con escalado
        self.target_linear = trigger * self.max_linear_vel * self.scale_linear
        
        # Aplicar límite mínimo (evita reversa si min > 0)
        self.target_linear = max(self.target_linear, self.min_linear_vel)
        
        # Velocidad angular proporcional al steer con escalado
        self.target_angular = steer * self.max_angular_vel * self.scale_angular
    
    def apply_acceleration_limit(self, current, target, max_accel, dt):
        """
        Aplica límites de aceleración (perfil trapezoidal)
        
        Args:
            current: Velocidad actual
            target: Velocidad objetivo
            max_accel: Aceleración máxima permitida
            dt: Tiempo transcurrido
        
        Returns:
            Nueva velocidad limitada por aceleración
        """
        diff = target - current
        max_change = max_accel * dt
        
        if abs(diff) > max_change:
            # Acelerar/desacelerar gradualmente
            return current + math.copysign(max_change, diff)
        else:
            # Ya alcanzamos el objetivo
            return target
    
    def publish_cmd_vel(self):
        """Publica comandos de velocidad con límites aplicados"""
        # Verificar timeout (seguridad)
        time_since_joy = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        
        if time_since_joy > self.cmd_vel_timeout:
            # No hemos recibido comandos del joystick recientemente
            # Detener el robot suavemente
            self.target_linear = 0.0
            self.target_angular = 0.0
        
        # Aplicar límites de aceleración (perfil trapezoidal)
        self.current_linear = self.apply_acceleration_limit(
            self.current_linear,
            self.target_linear,
            self.max_linear_accel,
            self.dt
        )
        
        self.current_angular = self.apply_acceleration_limit(
            self.current_angular,
            self.target_angular,
            self.max_angular_accel,
            self.dt
        )
        
        # Crear y publicar mensaje
        cmd = Twist()
        cmd.linear.x = self.current_linear
        cmd.angular.z = self.current_angular
        
        self.cmd_vel_pub.publish(cmd)
        
        # Log ocasional (cada 2 segundos aprox)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        
        if self._log_counter % 100 == 0:  # Cada ~2 segundos a 50Hz
            self.get_logger().debug(
                f'Publishing: linear={self.current_linear:.2f}, '
                f'angular={self.current_angular:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TeleopNodeLimited()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot antes de salir
        if 'node' in locals():
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()