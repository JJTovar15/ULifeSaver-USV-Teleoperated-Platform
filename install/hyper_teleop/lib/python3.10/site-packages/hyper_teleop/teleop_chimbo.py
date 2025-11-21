#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import math


class TeleopIRL(Node):
    def __init__(self):
        super().__init__('teleop_irl')
        p = self.declare_parameter

        # ------------------ Botones ------------------
        self.deadman_btn      = p('deadman_button', 4).value
        self.walk_start_btn   = p('walk_start_button', 0).value
        self.walk_stop_btn    = p('walk_stop_button', 1).value
        self.estop_btn        = p('estop_button', 2).value
        self.estop_reset_btn  = p('estop_reset_button', 5).value or self.estop_btn
        self.toggle_mode_btn  = p('toggle_mode_button', 3).value

        # ------------------ Rampas (perfil trapezoidal) ------------------
        self.min_linear_vel    = p('min_linear_vel', -1.0).value
        self.max_linear_accel  = p('max_linear_accel', 1.0).value    # m/s²
        self.max_angular_accel = p('max_angular_accel', 2.0).value   # rad/s²

        self.last_lin = 0.0
        self.last_ang = 0.0
        self.last_time = self.get_clock().now()

        # ------------------ Ejes ------------------
        # Nuevo: gatillo derecho (positivo) y gatillo izquierdo (negativo)
        # Usa los índices que se mapean en tu /joy (por defecto RT=5, LT=2 en muchos drivers)
        self.trigger_axis_pos = p('trigger_axis_pos', 5).value   # RT: acelera +
        self.trigger_axis_neg = p('trigger_axis_neg', 2).value   # LT: acelera -
        self.axis_steer       = p('axis_steer', 0).value
        self.deadzone         = p('deadzone', 0.05).value

        # ------------------ Escalas ------------------
        self.scale_linear     = p('scale_linear', 1.0).value
        self.scale_angular    = p('scale_angular', 1.0).value

        # ------------------ WALK ------------------
        self.walk_lin_max     = p('walk_linear_max', 0.5).value
        self.walk_lin_min     = p('walk_linear_min', 0.0).value
        self.walk_ang_scale   = p('walk_angular_scale', 1.0).value

        # ------------------ Estado ------------------
        self.mode = 'ROLL'
        self.walk_active = False
        self.walk_linear_sp = self.walk_lin_min
        self.estop_latched = False
        self.prev_buttons = []
        self.have_prev = False

        # ------------------ Pub/Sub ------------------
        self.sub_joy = self.create_subscription(Joy, '/joy', self.on_joy, 50)
        self.pub_cmd_vel   = self.create_publisher(Twist, '/cmd_vel', 20)
        self.pub_gait_cmd  = self.create_publisher(String, '/gait_cmd', 10)   # "START"/"STOP"
        self.pub_mode_cmd  = self.create_publisher(String, '/mode_cmd', 10)   # "ROLL"/"WALK"
        self.pub_estop     = self.create_publisher(Bool, '/estop', 10)
        self.pub_estop_rst = self.create_publisher(Bool, '/estop_reset', 10)

    # ------------------ Utilidades ------------------
    def _dz(self, v, dz):
        return 0.0 if abs(v) < dz else v

    def _norm_trigger(self, t):
        """
        Convierte un gatillo en rango [-1..1] a [0..1]
        (la mayoría de drivers reportan -1 = presionado, +1 = suelto)
        """
        t = max(-1.0, min(1.0, float(t)))
        return (1.0 - t) * 0.5

    def _edge(self, current, idx):
        if not self.have_prev:
            return False
        try:
            return (not self.prev_buttons[idx]) and current[idx]
        except IndexError:
            return False

    def apply_acceleration_limit(self, current, target, max_accel, dt):
        """
        Aplica límites de aceleración (perfil trapezoidal)
        """
        diff = target - current
        max_change = max_accel * dt
        if abs(diff) > max_change:
            return current + math.copysign(max_change, diff)
        else:
            return target

    # ------------------ Callback de joystick ------------------
    def on_joy(self, msg: Joy):
        btn = [b == 1 for b in msg.buttons]
        ax  = msg.axes

        # Deadman
        deadman = btn[self.deadman_btn] if self.deadman_btn < len(btn) else False

        # Dirección (steer)
        steer = self._dz(ax[self.axis_steer], self.deadzone) if self.axis_steer < len(ax) else 0.0

        # Gatillos: RT positivo, LT negativo  -> trig = rt - lt
        rt = self._norm_trigger(ax[self.trigger_axis_pos]) if self.trigger_axis_pos < len(ax) else 0.0
        lt = self._norm_trigger(ax[self.trigger_axis_neg]) if self.trigger_axis_neg < len(ax) else 0.0
        trig = rt - lt  # [-1..+1] (si ambos a fondo se cancelan)

        # --- E-STOP / RESET ---
        if self._edge(btn, self.estop_btn):
            self.estop_latched = True
            self.pub_estop.publish(Bool(data=True))
            self.pub_cmd_vel.publish(Twist())   # cero inmediato
            self.prev_buttons = btn
            self.have_prev = True
            return

        if self.estop_latched:
            if deadman and self._edge(btn, self.estop_reset_btn):
                self.estop_latched = False
                self.pub_estop_rst.publish(Bool(data=True))
            else:
                self.pub_cmd_vel.publish(Twist())   # mantener cero
                self.prev_buttons = btn
                self.have_prev = True
                return

        # --- Toggle modo (requiere deadman) ---
        if deadman and self._edge(btn, self.toggle_mode_btn):
            self.mode = 'WALK' if self.mode == 'ROLL' else 'ROLL'
            self.walk_active = False
            self.walk_linear_sp = self.walk_lin_min
            self.pub_mode_cmd.publish(String(data=self.mode))

        # --- WALK START/STOP (evento con deadman) ---
        if self.mode == 'WALK' and deadman:
            if self._edge(btn, self.walk_start_btn):
                self.walk_active = True
                self.walk_linear_sp = self.walk_lin_max
                self.pub_gait_cmd.publish(String(data='START'))
            if self._edge(btn, self.walk_stop_btn):
                self.walk_active = False
                self.walk_linear_sp = self.walk_lin_min
                self.pub_gait_cmd.publish(String(data='STOP'))

        # --- Publicar /cmd_vel continuo en ambos modos ---
        tw = Twist()
        if self.mode == 'ROLL':
            if deadman:
                target_lin = self.scale_linear  * trig
                target_ang = self.scale_angular * steer
            else:
                target_lin = 0.0
                target_ang = 0.0

            # Perfil trapezoidal (solo en ROLL)
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            tw.linear.x  = self.apply_acceleration_limit(self.last_lin, target_lin,
                                                         self.max_linear_accel, dt)
            tw.angular.z = self.apply_acceleration_limit(self.last_ang, target_ang,
                                                         self.max_angular_accel, dt)

            self.last_lin = tw.linear.x
            self.last_ang = tw.angular.z

        else:  # WALK
            if self.walk_active:
                tw.linear.x  = self.walk_linear_sp
                tw.angular.z = self.walk_ang_scale * steer
            else:
                tw.linear.x  = 0.0
                tw.angular.z = 0.0

        self.pub_cmd_vel.publish(tw)

        self.prev_buttons = btn
        self.have_prev = True


def main():
    rclpy.init()
    node = TeleopIRL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()