#!/usr/bin/env python3
import rclpy
import socket
import threading
import time
import json

from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, NavSatFix, NavSatStatus


def crc8_maxim(data: bytes) -> int:
    """CRC-8 Maxim/Dallas, polinomio 0x31 (reflected 0x8C)"""
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 0x01) else (crc >> 1)
    return crc & 0xFF


class WifiSerialBridgeUDP(Node):
    """
    Nodo ROS2 que hace de puente UDP con el ESP32.
    Envía comandos y recibe mensajes (ASCII o JSON).

    - Mensajes ASCII: protocolo <v1,v2,v3,cmd,id,crc>
    - Mensajes JSON: {"lat":4.6067,"lon":-74.0653,"alt":256.4}
                     → se publican en /gps/fix (NavSatFix)
    """

    def __init__(self):
        super().__init__('bridge_node')

        # --- Parámetros configurables ---
        p = self.declare_parameter
        self.esp_host = p('esp_host', '10.121.52.241').value
        self.esp_port = p('esp_port', 9000).value
        self.local_port = p('local_port', 9001).value
        self.vel_rate_hz = p('vel_rate_hz', 10.0).value
        self.body_id_hex = p('body_id_hex', '0x000').value

        # Comandos disponibles
        self.CMD_IDLE        = p('cmd_idle', 0).value
        self.CMD_ROLL        = p('cmd_roll', 1).value
        self.CMD_WALK        = p('cmd_walk', 2).value
        self.CMD_ESTOP       = p('cmd_estop', 3).value
        self.CMD_ERES        = p('cmd_ereset', 4).value
        self.CMD_WALK_START  = p('cmd_walk_start', 6).value

        # Estado interno
        self.mode = 'ROLL'
        self.v_left = 0.0
        self.v_right = 0.0

        # Contadores de diagnóstico
        self.rx_count = 0
        self.gps_count = 0
        self.last_rx_time = time.time()

        # --- Sockets UDP ---
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx_addr = (self.esp_host, self.esp_port)

        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(('0.0.0.0', self.local_port))
        self.rx_sock.settimeout(1.0)  # Aumentado para mejor diagnóstico

        # --- Publisher GPS ---
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # --- Subscribers (control) ---
        self.create_subscription(JointState, '/joint_states', self.on_wheels, 50)
        self.create_subscription(String, '/gait_cmd', self.on_gait, 10)
        self.create_subscription(String, '/mode_cmd', self.on_mode, 10)
        self.create_subscription(Bool,   '/estop', self.on_estop, 10)
        self.create_subscription(Bool,   '/estop_reset', self.on_estop_reset, 10)

        # --- Timers y threads ---
        self.reader_stop = threading.Event()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        
        self.create_timer(1.0 / self.vel_rate_hz, self.tick_roll)
        self.create_timer(5.0, self.print_diagnostics)  # Timer de diagnóstico

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'UDP Bridge iniciado:')
        self.get_logger().info(f'  → Enviando a ESP32: {self.esp_host}:{self.esp_port}')
        self.get_logger().info(f'  → Escuchando en: 0.0.0.0:{self.local_port}')
        self.get_logger().info(f'  → Esperando mensajes GPS...')
        self.get_logger().info('=' * 60)

    # ============================================================
    #                 RECEPCIÓN DE DATOS UDP
    # ============================================================
    def _reader_loop(self):
        """Thread que escucha continuamente mensajes UDP del ESP32"""
        self.get_logger().info('[RX Thread] Iniciado y esperando datos...')
        
        while not self.reader_stop.is_set():
            try:
                data, addr = self.rx_sock.recvfrom(2048)
                if not data:
                    continue
                
                self.rx_count += 1
                self.last_rx_time = time.time()
                
                s = data.decode('utf-8', errors='replace').strip()
                
                # Debug: mostrar datos crudos recibidos
                if self.rx_count <= 5 or self.rx_count % 10 == 0:
                    self.get_logger().info(f'[RX #{self.rx_count}] de {addr}: {s[:80]}...')

                # ---- Si es JSON (GPS) ----
                if s.startswith('{') and s.endswith('}'):
                    self._handle_gps_json(s, addr)
                # ---- Si es mensaje ASCII del protocolo ----
                elif s.startswith('<') and s.endswith('>'):
                    self.get_logger().info(f'[RX-ASCII] {s}')
                else:
                    self.get_logger().debug(f'[RX-Desconocido] {s[:50]}')

            except socket.timeout:
                # Timeout normal, no es error
                pass
            except Exception as e:
                self.get_logger().error(f'[RX] Error en recepción: {e}')
                time.sleep(0.5)

        self.get_logger().info('[RX Thread] Detenido')

    # ============================================================
    #                   PROCESAR MENSAJE GPS
    # ============================================================
    def _handle_gps_json(self, s: str, addr):
        try:
            d = json.loads(s)
            
            # Validar que tenga los campos requeridos
            if 'lat' not in d or 'lon' not in d:
                self.get_logger().warn(f'[GPS] JSON sin lat/lon: {s}')
                return
            
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps_link"

            msg.status.status = NavSatStatus.STATUS_FIX
            msg.status.service = NavSatStatus.SERVICE_GPS

            msg.latitude = float(d['lat'])
            msg.longitude = float(d['lon'])
            msg.altitude = float(d.get('alt', 0.0))

            # Covarianza aproximada
            msg.position_covariance = [
                2.0, 0.0, 0.0,
                0.0, 2.0, 0.0,
                0.0, 0.0, 4.0
            ]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            self.gps_pub.publish(msg)
            self.gps_count += 1
            
            # Log GPS cada mensaje (puedes cambiar a cada N mensajes)
            self.get_logger().info(
                f"[GPS #{self.gps_count}] lat={msg.latitude:.7f}, "
                f"lon={msg.longitude:.7f}, alt={msg.altitude:.2f}m"
            )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"[GPS] Error decodificando JSON: {e}")
            self.get_logger().error(f"[GPS] Contenido: {s}")
        except Exception as e:
            self.get_logger().error(f"[GPS] Error procesando mensaje: {e}")

    # ============================================================
    #                   DIAGNÓSTICO
    # ============================================================
    def print_diagnostics(self):
        """Imprime estadísticas cada 5 segundos"""
        elapsed = time.time() - self.last_rx_time
        
        self.get_logger().info('─' * 50)
        self.get_logger().info(f'Estadísticas UDP:')
        self.get_logger().info(f'  Mensajes recibidos: {self.rx_count}')
        self.get_logger().info(f'  Mensajes GPS procesados: {self.gps_count}')
        self.get_logger().info(f'  Último mensaje hace: {elapsed:.1f}s')
        
        if self.rx_count == 0:
            self.get_logger().warn('⚠️  NO SE HAN RECIBIDO MENSAJES UDP')
            self.get_logger().warn(f'   Verifica:')
            self.get_logger().warn(f'   1. ESP32 esté enviando a este puerto: {self.local_port}')
            self.get_logger().warn(f'   2. IP de este PC: {self._get_local_ip()}')
            self.get_logger().warn(f'   3. Firewall no bloqueé puerto {self.local_port}')
        
        self.get_logger().info('─' * 50)

    def _get_local_ip(self):
        """Obtiene la IP local (aproximada)"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "desconocida"

    # ============================================================
    #                   ENVÍO DE COMANDOS UDP
    # ============================================================
    def _send_frame_ascii(self, v1, v2, v3, cmd, id_hex):
        payload = f"{v1:.3f},{v2:.3f},{v3:.3f},{int(cmd)},{id_hex}"
        crc = crc8_maxim(payload.encode('ascii'))
        frame = f"<{payload},{crc:02X}>\n"
        try:
            self.tx_sock.sendto(frame.encode('ascii'), self.tx_addr)
            # Solo log cada 20 mensajes para no saturar
            if hasattr(self, '_tx_count'):
                self._tx_count += 1
            else:
                self._tx_count = 1
            
            if self._tx_count % 20 == 0:
                self.get_logger().debug(f'[TX #{self._tx_count}] {frame.strip()}')
        except Exception as e:
            self.get_logger().error(f'[TX] Error enviando: {e}')

    # ============================================================
    #                       CALLBACKS
    # ============================================================
    def on_wheels(self, msg: JointState):
        if len(msg.velocity) >= 2:
            self.v_left = msg.velocity[0]
            self.v_right = msg.velocity[1]

    def on_gait(self, msg: String):
        m = msg.data.upper()
        if m == 'START':
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_WALK_START, self.body_id_hex)
            self.get_logger().info('[CMD] WALK START')
        elif m == 'STOP':
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_IDLE, self.body_id_hex)
            self.get_logger().info('[CMD] WALK STOP')

    def on_mode(self, msg: String):
        m = msg.data.upper()
        if m in ('ROLL', 'WALK'):
            self.mode = m
            cmd = self.CMD_ROLL if m == 'ROLL' else self.CMD_WALK
            self._send_frame_ascii(0.0, 0.0, 0.0, cmd, self.body_id_hex)
            self.get_logger().info(f'[CMD] Modo cambiado a {m}')

    def on_estop(self, msg: Bool):
        if msg.data:
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_ESTOP, self.body_id_hex)
            self.get_logger().warn('[CMD] E-STOP activado')

    def on_estop_reset(self, msg: Bool):
        if msg.data:
            self._send_frame_ascii(0.0, 0.0, 0.0, self.CMD_ERES, self.body_id_hex)
            self.get_logger().info('[CMD] E-STOP reseteado')

    def tick_roll(self):
        """Envío periódico de velocidades a ESP32 (modo ROLL)"""
        if self.mode == 'ROLL':
            self._send_frame_ascii(self.v_left, self.v_right, 0.0, self.CMD_ROLL, self.body_id_hex)

    # ============================================================
    #                       SHUTDOWN
    # ============================================================
    def destroy_node(self):
        self.get_logger().info('Cerrando bridge UDP...')
        self.reader_stop.set()
        if hasattr(self, 'reader_thread'):
            self.reader_thread.join(timeout=2.0)
        self.rx_sock.close()
        self.tx_sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = WifiSerialBridgeUDP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()