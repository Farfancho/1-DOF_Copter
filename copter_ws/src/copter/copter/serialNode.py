import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from example_interfaces.msg import UInt16

#* PUB: recibe los datos del serial y los publica 

#? PUB: Hace broadcast de lo que mete el usuario por Serial (va en otro archivo) 

#* SUB: Se suscribe a los datos que mete el usuario, los convierte a Gcode y los manda al micro

class CopterSerialNode (Node):
    def __init__(self):
        super().__init__("CopterSerialBridge")

        self.serial = serial.Serial(
            port = "/dev/ttyUSB0",
            baudrate=115200,
            timeout=0
        )

        self.rxBuffer = ''

        self.subscription = self.create_subscription(
            UInt16,
            "/manualSetpoint", #! PWM para el driver
            self.subscriber_callback, #? convertir a gcode y mandar
            10
        )

        self.publisher = self.create_publisher(
            UInt16,
            "/angle",
            10
        )

        self.pubTimer = self.create_timer(
            0.02,
            self.serial_callback
        )

    def subscriber_callback(self, msg):
        pwm = msg.data

        command = f"F{pwm}\n"

        self.serial.write(command.encode("utf-8"))

        self.get_logger().info(
            f"TX -> MCU: {command.strip()}"
        )
    def serial_callback(self):
        if self.serial.in_waiting == 0:
            return
        
        data = self.serial.read(
            self.serial.in_waiting
        ) 

        text = data.decode("utf-8")

        self.rxBuffer += text

        while '\n' in self.rxBuffer:
            line, self.rxBuffer = self.rxBuffer.split('\n', 1)

            line = line.strip()

            if line == '':
                continue

            angle = int(line)

            msg = UInt16()
            msg.data = angle

            self.publisher.publish(msg)

            self.get_logger().info(
                f"Angle: {msg.data}"
            )