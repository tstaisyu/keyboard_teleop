import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, '/motor_control', 10)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        if key == keyboard.Key.up:
            self.publish_command("forward")
        elif key == keyboard.Key.down:
            self.publish_command("backward")
        elif key == keyboard.Key.left:
            self.publish_command("left")
        elif key == keyboard.Key.right:
            self.publish_command("right")
        elif key == keyboard.Key.esc:
            self.publish_command("stop")
            return False

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % command)

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
