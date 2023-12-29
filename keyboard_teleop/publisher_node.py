import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pynput import keyboard

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(Int32, '/motor_control', 10)
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        self.get_logger().info("Key pressed")  # デバッグ用のログ出力
        try:
            if key == keyboard.Key.up:
                self.publish_command(1)  # forward
                self.get_logger().info("Moving Forward")
            elif key == keyboard.Key.down:
                self.publish_command(2)  # backward
                self.get_logger().info("Moving Backward")
            elif key == keyboard.Key.left:
                self.publish_command(3)  # left
                self.get_logger().info("Turning Left")
            elif key == keyboard.Key.right:
                self.publish_command(4)  # right
                self.get_logger().info("Turning Right")
            elif key == keyboard.Key.esc:
                self.publish_command(0)  # stop
                self.get_logger().info("Stopping")
                return False
        except Exception as e:
            self.get_logger().error('Error: {}'.format(e))

    def publish_command(self, command):
        msg = Int32()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % command)

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)
    keyboard_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
