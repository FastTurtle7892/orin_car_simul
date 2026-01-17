import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import math

# [수정] 속도를 0.8로 상향
MAX_SPEED = 1.5         
MAX_STEER = 0.6         
STEER_STEP = 0.05       
WHEEL_BASE = 0.28       

msg = """
========================================
   GAZEBO Real-Car Style Controller
========================================
 [ W ] : 전진 (속도 0.8 고정)
 [ S ] : 후진 (속도 -0.8 고정)
 [Space]: 정지
 
 [ A ] : 좌회전 (핸들 누적)
 [ D ] : 우회전 (핸들 누적)
 
 [ Q ] : 종료
========================================
"""

# ... (이하 코드는 이전과 동일)
class RealVehicleTeleop(Node):
    def __init__(self):
        super().__init__('real_vehicle_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_speed = 0.0
        self.current_steer = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def pub_cmd(self):
        twist = Twist()
        target_speed = self.current_speed
        
        # 정지 시 핸들 유지 트릭
        if target_speed == 0.0 and abs(self.current_steer) > 0.001:
            target_speed = 0.001

        twist.linear.x = target_speed
        
        # 아커만 공식
        if abs(target_speed) > 0.0001:
            twist.angular.z = (target_speed * math.tan(self.current_steer)) / WHEEL_BASE
        else:
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RealVehicleTeleop()

    print(msg)

    try:
        while True:
            key = node.get_key()

            if key == 'w':
                node.current_speed = MAX_SPEED
                print(f"\rForward ({node.current_speed}) | Steer: {node.current_steer:.2f}   ", end="")
            
            elif key == 's':
                node.current_speed = -MAX_SPEED
                print(f"\rBackward ({node.current_speed}) | Steer: {node.current_steer:.2f}  ", end="")

            elif key == ' ' or key == 'x':
                node.current_speed = 0.0
                print(f"\rStop                        | Steer: {node.current_steer:.2f}  ", end="")

            elif key == 'a':
                node.current_steer += STEER_STEP
                if node.current_steer > MAX_STEER:
                    node.current_steer = MAX_STEER
                print(f"\rLeft  (Angle: {node.current_steer:.2f})           ", end="")

            elif key == 'd':
                node.current_steer -= STEER_STEP
                if node.current_steer < -MAX_STEER:
                    node.current_steer = -MAX_STEER
                print(f"\rRight (Angle: {node.current_steer:.2f})           ", end="")

            elif key == 'q':
                break

            node.pub_cmd()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.publisher_.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
