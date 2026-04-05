import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32, String
import time

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (self.kp * error) + (self.kd * derivative)

class MecWheel(Node):
    def __init__(self):
        super().__init__('mec_wheel_node')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 휠 동작 상태를 알려주는 퍼블리셔
        self.status_pub = self.create_publisher(String, 'wheel_status', 10) 
        
        self.pose_sub = self.create_subscription(PoseArray, 'aruco_poses', self.aruco_callback, 10)
        self.depth_sub = self.create_subscription(Float32, 'target_depth', self.depth_callback, 10)
        
        self.TARGET_DIST = 0.3     
        self.ORBIT_RADIUS = 1.5    
        self.LPF_ALPHA = 0.3       
        self.FAST_ORBIT_VY = -0.3  
        
        self.pd_dist = PDController(kp=1.0, kd=0.2)   
        self.pd_lat = PDController(kp=1.2, kd=0.3)    
        self.pd_head = PDController(kp=1.5, kd=0.4)   
        
        self.state = "BLIND_ORBIT"
        self.last_marker_time = 0.0
        self.last_depth_time = 0.0
        self.last_control_time = time.time()
        
        self.clean_z, self.clean_x = 0.0, 0.0
        self.clean_depth = 0.0  
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def aruco_callback(self, msg):
        if len(msg.poses) > 0:
            self.last_marker_time = time.time()
            raw_z = msg.poses[0].position.z
            raw_x = msg.poses[0].position.x
            
            if self.clean_z == 0.0:
                self.clean_z = raw_z
                self.clean_x = raw_x
            else:
                self.clean_z = self.LPF_ALPHA * raw_z + (1 - self.LPF_ALPHA) * self.clean_z
                self.clean_x = self.LPF_ALPHA * raw_x + (1 - self.LPF_ALPHA) * self.clean_x

    def depth_callback(self, msg):
        self.last_depth_time = time.time()
        raw_depth = msg.data
        if 0.1 < raw_depth < 2.5:
            if self.clean_depth == 0.0:
                self.clean_depth = raw_depth
            else:
                self.clean_depth = self.LPF_ALPHA * raw_depth + (1 - self.LPF_ALPHA) * self.clean_depth

    def control_loop(self):
        msg = Twist()
        status_msg = String() # 상태 메시지 객체 생성
        current_time = time.time()
        
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        is_visible = (current_time - self.last_marker_time) < 0.5
        is_depth_valid = (current_time - self.last_depth_time) < 0.5
        current_distance = self.clean_z if is_visible else self.clean_depth

        # 기본적으로 상태는 "MOVING"으로 설정
        status_msg.data = "MOVING"

        if self.state == "BLIND_ORBIT":
            msg.linear.y = self.FAST_ORBIT_VY
            msg.angular.z = abs(self.FAST_ORBIT_VY) / self.ORBIT_RADIUS
            if is_depth_valid and self.clean_depth > 0:
                dist_error = self.clean_depth - self.ORBIT_RADIUS
                msg.linear.x = self.pd_dist.update(dist_error, dt)
            else:
                msg.linear.x = 0.0 
            if is_visible:
                self.state = "VISUAL_ALIGN"

        elif self.state == "VISUAL_ALIGN":
            if is_visible:
                dist_error = current_distance - self.ORBIT_RADIUS
                msg.linear.x = self.pd_dist.update(dist_error, dt)
                msg.linear.y = self.pd_lat.update(self.clean_x, dt)
                msg.angular.z = -self.pd_head.update(self.clean_x, dt)
                if abs(self.clean_x) < 0.05 and abs(dist_error) < 0.1:
                    self.state = "APPROACH"
            else:
                self.state = "BLIND_ORBIT" 

        elif self.state == "APPROACH":
            if is_visible:
                dist_error = current_distance - self.TARGET_DIST
                if dist_error > 0.02:
                    msg.linear.x = self.pd_dist.update(dist_error, dt)
                    msg.linear.y = self.pd_lat.update(self.clean_x, dt)
                    msg.angular.z = -self.pd_head.update(self.clean_x, dt)
                else:
                    self.state = "STOP"
            else:
                self.state = "BLIND_ORBIT"

        elif self.state == "STOP":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            status_msg.data = "STOPPED"

        # 속도 및 상태 발행
        self.cmd_vel_pub.publish(msg)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecWheel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist()) 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
