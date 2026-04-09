import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from std_msgs.msg import Float32, String
import time
from collections import deque

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
        self.status_pub = self.create_publisher(String, 'wheel_status', 10) 
        
        self.pose_sub = self.create_subscription(PoseArray, 'aruco_poses', self.aruco_callback, 10)
        self.depth_sub = self.create_subscription(Float32, 'target_depth', self.depth_callback, 10)
        
        self.TARGET_DIST = 0.3     
        self.ORBIT_RADIUS = 1.5    
        self.FAST_ORBIT_VY = -0.3  
        
        self.pd_dist = PDController(kp=1.0, kd=0.2)   
        self.pd_lat = PDController(kp=1.2, kd=0.3)    
        self.pd_head = PDController(kp=1.5, kd=0.4)   
        
        self.state = "BLIND_ORBIT"
        self.last_marker_time = 0.0
        self.last_depth_time = 0.0
        self.last_control_time = time.time()
        
        # 최신 5개의 데이터를 담을 큐 (버퍼)
        self.pose_window_z = deque(maxlen=5)
        self.pose_window_x = deque(maxlen=5)
        self.depth_window = deque(maxlen=5)
        
        # 평균 계산 결과가 담길 변수
        self.clean_z, self.clean_x = 0.0, 0.0
        self.clean_depth = 0.0  
        
        self.timer = self.create_timer(0.1, self.control_loop)

    def aruco_callback(self, msg):
        """마커 인식 시 최신 5개 데이터의 이동 평균 적용"""
        if len(msg.poses) > 0:
            self.last_marker_time = time.time()
            
            # 큐에 최신 데이터 추가 (자동으로 가장 오래된 것은 밀려남)
            self.pose_window_z.append(msg.poses[0].position.z)
            self.pose_window_x.append(msg.poses[0].position.x)
            
            # 평균값 도출
            self.clean_z = sum(self.pose_window_z) / len(self.pose_window_z)
            self.clean_x = sum(self.pose_window_x) / len(self.pose_window_x)

    def depth_callback(self, msg):
        """마커와 무관하게 위성 표면 뎁스 거리 수신 및 평균 적용"""
        self.last_depth_time = time.time()
        raw_depth = msg.data
        
        # 튀는 값(허공) 필터링 후 큐에 추가
        if 0.1 < raw_depth < 2.5:
            self.depth_window.append(raw_depth)
            self.clean_depth = sum(self.depth_window) / len(self.depth_window)

    def control_loop(self):
        msg = Twist()
        status_msg = String()
        current_time = time.time()
        
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        
        # 데이터 수신 상태 체크 (0.5초 이내 갱신 여부)
        is_visible = (current_time - self.last_marker_time) < 0.5
        is_depth_valid = (current_time - self.last_depth_time) < 0.5
        current_distance = self.clean_z if is_visible else self.clean_depth

        # 기본 상태는 "MOVING"
        status_msg.data = "MOVING"

        # 마커를 찾으며 원궤도 공전
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

        # 마커를 보며 중앙 정렬 및 거리(1.5m) 유지
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

        # 일직선을 유지하며 0.3m까지 전진
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

        # 최종 정지 및 완료 신호 전송
        elif self.state == "STOP":
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            
            # 완전히 정지했을 때 로봇 팔에게 신호를 주기 위해 "STOPPED" 발행
            status_msg.data = "STOPPED"

        # 발행 (Publish)
        self.cmd_vel_pub.publish(msg)
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MecWheel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 사용자가 Ctrl+C를 누르면 정상적인 종료 로직으로 넘어감
        node.get_logger().info('종료 신호 수신: 모터를 정지하고 노드를 닫습니다.')
    except Exception as e:
        node.get_logger().error(f'알 수 없는 에러 발생: {e}')
    finally:
        # ROS 통신 컨텍스트가 살아있을 때만 정지 명령 발행 시도
        if rclpy.ok():
            try:
                node.cmd_vel_pub.publish(Twist())
            except Exception:
                pass
        
        # 안전하게 노드 파괴 및 rclpy 종료
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
