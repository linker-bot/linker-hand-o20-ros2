import rclpy,os,sys,json,time,cv2
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import mediapipe as mp
from std_msgs.msg import String, Header, Float32MultiArray,Float32
from sensor_msgs.msg import JointState
import threading

from .utils.color_msg import ColorMsg
O6 = {
    "Rock": [102, 18, 0, 0, 0, 0],
    "Paper": [250, 250, 250, 250, 250, 250],
    "Scissors": [92, 87, 255, 255, 0, 0],
}
O20 = {
    "Rock1": [255, 77, 77, 120, 0, 108, 122, 138, 156, 214, 255, 0, 0, 0, 0, 255, 0, 0, 0, 0],
    "Rock2": [131, 77, 77, 120, 0, 108, 122, 131, 156, 214, 197, 0, 0, 0, 0, 111, 0, 0, 0, 0],
    "Paper": [255, 255, 255, 255, 255, 108, 122, 135, 156, 191, 255, 255, 254, 255, 88, 255, 255, 255, 255, 255],
    "Scissors": [131, 255, 255, 120, 0, 108, 194, 133, 156, 214, 197, 255, 255, 0, 0, 111, 255, 255, 0, 0],
}
class RockPaperScissors(Node):
    def __init__(self):
        super().__init__('rock_paper_scissors')
        # 声明参数（带默认值）
        self.declare_parameter('hand_type', 'left')
        self.declare_parameter('hand_joint', 'L6')

        
        # 获取参数值
        self.hand_type = self.get_parameter('hand_type').value
        self.hand_joint = self.get_parameter('hand_joint').value
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.labels = ['Rock', 'Paper', 'Scissors','NONE']  # 石头、布、剪刀
        self.shared_resource = 'NONE'
        self.current_gesture = 'NONE'
        # 60 Hz 发布者
        self.pub = self.create_publisher(JointState, f'/cb_{self.hand_type}_hand_control_cmd', 10)
        self.timer60 = self.create_timer(1.0/20.0, self.pub_hand_position)

        # # 30 Hz 订阅者
        # self.sub30 = self.create_subscription(Float32, '/slow_topic',
        #                                       self.on_30hz, 10)
        # 统计用
        self.cnt60 = 0
        self.cnt30 = 0
        self.camera_thread = threading.Thread(target=self.run_camera)
        self.camera_thread.daemon = True
        self.camera_thread.start()


    # ---------- 60 Hz 回调 ----------
    def pub_hand_position(self):
        if self.current_gesture == self.shared_resource:
            return
        if self.hand_joint == "O6":
            if self.shared_resource == "Rock":
                msg = self.joint_state_msg(O6["Paper"])
                self.pub.publish(msg)
            elif self.shared_resource == "Paper":
                msg = self.joint_state_msg(O6["Scissors"])
                self.pub.publish(msg)
            elif self.shared_resource == "Scissors":
                msg = self.joint_state_msg(O6["Rock"])
                self.pub.publish(msg)
        elif self.hand_joint == "O20":
            if self.shared_resource == "Rock":
                msg = self.joint_state_msg(O20["Paper"])
                self.pub.publish(msg)
            elif self.shared_resource == "Paper":
                msg = self.joint_state_msg(O20["Scissors"])
                self.pub.publish(msg)
            elif self.shared_resource == "Scissors":
                msg = self.joint_state_msg(O20["Rock1"])
                self.pub.publish(msg)
                time.sleep(0.2)
                msg = self.joint_state_msg(O20["Rock2"])
                self.pub.publish(msg)
        self.current_gesture = self.shared_resource



    # ---------- 30 Hz 回调 ----------
    # def on_30hz(self, msg):
    #     # 这里只把数据存下来，不阻塞
    #     self.cnt30 += 1
    #     if self.cnt30 % 30 == 0:          # 每 1 s 打印一次
    #         self.get_logger().info(f'Received 30 msgs, latest={msg.data}')
    def joint_state_msg(self, pose,vel=[]):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = [float(x) for x in pose]
        if len(vel) > 1:
            joint_state.velocity = [float(x) for x in vel]
        else:
            joint_state.velocity = [0.0] * len(pose)
        joint_state.effort = [0.0] * len(pose)
        return joint_state
    def distance(self,m, n):
        return ((n.x-m.x)**2+(n.y-m.y)**2)**0.5

    def classify_gesture(self, landmarks):
        base = 0.1


        distance_0_8 = self.distance(landmarks.landmark[0],landmarks.landmark[8])
        distance_0_12 = self.distance(landmarks.landmark[0],landmarks.landmark[12])
        distance_0_16 = self.distance(landmarks.landmark[0],landmarks.landmark[16])
        distance_0_20 = self.distance(landmarks.landmark[0],landmarks.landmark[20])
        distance_0_7 = self.distance(landmarks.landmark[0], landmarks.landmark[7])
        distance_0_11 = self.distance(landmarks.landmark[0], landmarks.landmark[11])
        distance_0_15 = self.distance(landmarks.landmark[0], landmarks.landmark[15])
        distance_0_19 = self.distance(landmarks.landmark[0], landmarks.landmark[19])


        # thumb_tip = landmarks[4].y
        # index_tip = landmarks[8].y

        if distance_0_8 > distance_0_7  and distance_0_12 > distance_0_11 and distance_0_16 < distance_0_15 and distance_0_20 < distance_0_19:
            return 2  # Scissors
        elif distance_0_8 >= distance_0_7 and distance_0_12 >= distance_0_11 and distance_0_16 > distance_0_15 and distance_0_20 > distance_0_19:
            return 1  # Paper
        elif distance_0_8 < distance_0_7 and distance_0_12 < distance_0_11 and distance_0_16 < distance_0_15 and distance_0_20 < distance_0_19:
            return 0  # Rock
        else:
            return 3 # NONE


    def run_camera(self):
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(frame_rgb)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    gesture_label = self.labels[self.classify_gesture(hand_landmarks)]
                    cv2.putText(frame, gesture_label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    self.shared_resource = gesture_label
            cv2.imshow('Hand Gesture Recognition', frame)

            if cv2.waitKey(5) & 0xFF == 27:  # 按 'ESC' 键退出
                break
        cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = RockPaperScissors()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

