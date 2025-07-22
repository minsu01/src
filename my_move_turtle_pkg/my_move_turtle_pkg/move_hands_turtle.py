import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

# MediaPipe 설정
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# 손가락 제스처 분류 함수
def classify_hand(hand_landmarks):
    results = "I dont know"
    landmarks = hand_landmarks.landmark

    def is_finger_straight(tip_idx, dip_idx):
        return landmarks[tip_idx].y < landmarks[dip_idx].y

    index_straight = is_finger_straight(8, 6)
    middle_straight = is_finger_straight(12, 10)
    ring_straight = is_finger_straight(16, 14)
    pinky_straight = is_finger_straight(20, 18)

    if index_straight and not middle_straight and not ring_straight and not pinky_straight:
        results = "1"
    elif index_straight and middle_straight and not ring_straight and not pinky_straight:
        results = "2"
    elif index_straight and middle_straight and ring_straight and not pinky_straight:
        results = "3"
    elif not index_straight and not middle_straight and not ring_straight and not pinky_straight:
        results = "0"
    elif index_straight and middle_straight and ring_straight and pinky_straight:
        results = "5"

    return results

# ROS2 노드 정의
class Move_turtle(Node):
    def __init__(self):
        super().__init__('move_hand_turtle')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        self.velocity = 0.0
        self.angular = 0.0

    def turtle_key_move(self, hand_landmarks):
        msg = Twist()
        hand_result = classify_hand(hand_landmarks)
        print("인식된 손 동작:", hand_result)

        if hand_result == "1":
            self.velocity += 0.01
        elif hand_result == "0":
            self.velocity = 0.0
            self.angular = 0.0
        elif hand_result == "2":
            self.velocity -= 0.01
        elif hand_result == "3":
            self.angular += 0.01
        elif hand_result == "5":
            self.angular -= 0.01
        else:
            self.velocity = 0.0
            self.angular = 0.0

        msg.linear.x = self.velocity
        msg.angular.z = self.angular
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

# MediaPipe 실행 함수
def run_mediapipe(node):
    cap = cv2.VideoCapture(0)
    with mp_hands.Hands(
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:
        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("카메라 오류")
                continue

            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            gesture = "No Hand"

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing_styles.get_default_hand_landmarks_style(),
                        mp_drawing_styles.get_default_hand_connections_style())
                    gesture = classify_hand(hand_landmarks)
                    node.turtle_key_move(hand_landmarks)

            cv2.putText(image, gesture, (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.imshow('Hand Control', image)
            if cv2.waitKey(5) & 0xFF == 27:
                break
    cap.release()
    cv2.destroyAllWindows()

# 메인 실행
def main(args=None):
    rclpy.init(args=args)
    node = Move_turtle()
    try:
        run_mediapipe(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
