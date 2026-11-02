import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar
from ps5_msgs.msg import PS5Joy

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector_node')

        # 6つの画像トピックをサブスクライブ
        self.subscribers = []
        self.topics = [
            '/NITRone/BackCamera',
            '/imax/cnoid_d435',
            '/NITRone/FrontCamera',
            '/NITRone/LeftCamera',
            '/NITRone/RightCamera',
            '/NITRone/UpCamera'
        ]
        self.topic_names = ['Back', 'Imax', 'Front', 'Left', 'Right', 'Up']
        
        for i, topic in enumerate(self.topics):
            self.subscribers.append(self.create_subscription(
                Image,
                topic,
                lambda msg, i=i: self.image_callback(msg, i),
                10))

        # PS5コントローラのメッセージをサブスクライブ
        self.joy_subscriber = self.create_subscription(
            PS5Joy,
            '/drone/ps5_joy',  # PS5のコントローラのトピック
            self.joy_callback,
            10)

        # 処理した画像をパブリッシュするパブリッシャー
        self.publisher_ = self.create_publisher(Image, '/camera/image_qr', 10)

        # QRコードのデータをパブリッシュするパブリッシャー
        self.qr_publisher = self.create_publisher(String, '/qr_detect', 10)

        # CvBridgeのインスタンスを作成
        self.bridge = CvBridge()

        # 処理する画像のインデックス（デフォルトはFront）
        self.selected_image_index = 2

        # 最後に読み取ったQRコードのデータログ（最大10行）
        self.qr_data_log = []

        # ボタンの押下状態を追跡するためのフラグ
        self.prev_r1 = False
        self.prev_l1 = False

        self.get_logger().info('QR Code Detector Node has been started.')

    def image_callback(self, msg, index):
        # 現在選択されている画像のみ処理する
        if index != self.selected_image_index:
            return

        # 受信した画像をROSメッセージからOpenCV形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # QRコードの検出
        qr_codes = pyzbar.decode(cv_image)

        # QRコードを検出した場合、データログを更新し、最大10行まで保持
        if qr_codes:
            new_qr_data = qr_codes[-1].data.decode('utf-8')

            # 新しいQRコードデータをログに追加
            self.qr_data_log.append(new_qr_data)

            # 11行以上になった場合は古い行から削除
            if len(self.qr_data_log) > 10:
                self.qr_data_log = self.qr_data_log[-10:]

            # 複数行の文字列としてパブリッシュ
            qr_msg = String()
            qr_msg.data = "\n".join(self.qr_data_log)
            self.qr_publisher.publish(qr_msg)

            # QRコードを緑の矩形で囲む
            for qr_code in qr_codes:
                (x, y, w, h) = qr_code.rect
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 左下に最後に読み取ったQRコードのデータを表示
        if self.qr_data_log:
            last_qr_data = self.qr_data_log[-1]
            text_position = (10, cv_image.shape[0] - 10)
            self._draw_text(cv_image, last_qr_data, text_position, (0, 255, 255))

        # 現在使用中の画像の名称を右上に表示（黄色でサイズを半分に）
        current_topic_name = self.topic_names[self.selected_image_index]
        text_position = (cv_image.shape[1] - 110, 20)  # 右上の端に近づける
        self._draw_text(cv_image, current_topic_name, text_position, (0, 255, 255))

        # 処理した画像をROSメッセージに変換してパブリッシュ
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(processed_image_msg)

    def joy_callback(self, msg):
        # r1ボタンが押されたら次の画像に切り替え
        if msg.r1 and not self.prev_r1:
            self.selected_image_index = (self.selected_image_index + 1) % 6
            self.get_logger().info(f'Selected image index: {self.selected_image_index}')

        # l1ボタンが押されたら前の画像に戻る
        if msg.l1 and not self.prev_l1:
            self.selected_image_index = (self.selected_image_index - 1) % 6
            self.get_logger().info(f'Selected image index: {self.selected_image_index}')

        # ボタンの状態を保存
        self.prev_r1 = msg.r1
        self.prev_l1 = msg.l1

    def _draw_text(self, image, text, position, color):
        """文字を黒で縁取りして描画するヘルパー関数"""
        font_scale = 0.45
        thickness = 1
        font = cv2.FONT_HERSHEY_SIMPLEX

        # 文字を黒で縁取り
        cv2.putText(image, text, position, font, font_scale, (0, 0, 0), thickness + 2)
        # 本来の文字を描画
        cv2.putText(image, text, position, font, font_scale, color, thickness)

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
