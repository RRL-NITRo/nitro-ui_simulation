import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pyzbar import pyzbar

class QRCodeDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_code_detector_node')

        # 画像トピックのサブスクライバーを作成
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # 受信する画像トピック名
            self.image_callback,
            10)

        # 処理した画像をパブリッシュするパブリッシャー
        self.publisher_ = self.create_publisher(Image, '/camera/image_qr', 10)

        # CvBridgeのインスタンスを作成
        self.bridge = CvBridge()

        self.get_logger().info('QR Code Detector Node has been started.')

    def image_callback(self, msg):
        # 受信した画像をROSメッセージからOpenCV形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # QRコードの検出
        qr_codes = pyzbar.decode(cv_image)

        # 検出したQRコードの内容と位置に描画
        for qr_code in qr_codes:
            x, y, w, h = qr_code.rect
            qr_data = qr_code.data.decode('utf-8')
            cv2.putText(cv_image, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 処理した画像をROSメッセージに変換してパブリッシュ
        processed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRCodeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

