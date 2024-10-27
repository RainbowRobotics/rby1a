import argparse
import signal

import zmq
import numpy as np
import cv2
import json

from ui_form import Ui_MainWindow
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QMainWindow, QApplication, QMessageBox, \
    QGraphicsOpacityEffect
from PySide6.QtCore import QSocketNotifier, QTimer
from PySide6.QtGui import QIntValidator, QPixmap, Qt, QFont, QPainter, QColor, QImage

COLOR_GREEN = '#55a194'
COLOR_BLUE = '#1e85f7'
COLOR_RED = '#f16a6f'


def json_to_mat(json_data):
    rows = json_data["rows"]
    cols = json_data["cols"]
    mat_type = json_data["type"]
    data = json_data["data"]

    # OpenCV 타입에 맞춰 numpy 데이터 타입 설정
    if mat_type == cv2.CV_8UC3:
        np_data = np.array(data, dtype=np.uint8).reshape((rows, cols, 3))
    elif mat_type == cv2.CV_32FC1:
        np_data = np.array(data, dtype=np.uint8).reshape((rows, cols * 4))
        np_data = np_data.view(np.float32).reshape((rows, cols))
    else:
        raise ValueError(f"Unsupported cv::Mat type {mat_type}")

    return np_data


def mat_to_pixmap(mat, convert=True):
    # BGR에서 RGB로 변환
    if convert:
        rgb_image = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
    else:
        rgb_image = mat

    # NumPy 배열을 QImage로 변환
    height, width, channel = rgb_image.shape
    bytes_per_line = channel * width
    q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format.Format_RGB888)

    # QImage를 QPixmap으로 변환
    pixmap = QPixmap.fromImage(q_image)

    return pixmap


class DataCollectorGui(QMainWindow, Ui_MainWindow):

    def __init__(self, address, cmd_port, data_port, parent=None):
        super(DataCollectorGui, self).__init__(parent)
        self.setupUi(self)

        self.address = address
        self.cmd_port = cmd_port
        self.data_port = data_port
        self.ctx = None
        self.sub = None
        self.dealer = None
        self.notifier = None
        self.setup_zmq()

        self.PB_Zero.clicked.connect(self.zero)
        self.PB_Ready.clicked.connect(self.ready)
        self.PB_StartTeleoperation.clicked.connect(self.start_teleop)
        self.PB_StopTeleoperation.clicked.connect(self.stop_teleop)
        self.PB_Close.clicked.connect(self.close)

    def setup_zmq(self):
        self.ctx = zmq.Context.instance()

        cmd_url = f"tcp://{self.address}:{self.cmd_port}"
        # print(cmd_url)
        self.dealer = self.ctx.socket(zmq.DEALER)
        self.dealer.setsockopt(zmq.IDENTITY, b'client')
        self.dealer.connect(cmd_url)

        sub_url = f"tcp://{self.address}:{self.data_port}"
        # print(sub_url)
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(sub_url)
        self.sub.setsockopt(zmq.SUBSCRIBE, b'')

        self.notifier = QSocketNotifier(self.sub.getsockopt(zmq.FD), QSocketNotifier.Type.Read, self)
        self.notifier.activated.connect(self.subscribe)

    def zero(self):
        self.robot_common_command("Do you want the robot to go zero pose?", "zero_pose")

    def ready(self):
        self.robot_common_command("Do you want the robot to go zero pose?", "ready_pose")

    def start_teleop(self):
        self.robot_common_command("Do you want to start tele-operation?", "start_teleop")

    def stop_teleop(self):
        self.robot_common_command("Do you want to stop tele-operation?", "stop_teleop")

    def subscribe(self):
        self.notifier.setEnabled(False)

        flags = self.sub.getsockopt(zmq.EVENTS)
        while flags:
            if flags & zmq.POLLIN:
                [topic, msg] = self.sub.recv_multipart()

                if topic == b"data":
                    data = json.loads(msg)

                    self.set_background_color(self.LE_12v, COLOR_GREEN if data["power_12v"] else COLOR_RED)
                    self.set_background_color(self.LE_48v, COLOR_GREEN if data["power_48v"] else COLOR_RED)
                    self.set_background_color(self.LE_ServoOn, COLOR_GREEN if data["servo_on"] else COLOR_RED)
                    self.set_background_color(self.LE_ControlManager,
                                              COLOR_GREEN if data["control_manger"] else COLOR_RED)
                    self.set_background_color(self.LE_Teleop, COLOR_GREEN if data["teleop"] else COLOR_RED)
                    self.set_background_color(self.LE_Recording, COLOR_GREEN if data["recording"] else COLOR_RED)
                    self.L_RecordingCount.setText(str(data["recording_count"]))
                    self.LE_UPCStorageFree.setText(f"{data['storage_free']:.2f} MB")
                    self.LE_UPCStorageAvailable.setText(f"{data['storage_available']:.2f} MB")
                    self.LE_UPCStorageCapacity.setText(f"{data['storage_capacity']:.2f} MB")

                if topic == b"image":
                    data = json.loads(msg)

                    image = json_to_mat(data["cam0_rgb"])
                    pixmap = mat_to_pixmap(image)
                    scaled_pixmap = pixmap.scaled(self.L_Cam0RGB.size(), Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation)
                    self.L_Cam0RGB.setPixmap(scaled_pixmap)

                    image = json_to_mat(data["cam1_rgb"])
                    pixmap = mat_to_pixmap(image)
                    scaled_pixmap = pixmap.scaled(self.L_Cam1RGB.size(), Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation)
                    self.L_Cam1RGB.setPixmap(scaled_pixmap)

                    image = json_to_mat(data["cam2_rgb"])
                    pixmap = mat_to_pixmap(image)
                    scaled_pixmap = pixmap.scaled(self.L_Cam2RGB.size(), Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation)
                    self.L_Cam2RGB.setPixmap(scaled_pixmap)

                    depth = json_to_mat(data["cam0_depth"])
                    depth = cv2.normalize(np.clip(depth, 0, 5), None, 0, 255, cv2.NORM_MINMAX)
                    image = cv2.applyColorMap(depth.astype(np.uint8), cv2.COLORMAP_JET)
                    pixmap = mat_to_pixmap(image, False)
                    scaled_pixmap = pixmap.scaled(self.L_Cam0Depth.size(), Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation)
                    self.L_Cam0Depth.setPixmap(scaled_pixmap)

                    depth = json_to_mat(data["cam1_depth"])
                    depth = cv2.normalize(np.clip(depth, 0, 0.7), None, 0, 255, cv2.NORM_MINMAX)
                    image = cv2.applyColorMap(depth.astype(np.uint8), cv2.COLORMAP_JET)
                    pixmap = mat_to_pixmap(image, False)
                    scaled_pixmap = pixmap.scaled(self.L_Cam1Depth.size(), Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation)
                    self.L_Cam1Depth.setPixmap(scaled_pixmap)

                    depth = json_to_mat(data["cam2_depth"])
                    depth = cv2.normalize(np.clip(depth, 0, 0.7), None, 0, 255, cv2.NORM_MINMAX)
                    image = cv2.applyColorMap(depth.astype(np.uint8), cv2.COLORMAP_JET)
                    pixmap = mat_to_pixmap(image, False)
                    scaled_pixmap = pixmap.scaled(self.L_Cam2Depth.size(), Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation)
                    self.L_Cam2Depth.setPixmap(scaled_pixmap)

            flags = self.sub.getsockopt(zmq.EVENTS)

        self.notifier.setEnabled(True)

    def robot_common_command(self, text: str, command: str):
        msg_box = QMessageBox()
        msg_box.setText(text)
        msg_box.setIcon(QMessageBox.Icon.Warning)
        msg_box.setStandardButtons(QMessageBox.StandardButton.Ok | QMessageBox.StandardButton.Cancel)
        msg_box.setDefaultButton(QMessageBox.StandardButton.Ok)
        ret = msg_box.exec()

        if ret != QMessageBox.StandardButton.Ok:
            return

        if self.dealer is None:
            return  # ZMQ is not initialized yet

        command = {'command': command}
        self.dealer.send_multipart([json.dumps(command).encode('utf-8')])

    @staticmethod
    def set_background_color(le, color):
        le.setStyleSheet(f"background-color: {color}")


if __name__ == '__main__':
    # os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"

    parser = argparse.ArgumentParser(description="data_collector_gui")
    parser.add_argument('--address', type=str, required=True, help="UPC Address")
    parser.add_argument('--cmd_port', type=int, default=5454, help="UPC Command Port")
    parser.add_argument('--data_port', type=int, default=5455, help="UPC Data Port")
    args = parser.parse_args()

    app = QApplication()
    window = DataCollectorGui(address=args.address,
                              cmd_port=args.cmd_port,
                              data_port=args.data_port)
    window.show()

    # Ensure that the application quits using CTRL-C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app.exec()
