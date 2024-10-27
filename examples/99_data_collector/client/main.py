import argparse
import signal

import zmq
import json

from ui_form import Ui_MainWindow
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QMainWindow, QApplication, QMessageBox, \
    QGraphicsOpacityEffect
from PySide6.QtCore import QSocketNotifier, QTimer
from PySide6.QtGui import QIntValidator, QPixmap, Qt, QFont, QPainter, QColor

COLOR_GREEN = '#55a194'
COLOR_BLUE = '#1e85f7'
COLOR_RED = '#f16a6f'


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
                    self.set_background_color(self.LE_ServoOn, COLOR_GREEN if data["control_manger"] else COLOR_RED)
                    self.set_background_color(self.LE_Running, COLOR_GREEN if data["running"] else COLOR_RED)
                    self.set_background_color(self.LE_Recording, COLOR_GREEN if data["recording"] else COLOR_RED)
                    self.L_RecordingCount.setText(data["recording_count"])
                    self.LE_UPCStorageFree.setText(data["storage_free"])
                    self.LE_UPCStorageAvailable.setText(data["storage_available"])
                    self.LE_UPCStorageCapacity.setText(data["storage_capacity"])

    @staticmethod
    def set_background_color(le, color):
        le.setStyleSheet(f"background-color: {color}")


if __name__ == '__main__':
    # os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"

    parser = argparse.ArgumentParser(description="data_collector_gui")
    parser.add_argument('--address', type=str, required=True, help="UPC Address")
    parser.add_argument('--cmd_port', type=int, default=5000, help="UPC Command Port")
    parser.add_argument('--data_port', type=int, default=5001, help="UPC Data Port")
    args = parser.parse_args()

    app = QApplication()
    window = DataCollectorGui(address=args.address,
                              cmd_port=args.cmd_port,
                              data_port=args.data_port)
    window.show()

    # Ensure that the application quits using CTRL-C
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app.exec()
