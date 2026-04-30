class SerialPort:
    """STM32 串口通信模块。

    接入硬件后实现 open/read/write/close。
    """

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self._port = port
        self._baudrate = baudrate
        self._serial = None

    def open(self):
        return False

    def close(self):
        pass

    def read(self):
        """读取一帧数据，返回 (encoder, imu) 元组。

        encoder: [v1, v2, v3, v4] 四轮速度
        imu: 未实现则为 None
        无数据时返回 (None, None)。
        """
        return None, None

    def write(self, data: list):
        """发送指令给 STM32。"""
        pass
