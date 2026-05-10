import threading
import serial


class SerialPort:
    """STM32 串口通信模块。"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self._port_name = port
        self._baudrate = baudrate
        self._serial = None
        self._running = False
        self._rx_thread = None
        self._lock = threading.Lock()
        self._rx_buf = bytearray()

        self.open()

    def open(self):
        if self._serial and self._serial.is_open:
            return True
        try:
            self._serial = serial.Serial(
                self._port_name, self._baudrate, timeout=0.1,
            )
            self._serial.rts = False
            self._serial.dtr = False
            self._serial.reset_input_buffer()
            self._running = True
            self._rx_thread = threading.Thread(target=self._rx_task, daemon=True)
            self._rx_thread.start()
            print(f'[SerialPort] 已打开 {self._port_name} @ {self._baudrate}')
            return True
        except serial.SerialException as e:
            print(f'[SerialPort] 打开失败 ({self._port_name}): {e}')
            return False

    def close(self):
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def read_raw(self, n=0):
        """读取接收缓冲区中的原始字节。

        n > 0 时返回恰好 n 字节（不足则返回实际读取量）。
        n = 0 时返回缓冲区中所有可用字节。
        """
        with self._lock:
            if n > 0 and len(self._rx_buf) >= n:
                data = bytes(self._rx_buf[:n])
                del self._rx_buf[:n]
                return data
            elif n == 0 and len(self._rx_buf) > 0:
                data = bytes(self._rx_buf)
                self._rx_buf.clear()
                return data
        return b''

    def read_frame(self):
        """读取并解析一帧数据，返回 (encoder, imu) 元组。

        encoder: [v1, v2, v3, v4] 四轮速度列表，无数据时为 None
        imu:     未实现则为 None
        """
        encoder = None
        imu = None

        with self._lock:
            if not self._rx_buf:
                return None, None
            data = bytes(self._rx_buf)
            self._rx_buf.clear()

        for line in data.split(b'\n'):
            line = line.strip()
            if not line:
                continue
            try:
                text = line.decode('utf-8', errors='replace')
            except Exception:
                continue

            if text.startswith('ENCODER:'):
                parts = text.split(':', 1)[1].split(',')
                if len(parts) == 4:
                    try:
                        encoder = [float(v) for v in parts]
                    except ValueError:
                        pass
            # 可扩展 IMU 等其他帧类型解析

        return encoder, imu

    def write(self, data):
        """发送数据到串口。

        data: bytes 或 bytearray 或 list[int]
        """
        if not self._serial or not self._serial.is_open:
            return
        if isinstance(data, list):
            data = bytes(data)
        try:
            n = self._serial.write(data)
            self._serial.flush()
            print(f'[SerialPort] 已发送 {n} 字节: {data.decode("utf-8", errors="replace").strip()}')
            return n
        except serial.SerialException as e:
            print(f'[SerialPort] 写入失败: {e}')
            return 0

    def _rx_task(self):
        while self._running:
            try:
                chunk = self._serial.read(64)
                if chunk:
                    with self._lock:
                        self._rx_buf.extend(chunk)
            except serial.SerialException:
                break
            except Exception:
                break


def main():

    data = "MOTOR:0,0,0,0"
    read_response = False
    timeout = 1.0
    sp = SerialPort()
    sp.write(data.encode('utf-8'))

    if read_response:
        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            resp = sp.read_raw()
            if resp:
                print(f'收到: {resp.decode("utf-8", errors="replace")}')
                break
            time.sleep(0.05)
        else:
            print('超时，未收到数据')

    sp.close()


if __name__ == '__main__':
    main()
