import threading
import serial


class SerialPort:
    """STM32 串口通信模块。"""

    def __init__(self, port='/dev/ttyACM0', baudrate=115200, debug=False, logger=None):
        self._port_name = port
        self._baudrate = baudrate
        self._debug = debug
        self._logger = logger
        self._serial = None
        self._running = False
        self._rx_thread = None
        self._lock = threading.Lock()
        self._rx_buf = bytearray()

        self.open()

    def _log(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)

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
            self._log(f'[SerialPort] 已打开 {self._port_name} @ {self._baudrate}')
            return True
        except serial.SerialException as e:
            self._log(f'[SerialPort] 打开失败 ({self._port_name}): {e}')
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
                last_nl = self._rx_buf.rfind(b'\n')
                if last_nl >= 0:
                    data = bytes(self._rx_buf[:last_nl + 1])
                    del self._rx_buf[:last_nl + 1]
                else:
                    data = b''
                return data
        return b''

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
            if self._debug:
                self._log(f'[SerialPort] TX {n}B: {data!r}')
            return n
        except serial.SerialException as e:
            self._log(f'[SerialPort] 写入失败: {e}')
            return 0

    def _rx_task(self):
        while self._running:
            try:
                chunk = self._serial.read(64)
                if chunk:
                    if self._debug:
                        self._log(f'[SerialPort] RX {len(chunk)}B: {chunk!r}')
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
