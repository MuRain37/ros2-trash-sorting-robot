import threading
import serial


FRAME_HEAD = bytes([0xAA, 0xBB])
FRAME_OVERHEAD = 5
MAX_PAYLOAD_LEN = 250


def build_frame(cmd, payload=b''):
    """Build one STM32 binary protocol frame."""
    if not 0 <= cmd <= 0xFF:
        raise ValueError('cmd must fit in one byte')
    if payload is None:
        payload = b''
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(
            f'payload too large: {len(payload)} > {MAX_PAYLOAD_LEN}'
        )

    frame = bytearray(FRAME_HEAD)
    frame.append(cmd)
    frame.append(len(payload))
    frame.extend(payload)
    frame.append(sum(frame) & 0xFF)
    return bytes(frame)


class SerialPort:
    """STM32 serial communication module."""

    def __init__(
        self, port='/dev/ttyACM0', baudrate=115200, debug=False, logger=None
    ):
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
            self._rx_thread = threading.Thread(
                target=self._rx_task, daemon=True
            )
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

    def read_frames(self, max_frames=None):
        """Read complete binary protocol frames from the receive buffer."""
        frames = []
        with self._lock:
            while max_frames is None or len(frames) < max_frames:
                head_idx = self._rx_buf.find(FRAME_HEAD)
                if head_idx < 0:
                    if len(self._rx_buf) > 1:
                        del self._rx_buf[:-1]
                    break

                if head_idx > 0:
                    del self._rx_buf[:head_idx]

                if len(self._rx_buf) < FRAME_OVERHEAD:
                    break

                payload_len = self._rx_buf[3]
                frame_len = FRAME_OVERHEAD + payload_len
                if payload_len > MAX_PAYLOAD_LEN:
                    del self._rx_buf[0]
                    continue
                if len(self._rx_buf) < frame_len:
                    break

                frame = bytes(self._rx_buf[:frame_len])

                checksum = sum(frame[:-1]) & 0xFF
                if checksum != frame[-1]:
                    if self._debug:
                        self._log(
                            '[SerialPort] 丢弃校验失败帧: '
                            f'expected=0x{checksum:02X}, got=0x{frame[-1]:02X}'
                        )
                    del self._rx_buf[0]
                    continue

                del self._rx_buf[:frame_len]
                frames.append((frame[2], frame[4:-1]))

        return frames

    def read_raw(self, n=0):
        """Read raw bytes from the receive buffer for debugging."""
        with self._lock:
            if n > 0:
                data = bytes(self._rx_buf[:n])
                del self._rx_buf[:len(data)]
                return data
            data = bytes(self._rx_buf)
            self._rx_buf.clear()
            return data

    def write(self, data):
        """
        Send data to the serial port.

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

    def write_frame(self, cmd, payload=b''):
        """Build and send one binary protocol frame."""
        return self.write(build_frame(cmd, payload))

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

    from robot_controller.packet_protocol import (
        PACKET_CMD_SERVO,
        pack_servo_command,
    )

    read_response = False
    timeout = 1.0
    sp = SerialPort()
    
    payload = pack_servo_command(1, 500.0, 1000.0)
    sent = sp.write_frame(PACKET_CMD_SERVO, payload)
    if sent:
        print(f'发送成功: {sent} 字节')
    else:
        print('发送失败')

    if read_response:
        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            frames = sp.read_frames()
            if frames:
                print(f'收到: {frames!r}')
                break
            time.sleep(0.05)
        else:
            print('超时，未收到数据')

    sp.close()


if __name__ == '__main__':
    main()
