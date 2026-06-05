import threading
import serial

from robot_hardware_bridge.packet_protocol import (
    FRAME_HEAD,
    FRAME_OVERHEAD,
    MAX_PAYLOAD_LEN,
    PACKET_CMD_SERVO,
    build_frame,
    pack_servo_command,
)


class SerialPort:
    """STM32 串口通信模块，负责字节流收发和帧解析."""

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
        self._rx_buf = bytearray()  # 接收字节缓冲区

        self.open()

    def _log(self, msg):
        if self._logger:
            self._logger.info(msg)
        else:
            print(msg)

    def open(self):
        """打开串口并启动接收线程."""
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
            # 后台线程持续读串口，数据放入 _rx_buf
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
        """关闭串口，停止接收线程."""
        self._running = False
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def read_frames(self, max_frames=None):
        """
        从接收缓冲区解析完整帧，返回 [(cmd, payload), ...] 列表.

        解析流程：找帧头 → 读长度 → 等够整帧 → 校验 → 返回。
        帧头错位、长度非法、校验失败均跳过并继续搜索下一帧。
        """
        frames = []
        with self._lock:
            while max_frames is None or len(frames) < max_frames:
                # 查找帧头
                head_idx = self._rx_buf.find(FRAME_HEAD)
                if head_idx < 0:
                    # 未找到帧头，保留最后1字节防止 AA 被截断
                    if len(self._rx_buf) > 1:
                        del self._rx_buf[:-1]
                    break

                # 丢弃帧头前的废字节
                if head_idx > 0:
                    del self._rx_buf[:head_idx]

                # 数据不够最小完整帧，等下次
                if len(self._rx_buf) < FRAME_OVERHEAD:
                    break

                payload_len = self._rx_buf[3]  # 第4字节 = 负载长度
                frame_len = FRAME_OVERHEAD + payload_len

                # 负载长度超限，丢弃当前帧头字节后重试
                if payload_len > MAX_PAYLOAD_LEN:
                    del self._rx_buf[0]
                    continue

                # 数据不够完整帧，等下次
                if len(self._rx_buf) < frame_len:
                    break

                frame = bytes(self._rx_buf[:frame_len])

                # 累加和校验
                checksum = sum(frame[:-1]) & 0xFF
                if checksum != frame[-1]:
                    if self._debug:
                        self._log(
                            '[SerialPort] 丢弃校验失败帧: '
                            f'expected=0x{checksum:02X}, got=0x{frame[-1]:02X}'
                        )
                    del self._rx_buf[0]  # 丢弃帧头字节，继续搜索
                    continue

                # 校验通过，移除该帧并记录
                del self._rx_buf[:frame_len]
                frames.append((frame[2], frame[4:-1]))  # (cmd, payload)

        return frames

    def read_raw(self, n=0):
        """读取接收缓冲区的原始字节（调试用）."""
        with self._lock:
            if n > 0:
                data = bytes(self._rx_buf[:n])
                del self._rx_buf[:len(data)]
                return data
            data = bytes(self._rx_buf)
            self._rx_buf.clear()
            return data

    def write(self, data):
        """发送原始字节到串口，支持 bytes/bytearray/list."""
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
        """构建并发送一帧协议数据."""
        return self.write(build_frame(cmd, payload))

    def _rx_task(self):
        """后台接收线程：持续读串口，数据放入 _rx_buf."""
        while self._running:
            try:
                chunk = self._serial.read(64)  # 每次最多读64字节
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
    """简单测试：发送一帧舵机指令."""
    read_response = False
    timeout = 1.0
    sp = SerialPort()
    import time
    servo_targets = [
        (1, 511.2),
        (2, 317.0),
        (3, 118.5),
        (4, 354.6),
        (5, 488.0),
    ]
    for servo_id, position in servo_targets:
        payload = pack_servo_command(servo_id, position, 3000.0)
        sent = sp.write_frame(PACKET_CMD_SERVO, payload)
        if sent:
            print(f'舵机{servo_id}发送成功: {sent} 字节')
        else:
            print(f'舵机{servo_id}发送失败')
        time.sleep(0.01)


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
