"""ROS2 node for the WonderEcho voice module serial protocol."""

import json
import os
import subprocess
import threading
import time

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import UInt16

from robot_interfaces.action import RobotTask

from robot_speak.command_map import load_command_map
from robot_speak.protocol import (
    WonderEchoFrameParser,
    frame_to_hex,
    parse_frame_hex,
)


try:
    import serial
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None


AUDIO_EXTS = ['.mp3', '.wav', '.ogg']


class Speaker:
    """Play pre-recorded audio files for voice responses.

    Falls back to TTS command when no audio file is found.
    """

    def __init__(self, node, audio_dir, play_cmd, tts_cmd):
        self._node = node
        self._audio_dir = audio_dir
        self._play_cmd = play_cmd
        self._tts_cmd = tts_cmd
        self._tts_enabled = bool(tts_cmd)

    def speak(self, text, name=''):
        """Play audio for a command response by name. Falls back to TTS."""
        if not name:
            return
        path = self._find_audio(name)
        if path:
            self._node.get_logger().info(f'Playing audio: {path}')
            threading.Thread(
                target=self._play_file, args=(path,), daemon=True,
            ).start()
        elif self._tts_enabled:
            threading.Thread(
                target=self._say, args=(text,), daemon=True,
            ).start()

    def _find_audio(self, name):
        if not name or not self._audio_dir:
            return None
        for ext in AUDIO_EXTS:
            path = os.path.join(self._audio_dir, f'{name}{ext}')
            if os.path.isfile(path):
                return path
        return None

    def _play_file(self, path):
        try:
            cmd = self._play_cmd.format(path=path)
            subprocess.run(cmd, shell=True, timeout=30, capture_output=True)
        except Exception as e:
            self._node.get_logger().warning(f'Audio playback failed: {e}')

    def _say(self, text):
        try:
            cmd = self._tts_cmd.format(text=text)
            subprocess.run(
                cmd, shell=True, timeout=30, capture_output=True,
            )
        except Exception as e:
            self._node.get_logger().warning(f'TTS failed: {e}')


class VoiceSerialNode(Node):
    """Bridge WonderEcho serial recognition frames to ROS topics."""

    def __init__(self):
        super().__init__('voice_serial_node')

        self.declare_parameter('port', '/dev/myvoice')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('read_period_sec', 0.01)
        self.declare_parameter('commands_config', self._default_config_path())
        self.declare_parameter('speak_on_recognition', True)
        self.declare_parameter('speak_delay_sec', 0.4)
        self.declare_parameter('debug', False)
        self.declare_parameter('tts_enabled', True)
        self.declare_parameter(
            'tts_command',
            '',
        )
        self.declare_parameter(
            'audio_dir',
            self._default_audio_path(),
        )
        self.declare_parameter(
            'audio_play_cmd',
            "mpg123 -q -a plughw:1,0 '{path}'",
        )

        self.declare_parameter('raw_frame_topic', 'voice/raw_frame')
        self.declare_parameter('command_topic', 'voice/command')
        self.declare_parameter('recognized_text_topic', 'voice/recognized_text')
        self.declare_parameter('semantic_id_topic', 'voice/semantic_id')
        self.declare_parameter('command_detail_topic', 'voice/command_detail')
        self.declare_parameter('speak_command_topic', 'voice/speak_command')
        self.declare_parameter('speak_frame_topic', 'voice/speak_frame')
        self.declare_parameter('robot_task_action', '/robot_task')

        self._debug = bool(self.get_parameter('debug').value)
        self._speak_on_recognition = bool(
            self.get_parameter('speak_on_recognition').value
        )
        self._speak_delay_sec = max(
            0.0,
            float(self.get_parameter('speak_delay_sec').value),
        )
        self._commands = load_command_map(
            self.get_parameter('commands_config').value
        )
        self._parser = WonderEchoFrameParser()
        self._serial_lock = threading.Lock()
        self._pending_writes = []
        self._serial = self._open_serial()
        self._speaker = Speaker(
            self,
            audio_dir=str(self.get_parameter('audio_dir').value),
            play_cmd=str(self.get_parameter('audio_play_cmd').value),
            tts_cmd=str(self.get_parameter('tts_command').value)
            if bool(self.get_parameter('tts_enabled').value)
            else '',
        )

        self._raw_pub = self.create_publisher(
            String,
            self.get_parameter('raw_frame_topic').value,
            10,
        )
        self._command_pub = self.create_publisher(
            String,
            self.get_parameter('command_topic').value,
            10,
        )
        self._recognized_text_pub = self.create_publisher(
            String,
            self.get_parameter('recognized_text_topic').value,
            10,
        )
        self._semantic_id_pub = self.create_publisher(
            UInt16,
            self.get_parameter('semantic_id_topic').value,
            10,
        )
        self._detail_pub = self.create_publisher(
            String,
            self.get_parameter('command_detail_topic').value,
            10,
        )

        self._speak_command_sub = self.create_subscription(
            String,
            self.get_parameter('speak_command_topic').value,
            self._on_speak_command,
            10,
        )
        self._speak_frame_sub = self.create_subscription(
            String,
            self.get_parameter('speak_frame_topic').value,
            self._on_speak_frame,
            10,
        )

        # 高层任务入口：所有语音命令只发给行为树。
        cb_group = ReentrantCallbackGroup()
        self._robot_task_client = ActionClient(
            self,
            RobotTask,
            self.get_parameter('robot_task_action').value,
            callback_group=cb_group,
        )

        self._timer = self.create_timer(
            float(self.get_parameter('read_period_sec').value),
            self._poll_serial,
        )

        self.get_logger().info(
            'voice serial bridge started: '
            f'port={self.get_parameter("port").value}, '
            f'baudrate={self.get_parameter("baudrate").value}, '
            f'commands={len(self._commands.commands)}, '
            f'speak_on_recognition={self._speak_on_recognition}, '
            f'speak_delay_sec={self._speak_delay_sec}'
        )

        if self._speak_on_recognition:
            audio_dir = str(self.get_parameter('audio_dir').value)
            n_files = len([
                p for p in os.listdir(audio_dir)
                if os.path.splitext(p)[1] in AUDIO_EXTS
            ]) if os.path.isdir(audio_dir) else 0
            self.get_logger().info(
                f'voice responses via audio: {n_files} files in {audio_dir}'
            )

    def destroy_node(self):
        """Close serial port before node shutdown."""
        if getattr(self, '_serial', None) and self._serial.is_open:
            self._serial.close()
        super().destroy_node()

    def _default_config_path(self):
        try:
            package_share = get_package_share_directory('robot_speak')
        except PackageNotFoundError:
            return ''
        return package_share + '/config/default_commands.yaml'

    def _default_audio_path(self):
        try:
            package_share = get_package_share_directory('robot_speak')
        except PackageNotFoundError:
            return os.path.join(os.path.dirname(__file__), '..', 'audio')
        return package_share + '/audio'

    def _open_serial(self):
        if serial is None:
            raise RuntimeError(
                'pyserial is required to run robot_speak'
            ) from SERIAL_IMPORT_ERROR

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)
        try:
            serial_port = serial.Serial(port, baudrate, timeout=0.02)
            serial_port.rts = False
            serial_port.dtr = False
            serial_port.reset_input_buffer()
            return serial_port
        except serial.SerialException as exc:
            raise RuntimeError(
                f'failed to open voice serial port {port}: {exc}'
            ) from exc

    def _poll_serial(self):
        try:
            data = self._serial.read(64)
        except serial.SerialException as exc:
            self.get_logger().error(f'voice serial read failed: {exc}')
            return

        for frame in self._parser.feed(data):
            self._handle_frame(frame)
        self._flush_pending_writes()

    def _handle_frame(self, frame):
        frame_hex = frame_to_hex(frame)
        self._publish_string(self._raw_pub, frame_hex)

        command = self._commands.find_by_frame(frame)
        if command is None:
            self._publish_unknown(frame_hex)
            return

        self._publish_string(self._command_pub, command.name)
        self._publish_string(self._recognized_text_pub, command.phrase)
        semantic_id = UInt16()
        semantic_id.data = command.semantic_id
        self._semantic_id_pub.publish(semantic_id)
        self._publish_string(
            self._detail_pub,
            json.dumps(command.to_dict(), ensure_ascii=False),
        )

        self.get_logger().info(
            f'{frame_hex} -> {command.name} ({command.phrase})'
        )
        if self._speak_on_recognition:
            self._speaker.speak(command.response, command.name)

        self._on_voice_command(command)

    def _on_voice_command(self, command):
        """根据语音指令发送高层行为树任务。"""
        name = command.name
        if name == 'clean_trash':
            self._send_task(
                RobotTask.Goal.TASK_CLEAN_REGION,
                'clean_region_1',
                bucket_id=1,
            )
        elif name == 'stop':
            self._send_task(RobotTask.Goal.TASK_STOP, 'pause')
        elif name == 'resume':
            self._send_task(RobotTask.Goal.TASK_RESUME, 'resume')
        elif name == 'power_off':
            self._send_task(RobotTask.Goal.TASK_STOP, 'shutdown')
        elif name == 'dump_trash':
            self._send_task(
                RobotTask.Goal.TASK_DUMP_TRASH,
                'dump_trash',
                bucket_id=0,
            )

    def _send_task(self, task_type, command, bucket_id=0):
        """异步发送行为树任务，结果通过日志反馈。"""
        if not self._robot_task_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning('RobotTask action server not available')
            return

        goal = RobotTask.Goal()
        goal.task_type = int(task_type)
        goal.command = str(command)
        goal.bucket_id = int(bucket_id)

        future = self._robot_task_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done: self._handle_task_goal(done, command)
        )

    def _handle_task_goal(self, future, command):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f'{command} task send failed: {exc}')
            return

        if not goal_handle.accepted:
            self.get_logger().warning(f'{command} task rejected')
            return

        self.get_logger().info(f'{command} task accepted')
        goal_handle.get_result_async().add_done_callback(
            lambda done: self._handle_task_result(done, command)
        )

    def _handle_task_result(self, future, command):
        try:
            result = future.result().result
        except Exception as exc:
            self.get_logger().warning(f'{command} task failed: {exc}')
            return

        if result.success:
            self.get_logger().info(f'{command} task succeeded: {result.message}')
        else:
            self.get_logger().warning(f'{command} task failed: {result.message}')

    def _publish_unknown(self, frame_hex):
        self._publish_string(self._command_pub, 'unknown')
        self._publish_string(self._recognized_text_pub, '')
        semantic_id = UInt16()
        semantic_id.data = 0
        self._semantic_id_pub.publish(semantic_id)
        self._publish_string(
            self._detail_pub,
            json.dumps(
                {
                    'name': 'unknown',
                    'semantic_id': 0,
                    'phrase': '',
                    'response': '',
                    'send_frame': frame_hex,
                    'receive_frame': '',
                },
                ensure_ascii=False,
            ),
        )
        self.get_logger().warning(f'unknown voice frame: {frame_hex}')

    def _on_speak_command(self, msg):
        command = self._commands.find(msg.data)
        if command is None:
            self.get_logger().warning(
                f'unknown speak command: {msg.data!r}'
            )
            return

        self._speaker.speak(command.response, command.name)

    def _on_speak_frame(self, msg):
        try:
            frame = parse_frame_hex(msg.data)
        except ValueError as exc:
            self.get_logger().warning(f'invalid speak frame: {exc}')
            return

        self._queue_write_frame(frame, reason='speak frame')

    def _queue_write_frame(self, frame, reason=''):
        send_at = time.monotonic() + self._speak_delay_sec
        self._pending_writes.append((send_at, bytes(frame), reason))
        if self._debug:
            self.get_logger().info(
                f'Queued TX {frame_to_hex(frame)} after '
                f'{self._speak_delay_sec:.3f}s ({reason})'
            )

    def _flush_pending_writes(self):
        if not self._pending_writes:
            return

        now = time.monotonic()
        ready = [
            item for item in self._pending_writes
            if item[0] <= now
        ]
        self._pending_writes = [
            item for item in self._pending_writes
            if item[0] > now
        ]
        for _, frame, reason in ready:
            self._write_frame(frame, reason)

    def _write_frame(self, frame, reason=''):
        sent = 0
        with self._serial_lock:
            try:
                sent = self._serial.write(frame)
                self._serial.flush()
            except serial.SerialException as exc:
                self.get_logger().error(f'voice serial write failed: {exc}')
                return

        suffix = f' ({reason})' if reason else ''
        log_msg = f'TX {sent}B {frame_to_hex(frame)}{suffix}'
        if sent != len(frame):
            self.get_logger().warning(log_msg)
        else:
            self.get_logger().info(log_msg)

    @staticmethod
    def _publish_string(publisher, data):
        msg = String()
        msg.data = data
        publisher.publish(msg)


def main(args=None):
    """Start the voice serial node."""
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceSerialNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
