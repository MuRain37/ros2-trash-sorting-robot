"""Command mapping for WonderEcho voice frames."""

from dataclasses import dataclass

from robot_speak.protocol import frame_to_hex, parse_frame_hex


try:
    import yaml
except ImportError:  # pragma: no cover - runtime dependency guard
    yaml = None


DEFAULT_COMMAND_ROWS = [
    {
        'name': 'welcome',
        'semantic_id': 1,
        'phrase': '欢迎语',
        'response': '欢迎使用小幻',
        'send_frame': 'AA 55 01 00 FB',
        'receive_frame': 'AA 55 01 00 FB',
    },
    {
        'name': 'sleep',
        'semantic_id': 2,
        'phrase': '休息语',
        'response': '我去休息啦',
        'send_frame': 'AA 55 02 00 FB',
        'receive_frame': 'AA 55 02 00 FB',
    },
    {
        'name': 'wake',
        'semantic_id': 3,
        'phrase': '小幻小幻',
        'response': '我在',
        'send_frame': 'AA 55 03 00 FB',
        'receive_frame': 'AA 55 03 00 FB',
    },
    {
        'name': 'clean_trash',
        'semantic_id': 4,
        'phrase': '清扫垃圾',
        'response': '收到，开始工作',
        'send_frame': 'AA 55 00 76 FB',
        'receive_frame': 'AA 55 00 76 FB',
    },
    {
        'name': 'dump_trash',
        'semantic_id': 5,
        'phrase': '倒垃圾',
        'response': '收到，前往回收点',
        'send_frame': 'AA 55 00 8D FB',
        'receive_frame': 'AA 55 00 8D FB',
    },
    {
        'name': 'stop',
        'semantic_id': 6,
        'phrase': '停下',
        'response': '好的',
        'send_frame': 'AA 55 00 0B FB',
        'receive_frame': 'AA 55 00 0B FB',
    },
    {
        'name': 'resume',
        'semantic_id': 7,
        'phrase': '继续',
        'response': '收到',
        'send_frame': 'AA 55 00 0A FB',
        'receive_frame': 'AA 55 00 0A FB',
    },
    {
        'name': 'power_on',
        'semantic_id': 8,
        'phrase': '开机',
        'response': '欢迎使用小幻',
        'send_frame': 'AA 55 00 8A FB',
        'receive_frame': 'AA 55 00 8A FB',
    },
    {
        'name': 'power_off',
        'semantic_id': 9,
        'phrase': '关机',
        'response': '好的，再见',
        'send_frame': 'AA 55 00 8B FB',
        'receive_frame': 'AA 55 00 8B FB',
    },
    {
        'name': 'clean_area_1',
        'semantic_id': 10,
        'phrase': '清理区域一',
        'response': '收到，前往区域一',
        'send_frame': 'AA 55 00 8C FB',
        'receive_frame': 'AA 55 00 8C FB',
    },
    {
        'name': 'clean_area_2',
        'semantic_id': 11,
        'phrase': '清理区域二',
        'response': '收到，前往区域二',
        'send_frame': 'AA 55 00 09 FB',
        'receive_frame': 'AA 55 00 09 FB',
    },
]


@dataclass(frozen=True)
class VoiceCommand:
    """One firmware command row."""

    name: str
    semantic_id: int
    phrase: str
    response: str
    send_frame: bytes
    receive_frame: bytes

    @property
    def send_frame_hex(self):
        return frame_to_hex(self.send_frame)

    @property
    def receive_frame_hex(self):
        return frame_to_hex(self.receive_frame)

    def to_dict(self):
        return {
            'name': self.name,
            'semantic_id': self.semantic_id,
            'phrase': self.phrase,
            'response': self.response,
            'send_frame': self.send_frame_hex,
            'receive_frame': self.receive_frame_hex,
        }


class VoiceCommandMap:
    """Lookup commands by send frame, command name, or phrase."""

    def __init__(self, commands):
        self.commands = list(commands)
        self.by_send_frame = {
            command.send_frame_hex: command
            for command in self.commands
        }
        self.by_name = {
            command.name.lower(): command
            for command in self.commands
        }
        self.by_phrase = {
            command.phrase: command
            for command in self.commands
            if command.phrase
        }

    def find_by_frame(self, frame):
        return self.by_send_frame.get(frame_to_hex(frame))

    def find(self, value):
        """Find a command by name, phrase, or 5-byte hex frame."""
        key = str(value).strip()
        if not key:
            return None

        try:
            frame = parse_frame_hex(key)
        except ValueError:
            frame = None
        if frame is not None:
            return self.find_by_frame(frame)

        return self.by_name.get(key.lower()) or self.by_phrase.get(key)


def load_command_map(config_path=None):
    """Load command mapping from YAML, or use built-in defaults."""
    rows = DEFAULT_COMMAND_ROWS
    if config_path:
        if yaml is None:
            raise RuntimeError('python3-yaml is required for commands_config')
        with open(config_path, 'r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        rows = data.get('commands', rows)

    commands = [_command_from_row(row) for row in rows]
    return VoiceCommandMap(commands)


def _command_from_row(row):
    send_frame = parse_frame_hex(row['send_frame'])
    receive_frame = parse_frame_hex(row.get('receive_frame', row['send_frame']))
    return VoiceCommand(
        name=str(row['name']),
        semantic_id=int(row.get('semantic_id', 0)),
        phrase=str(row.get('phrase', '')),
        response=str(row.get('response', '')),
        send_frame=send_frame,
        receive_frame=receive_frame,
    )
