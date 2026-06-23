import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from robot_speak.command_map import load_command_map  # noqa: E402
from robot_speak.protocol import (  # noqa: E402
    WonderEchoFrameParser,
    frame_to_hex,
    normalize_frame_hex,
    parse_frame_hex,
)


def test_parse_frame_hex_normalizes_spacing_and_case():
    assert normalize_frame_hex('aa 55 00 76 fb') == 'AA 55 00 76 FB'
    assert parse_frame_hex('AA 55 00 76 FB') == bytes.fromhex(
        'AA550076FB'
    )


def test_parser_extracts_frames_from_noise_and_partial_data():
    parser = WonderEchoFrameParser()

    assert parser.feed(b'noise\xAA') == []
    assert parser.feed(bytes.fromhex('55 00 76 FB')) == [
        bytes.fromhex('AA550076FB')
    ]
    assert parser.feed(bytes.fromhex('00 AA 55 00 8D FB')) == [
        bytes.fromhex('AA55008DFB')
    ]


def test_parser_resyncs_after_bad_tail():
    parser = WonderEchoFrameParser()
    data = bytes.fromhex('AA 55 00 76 00 AA 55 00 0B FB')

    assert parser.feed(data) == [bytes.fromhex('AA55000BFB')]


def test_default_command_map_finds_command_by_frame_name_and_phrase():
    commands = load_command_map()
    command = commands.find_by_frame(bytes.fromhex('AA550076FB'))

    assert command.name == 'clean_trash'
    assert command.phrase == '清扫垃圾'
    assert command.receive_frame_hex == 'AA 55 00 76 FB'
    assert commands.find('clean_trash') == command
    assert commands.find('清扫垃圾') == command
    assert commands.find('AA 55 00 76 FB') == command
    assert frame_to_hex(command.send_frame) == 'AA 55 00 76 FB'
