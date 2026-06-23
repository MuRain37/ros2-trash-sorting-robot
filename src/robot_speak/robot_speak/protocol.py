"""WonderEcho fixed-length serial protocol helpers."""


FRAME_HEAD = bytes([0xAA, 0x55])
FRAME_TAIL = 0xFB
FRAME_LEN = 5


def frame_to_hex(frame):
    """Convert a 5-byte frame to uppercase hex text."""
    frame = bytes(frame)
    if len(frame) != FRAME_LEN:
        raise ValueError(f'frame must be {FRAME_LEN} bytes, got {len(frame)}')
    return ' '.join(f'{byte:02X}' for byte in frame)


def parse_frame_hex(value):
    """Parse hex text like 'AA 55 00 76 FB' into a validated frame."""
    if isinstance(value, bytes):
        frame = value
    else:
        text = str(value).strip().replace(',', ' ')
        parts = [part for part in text.split() if part]
        if len(parts) != FRAME_LEN:
            raise ValueError(
                f'frame must have {FRAME_LEN} bytes: {value!r}'
            )
        try:
            frame = bytes(int(part, 16) for part in parts)
        except ValueError as exc:
            raise ValueError(f'invalid hex frame: {value!r}') from exc

    validate_frame(frame)
    return bytes(frame)


def normalize_frame_hex(value):
    """Parse and render a frame as canonical uppercase hex text."""
    return frame_to_hex(parse_frame_hex(value))


def validate_frame(frame):
    """Validate WonderEcho frame shape."""
    frame = bytes(frame)
    if len(frame) != FRAME_LEN:
        raise ValueError(f'frame must be {FRAME_LEN} bytes, got {len(frame)}')
    if not frame.startswith(FRAME_HEAD):
        raise ValueError(f'invalid frame head: {frame_to_hex(frame)}')
    if frame[-1] != FRAME_TAIL:
        raise ValueError(f'invalid frame tail: {frame_to_hex(frame)}')


class WonderEchoFrameParser:
    """Incrementally extract WonderEcho frames from a noisy byte stream."""

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        """Append bytes and return all complete valid frames."""
        if data:
            self._buffer.extend(data)

        frames = []
        while len(self._buffer) >= FRAME_LEN:
            head_index = self._buffer.find(FRAME_HEAD)
            if head_index < 0:
                self._keep_possible_partial_head()
                break

            if head_index > 0:
                del self._buffer[:head_index]

            if len(self._buffer) < FRAME_LEN:
                break

            if self._buffer[FRAME_LEN - 1] != FRAME_TAIL:
                del self._buffer[0]
                continue

            frame = bytes(self._buffer[:FRAME_LEN])
            del self._buffer[:FRAME_LEN]
            frames.append(frame)

        return frames

    def _keep_possible_partial_head(self):
        if self._buffer and self._buffer[-1] == FRAME_HEAD[0]:
            del self._buffer[:-1]
        else:
            self._buffer.clear()
