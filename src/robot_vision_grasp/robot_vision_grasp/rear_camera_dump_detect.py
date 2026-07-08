#!/usr/bin/env python3
"""Standalone OpenVINO detector for the rear dump camera."""

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np


DEFAULT_USB_ID = '1908:2310'
DEFAULT_CLASS_NAME = 'bin'
DEFAULT_CONFIDENCE = 0.55
DEFAULT_IOU = 0.4
DEFAULT_MODEL_DEVICE = 'CPU'


class DumpBinDetector:
    """Small OpenVINO YOLO detector matching the existing dump bin model."""

    def __init__(self, model_path, ov_device, conf_threshold, iou_threshold):
        import openvino as ov

        core = ov.Core()
        model = core.read_model(str(model_path))
        self._compiled = core.compile_model(model, ov_device)
        self._input_name = model.input(0).get_any_name()
        self._output_name = model.output(0).get_any_name()
        shape = model.input(0).shape
        self._input_height = int(shape[2])
        self._input_width = int(shape[3])
        self._conf_threshold = float(conf_threshold)
        self._iou_threshold = float(iou_threshold)

    def detect(self, frame):
        blob, scale, pad_w, pad_h = self._preprocess(frame)
        infer_request = self._compiled.create_infer_request()
        raw = infer_request.infer({self._input_name: blob})[self._output_name]
        pred = np.asarray(raw)
        if pred.ndim == 3:
            pred = pred[0]
        if pred.ndim == 2 and pred.shape[0] <= 8 and pred.shape[0] < pred.shape[1]:
            pred = pred.T
        if pred.ndim != 2 or pred.shape[1] < 5:
            raise ValueError(f'unexpected OpenVINO output shape: {raw.shape}')

        boxes = []
        scores = []
        for row in pred:
            score = self._score_from_row(row)
            if score < self._conf_threshold:
                continue
            xc, yc, bw, bh = [float(value) for value in row[:4]]
            x1 = (xc - bw * 0.5 - pad_w) / scale
            y1 = (yc - bh * 0.5 - pad_h) / scale
            x2 = (xc + bw * 0.5 - pad_w) / scale
            y2 = (yc + bh * 0.5 - pad_h) / scale
            if x2 <= x1 or y2 <= y1:
                continue
            boxes.append([x1, y1, x2, y2])
            scores.append(score)

        if not boxes:
            return []

        boxes_arr = np.asarray(boxes, dtype=np.float32)
        scores_arr = np.asarray(scores, dtype=np.float32)
        keep = self._nms(boxes_arr, scores_arr)
        detections = [
            self._detection_from_box(frame, boxes_arr[index], scores_arr[index])
            for index in keep
        ]
        detections.sort(key=lambda item: item['confidence'], reverse=True)
        return detections

    def _preprocess(self, frame):
        height, width = frame.shape[:2]
        scale = min(self._input_height / height, self._input_width / width)
        new_h = int(round(height * scale))
        new_w = int(round(width * scale))
        pad_h = (self._input_height - new_h) // 2
        pad_w = (self._input_width - new_w) // 2

        resized = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.full(
            (self._input_height, self._input_width, 3),
            114,
            dtype=np.uint8,
        )
        canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized
        blob = np.transpose(canvas.astype(np.float32) / 255.0, (2, 0, 1))
        return blob[np.newaxis, :, :, :], scale, pad_w, pad_h

    def _score_from_row(self, row):
        if row.shape[0] == 5:
            return float(row[4])
        return float(np.max(row[4:]))

    def _nms(self, boxes, scores):
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            index = order[0]
            keep.append(index)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[index], x1[order[1:]])
            yy1 = np.maximum(y1[index], y1[order[1:]])
            xx2 = np.minimum(x2[index], x2[order[1:]])
            yy2 = np.minimum(y2[index], y2[order[1:]])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            union = areas[index] + areas[order[1:]] - inter
            iou = inter / np.maximum(union, 1e-6)
            order = order[1:][iou <= self._iou_threshold]
        return keep

    def _detection_from_box(self, frame, xyxy, confidence):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = np.clip(
            xyxy,
            [0.0, 0.0, 0.0, 0.0],
            [width - 1.0, height - 1.0, width - 1.0, height - 1.0],
        )
        center_x = (x1 + x2) * 0.5
        center_y = (y1 + y2) * 0.5
        image_center_x = (float(width) - 1.0) * 0.5
        image_center_y = (float(height) - 1.0) * 0.5
        offset_x = center_x - image_center_x
        offset_y = center_y - image_center_y
        return {
            'class_name': DEFAULT_CLASS_NAME,
            'confidence': float(confidence),
            'x1': float(x1),
            'y1': float(y1),
            'x2': float(x2),
            'y2': float(y2),
            'center_x_px': float(center_x),
            'center_y_px': float(center_y),
            'offset_x_px': float(offset_x),
            'offset_y_px': float(offset_y),
            'offset_x_ratio': float(offset_x / max(1.0, image_center_x)),
            'offset_y_ratio': float(offset_y / max(1.0, image_center_y)),
            'area_px': int(max(0.0, (x2 - x1) * (y2 - y1))),
        }


def parse_args():
    default_model = Path(__file__).resolve().parent / 'models' / 'trash_bin.xml'
    parser = argparse.ArgumentParser(
        description='Run rear USB camera dump-bin detection with OpenVINO.',
    )
    parser.add_argument(
        '--camera',
        default='auto',
        help='Camera index, /dev/video path, or "auto" to match --usb-id.',
    )
    parser.add_argument(
        '--usb-id',
        default=DEFAULT_USB_ID,
        help='USB vendor:product id used when --camera=auto.',
    )
    parser.add_argument('--list-cameras', action='store_true')
    parser.add_argument('--model', default=str(default_model))
    parser.add_argument('--ov-device', default=DEFAULT_MODEL_DEVICE)
    parser.add_argument('--conf', type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument('--iou', type=float, default=DEFAULT_IOU)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--no-window', action='store_true')
    parser.add_argument('--print-every', type=float, default=0.5)
    return parser.parse_args()


def list_video_devices():
    devices = []
    for path in sorted(Path('/sys/class/video4linux').glob('video*')):
        name_path = path / 'name'
        name = read_text(name_path) or 'unknown'
        usb_id = find_usb_id(path)
        devices.append({
            'dev': f'/dev/{path.name}',
            'name': name,
            'usb_id': usb_id or 'unknown',
        })
    return devices


def read_text(path):
    try:
        return path.read_text(encoding='utf-8').strip()
    except OSError:
        return ''


def find_usb_id(video_sys_path):
    current = video_sys_path.resolve()
    for parent in (current, *current.parents):
        vendor = read_text(parent / 'idVendor')
        product = read_text(parent / 'idProduct')
        if vendor and product:
            return f'{vendor.lower()}:{product.lower()}'
    return ''


def candidate_camera_paths(camera_arg, usb_id):
    if camera_arg != 'auto':
        return [camera_arg]

    expected = usb_id.lower()
    matches = [
        item['dev']
        for item in list_video_devices()
        if item['usb_id'].lower() == expected
    ]
    if matches:
        return matches
    return [0]


def open_camera(camera_arg, usb_id, width, height):
    last_error = ''
    for candidate in candidate_camera_paths(camera_arg, usb_id):
        source = int(candidate) if str(candidate).isdigit() else candidate
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        if cap.isOpened():
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap, candidate
            last_error = f'opened {candidate}, but no frame was returned'
        else:
            last_error = f'failed to open {candidate}'
        cap.release()
    raise RuntimeError(last_error or 'no usable camera found')


def draw_detection(frame, detection, fps):
    height, width = frame.shape[:2]
    cv2.line(frame, (width // 2, 0), (width // 2, height), (255, 255, 0), 1)
    cv2.line(frame, (0, height // 2), (width, height // 2), (255, 255, 0), 1)
    cv2.putText(
        frame,
        f'fps={fps:.1f}',
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    if detection is None:
        cv2.putText(
            frame,
            'no bin',
            (10, 54),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return frame

    x1, y1, x2, y2 = [int(round(detection[key])) for key in ('x1', 'y1', 'x2', 'y2')]
    center = (
        int(round(detection['center_x_px'])),
        int(round(detection['center_y_px'])),
    )
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, center, 4, (0, 255, 255), -1)
    text = (
        f"bin {detection['confidence']:.2f} "
        f"dx={detection['offset_x_ratio']:.3f} "
        f"dy={detection['offset_y_ratio']:.3f}"
    )
    cv2.putText(
        frame,
        text,
        (max(0, x1), max(24, y1 - 8)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return frame


def detection_text(detection):
    if detection is None:
        return 'no bin detected'
    return (
        f"detected bin conf={detection['confidence']:.3f} "
        f"bbox=({detection['x1']:.1f},{detection['y1']:.1f},"
        f"{detection['x2']:.1f},{detection['y2']:.1f}) "
        f"center=({detection['center_x_px']:.1f},"
        f"{detection['center_y_px']:.1f}) "
        f"offset_ratio=({detection['offset_x_ratio']:.4f},"
        f"{detection['offset_y_ratio']:.4f}) "
        f"area_px={detection['area_px']}"
    )


def main():
    args = parse_args()

    if args.list_cameras:
        for item in list_video_devices():
            print(f"{item['dev']}\t{item['usb_id']}\t{item['name']}")
        return 0

    model_path = Path(args.model).expanduser()
    if not model_path.exists():
        raise FileNotFoundError(f'model not found: {model_path}')

    detector = DumpBinDetector(
        model_path=model_path,
        ov_device=args.ov_device,
        conf_threshold=args.conf,
        iou_threshold=args.iou,
    )
    cap, opened_camera = open_camera(
        args.camera,
        args.usb_id,
        args.width,
        args.height,
    )
    print(
        f'rear camera detector ready: camera={opened_camera}, '
        f'usb_id={args.usb_id}, model={model_path}, ov_device={args.ov_device}'
    )

    last_print = 0.0
    last_time = time.monotonic()
    fps = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print('camera frame read failed')
                time.sleep(0.05)
                continue

            now = time.monotonic()
            dt = now - last_time
            last_time = now
            if dt > 0.0 and math.isfinite(dt):
                fps = 0.85 * fps + 0.15 * (1.0 / dt) if fps > 0.0 else 1.0 / dt

            detections = detector.detect(frame)
            best = detections[0] if detections else None

            if now - last_print >= max(0.05, args.print_every):
                print(detection_text(best))
                last_print = now

            if not args.no_window:
                cv2.imshow('rear camera dump bin detector', draw_detection(frame, best, fps))
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if not args.no_window:
            cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
