"""Small OpenVINO YOLO segmentation wrapper."""

import cv2
import numpy as np


class _TensorLike:
    """Compatibility shim for the subset of the ultralytics API we use."""

    __slots__ = ('_arr',)

    def __init__(self, arr):
        self._arr = np.asarray(arr)

    def cpu(self):
        return self

    def numpy(self):
        return self._arr

    def __getitem__(self, idx):
        return self._arr[idx]


class _Box:
    __slots__ = ('xyxy', 'cls', 'conf')

    def __init__(self, x1, y1, x2, y2, cls_id, conf):
        self.xyxy = _TensorLike(
            np.array([[x1, y1, x2, y2]], dtype=np.float32)
        )
        self.cls = _TensorLike(np.array([cls_id], dtype=np.float32))
        self.conf = _TensorLike(np.array([conf], dtype=np.float32))


class _Boxes:
    def __init__(self, boxes):
        self._boxes = boxes

    def __iter__(self):
        return iter(self._boxes)

    def __len__(self):
        return len(self._boxes)


class _Masks:
    def __init__(self, masks, soft_masks=None):
        self.data = masks
        self.soft_data = soft_masks


class _Result:
    def __init__(self, boxes, masks, names, soft_masks=None):
        self.boxes = _Boxes(boxes)
        self.masks = _Masks(masks, soft_masks) if masks is not None else None
        self.names = names


class YOLOSegOpenVINO:
    """OpenVINO YOLO-seg inference returning ultralytics-like results."""

    def __init__(
        self,
        xml_path,
        device='CPU',
        names=None,
        conf_thres=0.45,
        iou_thres=0.65,
        mask_coeff_count=32,
    ):
        import openvino as ov

        core = ov.Core()
        model = core.read_model(str(xml_path))
        self._compiled = core.compile_model(model, device)
        self._input_name = model.input(0).get_any_name()
        self._output_names = [o.get_any_name() for o in model.outputs]
        self._height = int(model.input(0).shape[2])
        self._width = int(model.input(0).shape[3])
        self._conf_thres = float(conf_thres)
        self._iou_thres = float(iou_thres)
        self._mask_coeff_count = int(mask_coeff_count)
        self._names = names or {
            0: 'bottle',
            1: 'pen',
            2: 'paper',
            3: 'charger',
            4: 'usb',
            5: 'yan'
        }

    def __call__(self, img, verbose=False):
        del verbose
        orig_h, orig_w = img.shape[:2]
        blob, scale, pad_w, pad_h, new_h, new_w = self._preprocess(img)

        infer_request = self._compiled.create_infer_request()
        raw = infer_request.infer({self._input_name: blob})
        outputs = [raw[name] for name in self._output_names]
        det_out = next((out for out in outputs if out.ndim == 3), None)
        proto = next((out for out in outputs if out.ndim == 4), None)
        if det_out is None or proto is None:
            return []

        return self._postprocess(
            det_out,
            proto,
            orig_w,
            orig_h,
            scale,
            pad_w,
            pad_h,
            new_h,
            new_w,
        )

    def _preprocess(self, img):
        h, w = img.shape[:2]
        scale = min(self._height / h, self._width / w)
        new_h, new_w = int(h * scale), int(w * scale)
        pad_h = (self._height - new_h) // 2
        pad_w = (self._width - new_w) // 2

        resized = cv2.resize(
            img,
            (new_w, new_h),
            interpolation=cv2.INTER_LINEAR,
        )
        canvas = np.full((self._height, self._width, 3), 114, dtype=np.uint8)
        canvas[pad_h:pad_h + new_h, pad_w:pad_w + new_w] = resized

        blob = canvas.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))[np.newaxis, :, :, :]
        return blob, scale, pad_w, pad_h, new_h, new_w

    @staticmethod
    def _nms(boxes, scores, iou_thres):
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            if order.size == 1:
                break
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[1:][iou <= iou_thres]
        return np.array(keep)

    def _postprocess(
        self,
        det_out,
        proto,
        orig_w,
        orig_h,
        scale,
        pad_w,
        pad_h,
        new_h,
        new_w,
    ):
        nc = det_out.shape[1] - 4 - self._mask_coeff_count
        if nc <= 0:
            return []

        det = np.transpose(det_out, (0, 2, 1))[0]
        boxes, scores, cls_ids, det_idxs = [], [], [], []

        for i, pred in enumerate(det):
            cls_scores = pred[4:4 + nc]
            cls_id = int(np.argmax(cls_scores))
            score = float(cls_scores[cls_id])
            if score < self._conf_thres:
                continue

            xc, yc, bw, bh = pred[0], pred[1], pred[2], pred[3]
            x1 = (xc - bw / 2 - pad_w) / scale
            y1 = (yc - bh / 2 - pad_h) / scale
            x2 = (xc + bw / 2 - pad_w) / scale
            y2 = (yc + bh / 2 - pad_h) / scale
            if x2 <= x1 or y2 <= y1:
                continue

            boxes.append([x1, y1, x2, y2])
            scores.append(score)
            cls_ids.append(cls_id)
            det_idxs.append(i)

        if not boxes:
            return []

        boxes_arr = np.array(boxes, dtype=np.float32)
        scores_arr = np.array(scores, dtype=np.float32)
        indices = self._nms(boxes_arr, scores_arr, self._iou_thres)

        coeffs_list = np.array([
            det[det_idxs[idx], 4 + nc:4 + nc + self._mask_coeff_count]
            for idx in indices
        ])
        if len(coeffs_list) == 0:
            return []

        proto_arr = proto[0]
        masks_raw = proto_arr.reshape(self._mask_coeff_count, -1).T
        masks_raw = masks_raw @ coeffs_list.T
        masks_raw = masks_raw.reshape(
            proto_arr.shape[1],
            proto_arr.shape[2],
            -1,
        )
        masks_raw = 1.0 / (1.0 + np.exp(-masks_raw))

        result_boxes = []
        result_masks = []
        result_soft_masks = []

        for i, idx in enumerate(indices):
            x1, y1, x2, y2 = boxes_arr[idx]
            x1 = max(0, min(orig_w - 1, x1))
            y1 = max(0, min(orig_h - 1, y1))
            x2 = max(0, min(orig_w - 1, x2))
            y2 = max(0, min(orig_h - 1, y2))

            mask = masks_raw[:, :, i]
            mask = cv2.resize(
                mask,
                (self._width, self._height),
                interpolation=cv2.INTER_LINEAR,
            )
            mask = mask[pad_h:pad_h + new_h, pad_w:pad_w + new_w]
            mask = cv2.resize(
                mask,
                (orig_w, orig_h),
                interpolation=cv2.INTER_LINEAR,
            )

            roi = np.zeros((orig_h, orig_w), dtype=np.float32)
            roi[int(y1):int(y2), int(x1):int(x2)] = 1.0
            mask *= roi

            result_boxes.append(
                _Box(x1, y1, x2, y2, cls_ids[idx], scores[idx])
            )
            result_masks.append((mask > 0.5).astype(np.uint8))
            result_soft_masks.append(mask.astype(np.float32))

        return [
            _Result(result_boxes, result_masks, self._names, result_soft_masks)
        ]
