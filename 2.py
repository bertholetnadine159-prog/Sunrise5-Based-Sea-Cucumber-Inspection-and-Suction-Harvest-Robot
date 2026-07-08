#!/usr/bin/env python3

import argparse
import logging
import os
import re
from time import time

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn


logging.basicConfig(
    level=logging.DEBUG,
    format="[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("RDK_YOLO11_SEG")


sea_cucumber_names = ["sea_cucumber"]
image_suffixes = (".jpg", ".jpeg", ".png", ".bmp", ".webp")

rdk_colors = [
    (56, 56, 255),
    (151, 157, 255),
    (31, 112, 255),
    (29, 178, 255),
    (49, 210, 207),
    (10, 249, 72),
    (23, 204, 146),
    (134, 219, 61),
    (52, 147, 26),
    (187, 212, 0),
    (168, 153, 44),
    (255, 194, 0),
    (147, 69, 52),
    (255, 115, 100),
    (236, 24, 0),
    (255, 56, 132),
    (133, 0, 82),
    (255, 56, 203),
    (200, 149, 255),
    (199, 55, 255),
]


def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x: np.ndarray, axis: int = -1) -> np.ndarray:
    x = x - np.max(x, axis=axis, keepdims=True)
    exp_x = np.exp(x)
    return exp_x / np.sum(exp_x, axis=axis, keepdims=True)


class BaseModel:
    def __init__(self, model_file: str) -> None:
        try:
            begin_time = time()
            self.quantize_model = dnn.load(model_file)
            logger.debug("Load D-Robotics quantized model time = %.2f ms", 1000 * (time() - begin_time))
        except Exception as e:
            logger.error("Failed to load model file: %s", model_file)
            logger.error(e)
            raise

        logger.info("-> input tensors")
        for i, quantize_input in enumerate(self.quantize_model[0].inputs):
            logger.info(
                "input[%d], name=%s, type=%s, shape=%s",
                i,
                quantize_input.name,
                quantize_input.properties.dtype,
                quantize_input.properties.shape,
            )

        logger.info("-> output tensors")
        for i, quantize_output in enumerate(self.quantize_model[0].outputs):
            logger.info(
                "output[%d], name=%s, type=%s, shape=%s",
                i,
                quantize_output.name,
                quantize_output.properties.dtype,
                quantize_output.properties.shape,
            )

        input_shape = self.quantize_model[0].inputs[0].properties.shape
        if len(input_shape) == 4 and input_shape[1] in (1, 3):
            self.model_input_height, self.model_input_width = input_shape[2:4]
        else:
            self.model_input_height, self.model_input_width = input_shape[1:3]
        self.input_format = "nv12"
        logger.info(
            "runtime input format: %s, image size: %dx%d",
            self.input_format,
            self.model_input_width,
            self.model_input_height,
        )

    def resizer(self, img: np.ndarray) -> np.ndarray:
        img_h, img_w = img.shape[:2]
        self.orig_h, self.orig_w = img_h, img_w

        gain = min(self.model_input_width / img_w, self.model_input_height / img_h)
        new_w = int(round(img_w * gain))
        new_h = int(round(img_h * gain))
        pad_w = self.model_input_width - new_w
        pad_h = self.model_input_height - new_h

        left = int(round(pad_w / 2 - 0.1))
        right = int(round(pad_w / 2 + 0.1))
        top = int(round(pad_h / 2 - 0.1))
        bottom = int(round(pad_h / 2 + 0.1))

        resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        letterboxed = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )

        self.gain = gain
        self.pad_left = left
        self.pad_top = top
        self.pad_right = right
        self.pad_bottom = bottom
        return letterboxed

    def bgr2nv12(self, bgr_img: np.ndarray) -> np.ndarray:
        begin_time = time()
        bgr_img = self.resizer(bgr_img)
        height, width = bgr_img.shape[:2]
        area = height * width

        yuv420p = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))

        nv12 = np.zeros_like(yuv420p)
        nv12[:area] = y
        nv12[area:] = uv_packed

        expected_size = self.model_input_height * self.model_input_width * 3 // 2
        if nv12.size != expected_size:
            raise ValueError(f"NV12 input size mismatch: got {nv12.size}, expected {expected_size}")

        logger.debug("bgr8 to nv12 time = %.2f ms", 1000 * (time() - begin_time))
        return nv12

    def forward(self, input_tensor: np.ndarray):
        begin_time = time()
        outputs = self.quantize_model[0].forward(input_tensor)
        self.last_forward_ms = 1000 * (time() - begin_time)
        logger.debug("forward time = %.2f ms", self.last_forward_ms)
        return outputs

    def c2numpy(self, outputs) -> list[np.ndarray]:
        begin_time = time()
        outputs = [dnn_tensor.buffer for dnn_tensor in outputs]
        logger.debug("c to numpy time = %.2f ms", 1000 * (time() - begin_time))
        return outputs


class YOLO11_Segment(BaseModel):
    def __init__(
        self,
        model_file: str,
        conf: float,
        iou: float,
        classes_num: int = 1,
        reg: int = 16,
        mask_dim: int = 32,
        mask_thres: float = 0.5,
    ):
        super().__init__(model_file)

        self.conf = conf
        self.iou = iou
        self.classes_num = classes_num
        self.reg = reg
        self.mask_dim = mask_dim
        self.mask_thres = mask_thres
        self.conf_inverse = -np.log(1 / conf - 1)

        self.weights_static = np.arange(reg, dtype=np.float32)[np.newaxis, np.newaxis, :]
        self.strides = [8, 16, 32]
        self.anchors = [
            self.make_anchor(self.model_input_width // 8, self.model_input_height // 8),
            self.make_anchor(self.model_input_width // 16, self.model_input_height // 16),
            self.make_anchor(self.model_input_width // 32, self.model_input_height // 32),
        ]

        logger.info("anchors: %s, %s, %s", self.anchors[0].shape, self.anchors[1].shape, self.anchors[2].shape)
        logger.info("iou threshold = %.2f, conf threshold = %.2f", iou, conf)
        logger.info("sigmoid inverse threshold = %.2f", self.conf_inverse)

    @staticmethod
    def make_anchor(grid_w: int, grid_h: int) -> np.ndarray:
        return np.stack(
            [
                np.tile(np.linspace(0.5, grid_w - 0.5, grid_w), reps=grid_h),
                np.repeat(np.arange(0.5, grid_h + 0.5, 1), grid_w),
            ],
            axis=0,
        ).transpose(1, 0).astype(np.float32)

    def split_outputs(self, outputs: list[np.ndarray]):
        # Your model exports 4 tensors:
        # p3/p4/p5: NHWC, last dim 97 = 64 bbox + 1 class + 32 mask coeff.
        # proto: NHWC, last dim 32.
        if len(outputs) == 4:
            return outputs[0:3], outputs[3]

        # Compatibility path for models exported as separated bbox/cls/mask heads.
        if len(outputs) == 10:
            bboxes = outputs[0:3]
            clses = outputs[3:6]
            mask_coeffs = outputs[6:9]
            proto = outputs[9]
            heads = []
            for i in range(3):
                heads.append(
                    np.concatenate(
                        [
                            bboxes[i].reshape(-1, 4 * self.reg),
                            clses[i].reshape(-1, self.classes_num),
                            mask_coeffs[i].reshape(-1, self.mask_dim),
                        ],
                        axis=1,
                    )
                )
            return heads, proto

        raise ValueError(
            "Unsupported YOLO11-seg output count. Expected 4 outputs "
            "(p3, p4, p5, proto) or 10 split outputs. "
            f"Current output count is {len(outputs)}."
        )

    def decode_one_scale(self, head_output: np.ndarray, anchor: np.ndarray, stride: int):
        expect_dim = 4 * self.reg + self.classes_num + self.mask_dim
        head = head_output.reshape(-1, expect_dim)
        bboxes = head[:, :4 * self.reg]
        clses = head[:, 4 * self.reg:4 * self.reg + self.classes_num]
        masks = head[:, 4 * self.reg + self.classes_num:]

        if masks.shape[1] != self.mask_dim:
            raise ValueError(f"Mask coeff dim mismatch: got {masks.shape[1]}, expected {self.mask_dim}")

        max_scores = np.max(clses, axis=1)
        valid_indices = np.flatnonzero(max_scores >= self.conf_inverse)
        if len(valid_indices) == 0:
            return (
                np.empty((0, 4), dtype=np.float32),
                np.empty((0,), dtype=np.float32),
                np.empty((0,), dtype=np.int32),
                np.empty((0, self.mask_dim), dtype=np.float32),
            )

        ids = np.argmax(clses[valid_indices, :], axis=1).astype(np.int32)
        scores = sigmoid(max_scores[valid_indices]).astype(np.float32)

        valid_bboxes = bboxes[valid_indices, :].astype(np.float32)
        ltrb = np.sum(
            softmax(valid_bboxes.reshape(-1, 4, self.reg), axis=2) * self.weights_static,
            axis=2,
        )

        valid_anchor = anchor[valid_indices, :]
        x1y1 = valid_anchor - ltrb[:, 0:2]
        x2y2 = valid_anchor + ltrb[:, 2:4]
        dbboxes = np.hstack([x1y1, x2y2]) * stride

        return dbboxes, scores, ids, masks[valid_indices, :].astype(np.float32)

    def format_proto(self, proto: np.ndarray) -> np.ndarray:
        proto = np.squeeze(proto)
        if proto.ndim != 3:
            raise ValueError(f"Proto output shape error: {proto.shape}")

        if proto.shape[0] == self.mask_dim:
            proto = np.transpose(proto, (1, 2, 0))
        elif proto.shape[-1] != self.mask_dim:
            mask_axis = int(np.argmin(np.abs(np.array(proto.shape) - self.mask_dim)))
            proto = np.moveaxis(proto, mask_axis, -1)

        return proto.astype(np.float32)

    def crop_mask(self, mask: np.ndarray, bbox: np.ndarray) -> np.ndarray:
        x1, y1, x2, y2 = bbox.astype(np.int32)
        x1 = int(np.clip(x1, 0, self.orig_w))
        x2 = int(np.clip(x2, 0, self.orig_w))
        y1 = int(np.clip(y1, 0, self.orig_h))
        y2 = int(np.clip(y2, 0, self.orig_h))

        cropped = np.zeros_like(mask, dtype=np.uint8)
        if x2 > x1 and y2 > y1:
            cropped[y1:y2, x1:x2] = mask[y1:y2, x1:x2]
        return cropped

    def scale_boxes_to_original(self, boxes: np.ndarray) -> np.ndarray:
        boxes = boxes.copy()
        boxes[:, [0, 2]] -= self.pad_left
        boxes[:, [1, 3]] -= self.pad_top
        boxes[:, :4] /= self.gain
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, self.orig_w - 1)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, self.orig_h - 1)
        return boxes

    def decode_masks(self, proto: np.ndarray, mask_coeffs: np.ndarray, bboxes: np.ndarray) -> np.ndarray:
        if len(mask_coeffs) == 0:
            return np.empty((0, self.orig_h, self.orig_w), dtype=np.uint8)

        proto = self.format_proto(proto)
        proto_h, proto_w, proto_c = proto.shape
        if proto_c != mask_coeffs.shape[1]:
            raise ValueError(f"Mask dim mismatch: proto={proto.shape}, coeff={mask_coeffs.shape}")

        masks = sigmoid(mask_coeffs @ proto.reshape(-1, proto_c).T)
        masks = masks.reshape(-1, proto_h, proto_w)

        result = []
        for mask, bbox in zip(masks, bboxes):
            mask = cv2.resize(
                mask,
                (self.model_input_width, self.model_input_height),
                interpolation=cv2.INTER_LINEAR,
            )
            y1 = self.pad_top
            y2 = self.model_input_height - self.pad_bottom
            x1 = self.pad_left
            x2 = self.model_input_width - self.pad_right
            mask = mask[y1:y2, x1:x2]
            mask = cv2.resize(mask, (self.orig_w, self.orig_h), interpolation=cv2.INTER_LINEAR)
            mask = (mask >= self.mask_thres).astype(np.uint8)
            result.append(self.crop_mask(mask, bbox))

        return np.array(result, dtype=np.uint8)

    def postProcess(self, outputs: list[np.ndarray]):
        begin_time = time()
        head_outputs, proto = self.split_outputs(outputs)

        all_bboxes = []
        all_scores = []
        all_ids = []
        all_coeffs = []
        for i in range(3):
            bboxes, scores, ids, coeffs = self.decode_one_scale(
                head_outputs[i],
                self.anchors[i],
                self.strides[i],
            )
            all_bboxes.append(bboxes)
            all_scores.append(scores)
            all_ids.append(ids)
            all_coeffs.append(coeffs)

        dbboxes = np.concatenate(all_bboxes, axis=0)
        scores = np.concatenate(all_scores, axis=0)
        ids = np.concatenate(all_ids, axis=0)
        coeffs = np.concatenate(all_coeffs, axis=0)

        if len(dbboxes) == 0:
            logger.info("No object detected.")
            return ids, scores, dbboxes.astype(np.int32), np.empty((0, self.orig_h, self.orig_w), dtype=np.uint8)

        nms_boxes = dbboxes.copy()
        nms_boxes[:, 2:4] = nms_boxes[:, 2:4] - nms_boxes[:, 0:2]
        indices = cv2.dnn.NMSBoxes(nms_boxes.tolist(), scores.tolist(), self.conf, self.iou)
        if len(indices) == 0:
            logger.info("No object kept after NMS.")
            return (
                np.empty((0,), dtype=np.int32),
                np.empty((0,), dtype=np.float32),
                np.empty((0, 4), dtype=np.int32),
                np.empty((0, self.orig_h, self.orig_w), dtype=np.uint8),
            )

        indices = np.array(indices).flatten()
        bboxes = self.scale_boxes_to_original(dbboxes[indices])
        bboxes = bboxes.astype(np.int32)

        ids = ids[indices]
        scores = scores[indices]
        masks = self.decode_masks(proto, coeffs[indices], bboxes)

        logger.debug("Post Process time = %.2f ms", 1000 * (time() - begin_time))
        return ids, scores, bboxes, masks


def draw_segmentation(
    img: np.ndarray,
    bbox: tuple[int, int, int, int],
    mask: np.ndarray,
    score: float,
    class_id: int,
    class_names: list[str],
) -> None:
    color = rdk_colors[class_id % len(rdk_colors)]
    x1, y1, x2, y2 = bbox

    color_mask = np.zeros_like(img, dtype=np.uint8)
    color_mask[mask > 0] = color
    img[:] = cv2.addWeighted(img, 1.0, color_mask, 0.45, 0)

    cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
    label = f"{class_names[class_id]}: {score:.2f}"
    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    label_x = x1
    label_y = y1 - 10 if y1 - 10 > label_height else y1 + label_height + 10
    cv2.rectangle(
        img,
        (label_x, label_y - label_height - 4),
        (label_x + label_width, label_y + 4),
        color,
        cv2.FILLED,
    )
    cv2.putText(img, label, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def list_images(path: str) -> list[str]:
    if os.path.isfile(path):
        return [path]

    if not os.path.isdir(path):
        raise ValueError(f"Input path is not a file or directory: {path}")

    image_paths = []
    for name in sorted(os.listdir(path)):
        full_path = os.path.join(path, name)
        if os.path.isfile(full_path) and name.lower().endswith(image_suffixes):
            image_paths.append(full_path)

    if not image_paths:
        raise ValueError(f"No images found in directory: {path}")

    return image_paths


def find_label_path(image_path: str, label_dir: str) -> str:
    stem = os.path.splitext(os.path.basename(image_path))[0]
    return os.path.join(label_dir, stem + ".txt")


def load_yolo_seg_labels(label_path: str, img_h: int, img_w: int) -> list[dict]:
    if not os.path.exists(label_path):
        return []

    labels = []
    with open(label_path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 7:
                continue

            class_id = int(float(parts[0]))
            coords = np.array([float(x) for x in parts[1:]], dtype=np.float32)
            if len(coords) % 2 != 0:
                coords = coords[:-1]
            if len(coords) < 6:
                continue

            points = coords.reshape(-1, 2)
            points[:, 0] = np.clip(points[:, 0] * img_w, 0, img_w - 1)
            points[:, 1] = np.clip(points[:, 1] * img_h, 0, img_h - 1)
            polygon = points.astype(np.int32)

            mask = np.zeros((img_h, img_w), dtype=np.uint8)
            cv2.fillPoly(mask, [polygon], 1)
            labels.append({"class_id": class_id, "mask": mask})

    return labels


def mask_iou(mask_a: np.ndarray, mask_b: np.ndarray) -> float:
    intersection = np.logical_and(mask_a > 0, mask_b > 0).sum()
    union = np.logical_or(mask_a > 0, mask_b > 0).sum()
    if union == 0:
        return 0.0
    return float(intersection / union)


def compute_ap50(predictions: list[dict], ground_truths: dict, iou_thres: float = 0.5) -> tuple[float, float, float]:
    total_gt = sum(len(items) for items in ground_truths.values())
    if total_gt == 0:
        return 0.0, 0.0, 0.0

    predictions = sorted(predictions, key=lambda item: item["score"], reverse=True)
    matched = {image_path: np.zeros(len(items), dtype=bool) for image_path, items in ground_truths.items()}

    tp = np.zeros(len(predictions), dtype=np.float32)
    fp = np.zeros(len(predictions), dtype=np.float32)

    for pred_index, pred in enumerate(predictions):
        gts = ground_truths.get(pred["image_path"], [])
        best_iou = 0.0
        best_gt_index = -1

        for gt_index, gt in enumerate(gts):
            if matched[pred["image_path"]][gt_index]:
                continue
            if pred["class_id"] != gt["class_id"]:
                continue

            iou = mask_iou(pred["mask"], gt["mask"])
            if iou > best_iou:
                best_iou = iou
                best_gt_index = gt_index

        if best_iou >= iou_thres and best_gt_index >= 0:
            tp[pred_index] = 1.0
            matched[pred["image_path"]][best_gt_index] = True
        else:
            fp[pred_index] = 1.0

    total_tp = float(tp.sum())
    total_fp = float(fp.sum())
    precision = total_tp / (total_tp + total_fp + 1e-16)
    recall = total_tp / (total_gt + 1e-16)

    if len(predictions) == 0:
        return precision, recall, 0.0

    tp_cum = np.cumsum(tp)
    fp_cum = np.cumsum(fp)
    recall_curve = tp_cum / (total_gt + 1e-16)
    precision_curve = tp_cum / (tp_cum + fp_cum + 1e-16)

    ap = 0.0
    for recall_level in np.linspace(0.0, 1.0, 101):
        valid = precision_curve[recall_curve >= recall_level]
        ap += (valid.max() if valid.size else 0.0) / 101.0

    return precision, recall, float(ap)


def print_metrics_table(precision: float, recall: float, map50: float, fps: float) -> None:
    headers = ["P", "R", "mAP50", "FPS"]
    values = [f"{precision:.4f}", f"{recall:.4f}", f"{map50:.4f}", f"{fps:.2f}"]
    widths = [max(len(header), len(value)) for header, value in zip(headers, values)]
    border = "+" + "+".join("-" * (width + 2) for width in widths) + "+"
    header_row = "|" + "|".join(f" {header:^{width}} " for header, width in zip(headers, widths)) + "|"
    value_row = "|" + "|".join(f" {value:^{width}} " for value, width in zip(values, widths)) + "|"

    print(border)
    print(header_row)
    print(border)
    print(value_row)
    print(border)


def read_onnx_path_from_yaml(yaml_path: str) -> str | None:
    if not yaml_path or not os.path.exists(yaml_path):
        return None

    with open(yaml_path, "r", encoding="utf-8", errors="ignore") as f:
        text = f.read()

    match = re.search(r"onnx_model\s*:\s*['\"]?([^'\"\n\r#]+)", text)
    if not match:
        return None

    onnx_path = match.group(1).strip()
    if not os.path.isabs(onnx_path):
        onnx_path = os.path.join(os.path.dirname(yaml_path), onnx_path)
    return onnx_path


def get_onnx_shape(value_name: str, shape_map: dict) -> list[int] | None:
    shape = shape_map.get(value_name)
    if shape is None or any(dim is None or dim <= 0 for dim in shape):
        return None
    return shape


def estimate_onnx_compute(onnx_path: str) -> dict:
    try:
        import onnx
        from onnx import shape_inference
    except ImportError as exc:
        raise RuntimeError("ONNX compute estimation needs the python package: onnx") from exc

    if not os.path.exists(onnx_path):
        raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

    model = onnx.load(onnx_path)
    model = shape_inference.infer_shapes(model)
    graph = model.graph

    initializer_shapes = {init.name: list(init.dims) for init in graph.initializer}
    shape_map = {}
    for value in list(graph.input) + list(graph.value_info) + list(graph.output):
        dims = []
        tensor_type = value.type.tensor_type
        if not tensor_type.HasField("shape"):
            continue
        for dim in tensor_type.shape.dim:
            if dim.HasField("dim_value"):
                dims.append(int(dim.dim_value))
            else:
                dims.append(None)
        shape_map[value.name] = dims

    conv_macs = 0
    gemm_macs = 0
    matmul_macs = 0
    counted_nodes = 0

    for node in graph.node:
        if node.op_type == "Conv":
            if len(node.input) < 2 or not node.output:
                continue
            weight_shape = initializer_shapes.get(node.input[1])
            output_shape = get_onnx_shape(node.output[0], shape_map)
            if not weight_shape or not output_shape or len(weight_shape) != 4 or len(output_shape) != 4:
                continue

            group = 1
            for attr in node.attribute:
                if attr.name == "group":
                    group = max(1, int(attr.i))

            out_n, out_c, out_h, out_w = output_shape
            _, in_c_per_group, kernel_h, kernel_w = weight_shape
            conv_macs += out_n * out_c * out_h * out_w * in_c_per_group * kernel_h * kernel_w
            counted_nodes += 1

        elif node.op_type == "Gemm":
            if len(node.input) < 2:
                continue
            a_shape = get_onnx_shape(node.input[0], shape_map)
            b_shape = initializer_shapes.get(node.input[1]) or get_onnx_shape(node.input[1], shape_map)
            if not a_shape or not b_shape or len(a_shape) < 2 or len(b_shape) < 2:
                continue
            m = int(np.prod(a_shape[:-1]))
            k = a_shape[-1]
            n = b_shape[0]
            for attr in node.attribute:
                if attr.name == "transB" and int(attr.i) == 0:
                    n = b_shape[-1]
            gemm_macs += m * n * k
            counted_nodes += 1

        elif node.op_type == "MatMul":
            if len(node.input) < 2:
                continue
            a_shape = get_onnx_shape(node.input[0], shape_map)
            b_shape = initializer_shapes.get(node.input[1]) or get_onnx_shape(node.input[1], shape_map)
            if not a_shape or not b_shape or len(a_shape) < 2 or len(b_shape) < 2:
                continue
            m = int(np.prod(a_shape[:-1]))
            k = a_shape[-1]
            n = b_shape[-1]
            matmul_macs += m * n * k
            counted_nodes += 1

    total_macs = conv_macs + gemm_macs + matmul_macs
    return {
        "nodes": counted_nodes,
        "macs": total_macs,
        "gmacs": total_macs / 1e9,
        "gflops": total_macs * 2 / 1e9,
    }


def print_compute_estimate(opt) -> None:
    onnx_path = opt.onnx_model or read_onnx_path_from_yaml(opt.model_yaml)
    if not onnx_path:
        logger.warning("Compute estimate skipped: provide --onnx-model or a model.yaml with onnx_model.")
        return

    try:
        estimate = estimate_onnx_compute(onnx_path)
    except Exception as exc:
        logger.warning("Compute estimate failed: %s", exc)
        return

    print("+------------------+----------------+")
    print("| Item             | Estimate       |")
    print("+------------------+----------------+")
    print(f"| Counted nodes    | {estimate['nodes']:>14d} |")
    print(f"| MACs             | {estimate['macs']:>14.0f} |")
    print(f"| GMACs            | {estimate['gmacs']:>14.4f} |")
    print(f"| GFLOPs           | {estimate['gflops']:>14.4f} |")
    print("+------------------+----------------+")


def print_bin_benchmark_table(image_count: int, forward_ms: float, end_to_end_ms: float) -> None:
    avg_forward_ms = forward_ms / image_count
    avg_end_to_end_ms = end_to_end_ms / image_count
    forward_fps = 1000 * image_count / forward_ms
    end_to_end_fps = 1000 * image_count / end_to_end_ms

    print("+----------------------+----------------+")
    print("| Item                 | BIN Result     |")
    print("+----------------------+----------------+")
    print(f"| Images               | {image_count:>14d} |")
    print(f"| Forward avg ms       | {avg_forward_ms:>14.2f} |")
    print(f"| Forward FPS          | {forward_fps:>14.2f} |")
    print(f"| End-to-end avg ms    | {avg_end_to_end_ms:>14.2f} |")
    print(f"| End-to-end FPS       | {end_to_end_fps:>14.2f} |")
    print("+----------------------+----------------+")


def parse_interested_nodes(value: str | None) -> list[str] | None:
    if value is None or value.strip() == "":
        return None
    return [item.strip() for item in value.split(",") if item.strip()]


def run_quant_sensitivity_debug(opt) -> None:
    try:
        import horizon_nn.debug as dbg
    except ImportError as exc:
        raise RuntimeError("horizon_nn.debug is required for --debug-sensitivity") from exc

    logging.getLogger().setLevel(logging.INFO)
    node_message = dbg.get_sensitivity_of_nodes(
        model_or_file=opt.debug_model,
        metrics=["cosine-similarity", "mse"],
        calibrated_data=opt.debug_calibrated_data,
        output_node=opt.debug_output_node,
        node_type=opt.debug_node_type,
        data_num=opt.debug_data_num,
        verbose=opt.debug_verbose,
        interested_nodes=parse_interested_nodes(opt.debug_interested_nodes),
    )
    print(node_message)


def build_save_path(input_path: str, opt) -> str:
    if os.path.isdir(opt.test_img):
        save_dir = opt.output_dir or opt.img_save_path
        os.makedirs(save_dir, exist_ok=True)
        name, ext = os.path.splitext(os.path.basename(input_path))
        return os.path.join(save_dir, f"{name}_seg{ext or '.jpg'}")

    if os.path.isdir(opt.img_save_path) or not os.path.splitext(opt.img_save_path)[1]:
        os.makedirs(opt.img_save_path, exist_ok=True)
        name, ext = os.path.splitext(os.path.basename(input_path))
        return os.path.join(opt.img_save_path, f"{name}_seg{ext or '.jpg'}")

    if os.path.isfile(opt.test_img):
        return opt.img_save_path

    raise ValueError(f"Unsupported input path: {opt.test_img}")


def resolve_dataset_paths(opt) -> None:
    if opt.data_dir is None:
        return

    images_dir = os.path.join(opt.data_dir, "images")
    labels_dir = os.path.join(opt.data_dir, "labels")
    if not os.path.isdir(images_dir):
        raise ValueError(f"Images directory not found: {images_dir}")
    if not os.path.isdir(labels_dir):
        raise ValueError(f"Labels directory not found: {labels_dir}")

    opt.test_img = images_dir
    opt.label_dir = labels_dir
    if opt.img_save_path == "seg_results" and opt.output_dir is None:
        opt.img_save_path = "seg_results"


def infer_one_image(model: YOLO11_Segment, image_path: str, save_path: str):
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Failed to read image: {image_path}")

    begin_time = time()
    input_tensor = model.bgr2nv12(img)
    outputs = model.c2numpy(model.forward(input_tensor))
    ids, scores, bboxes, masks = model.postProcess(outputs)
    total_ms = 1000 * (time() - begin_time)

    logger.info("Draw Results: %s", image_path)
    for class_id, score, bbox, mask in zip(ids, scores, bboxes, masks):
        x1, y1, x2, y2 = bbox
        logger.info(
            "(%d, %d, %d, %d) -> %s: %.2f, mask_area=%d",
            x1,
            y1,
            x2,
            y2,
            sea_cucumber_names[class_id],
            score,
            int(mask.sum()),
        )
        draw_segmentation(img, (x1, y1, x2, y2), mask, score, class_id, sea_cucumber_names)

    cv2.imwrite(save_path, img)
    logger.info('saved in path: "%s"', save_path)
    prediction_items = []
    for class_id, score, mask in zip(ids, scores, masks):
        prediction_items.append(
            {
                "image_path": image_path,
                "class_id": int(class_id),
                "score": float(score),
                "mask": mask,
            }
        )

    return total_ms, model.last_forward_ms, len(bboxes), prediction_items, img.shape[:2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", type=str, default="YOLO11_LBL.bin")
    parser.add_argument("--data-dir", type=str, default="valid", help="Dataset root with images/ and labels/, for example valid.")
    parser.add_argument("--test-img", type=str, default="sea_cucumber", help="Image file or image directory.")
    parser.add_argument(
        "--img-save-path",
        type=str,
        default="seg_results",
        help="Output file for one image, or output directory for an image directory.",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Deprecated. If set, it overrides --img-save-path for directory input.",
    )
    parser.add_argument("--classes-num", type=int, default=1)
    parser.add_argument("--reg", type=int, default=16)
    parser.add_argument("--mask-dim", type=int, default=32)
    parser.add_argument("--mask-thres", type=float, default=0.5)
    parser.add_argument("--iou-thres", type=float, default=0.45)
    parser.add_argument("--conf-thres", type=float, default=0.15, help="Score threshold. Try 0.005 for very low-score recall testing.")
    parser.add_argument("--label-dir", type=str, default=None, help="YOLO segmentation label directory for P/R/mAP50.")
    parser.add_argument("--eval-iou-thres", type=float, default=0.5, help="IoU threshold for P/R/mAP50.")
    parser.add_argument("--estimate-compute", action="store_true", help="Estimate model compute from ONNX and exit.")
    parser.add_argument("--benchmark-bin", action="store_true", help="Benchmark actual BIN inference speed.")
    parser.add_argument("--onnx-model", type=str, default=None, help="ONNX model path used for compute estimation.")
    parser.add_argument("--model-yaml", type=str, default="model.yaml", help="YAML file containing onnx_model.")
    parser.add_argument("--debug-sensitivity", action="store_true", help="Run horizon_nn quantization sensitivity debug and exit.")
    parser.add_argument("--debug-model", type=str, default="./calibrated_model.onnx", help="Calibrated ONNX model for debug.")
    parser.add_argument("--debug-calibrated-data", type=str, default="./calibration_data/", help="Calibration data directory.")
    parser.add_argument("--debug-output-node", type=str, default=None, help="Output node name for sensitivity debug.")
    parser.add_argument("--debug-node-type", type=str, default="node", choices=["node", "layer"], help="Debug node type.")
    parser.add_argument("--debug-data-num", type=int, default=None, help="Number of calibration samples to use.")
    parser.add_argument("--debug-verbose", action="store_true", default=True, help="Print verbose sensitivity debug logs.")
    parser.add_argument("--debug-interested-nodes", type=str, default=None, help="Comma-separated node names to debug.")
    opt = parser.parse_args()

    if opt.estimate_compute:
        logger.info(opt)
        print_compute_estimate(opt)
        return

    if opt.debug_sensitivity:
        logger.info(opt)
        run_quant_sensitivity_debug(opt)
        return

    resolve_dataset_paths(opt)
    logger.info(opt)

    model = YOLO11_Segment(
        opt.model_path,
        opt.conf_thres,
        opt.iou_thres,
        classes_num=opt.classes_num,
        reg=opt.reg,
        mask_dim=opt.mask_dim,
        mask_thres=opt.mask_thres,
    )

    image_paths = list_images(opt.test_img)
    logger.info("Found %d image(s).", len(image_paths))

    total_begin = time()
    total_image_ms = 0.0
    total_forward_ms = 0.0
    total_objects = 0
    labeled_image_ms = 0.0
    labeled_forward_ms = 0.0
    labeled_image_count = 0
    all_predictions = []
    all_ground_truths = {}

    for image_path in image_paths:
        save_path = build_save_path(image_path, opt)
        image_ms, forward_ms, object_count, predictions, image_shape = infer_one_image(model, image_path, save_path)
        total_image_ms += image_ms
        total_forward_ms += forward_ms
        total_objects += object_count
        all_predictions.extend(predictions)

        if opt.label_dir is not None:
            img_h, img_w = image_shape
            label_path = find_label_path(image_path, opt.label_dir)
            labels = load_yolo_seg_labels(label_path, img_h, img_w)
            all_ground_truths[image_path] = labels
            if os.path.exists(label_path):
                labeled_image_ms += image_ms
                labeled_forward_ms += forward_ms
                labeled_image_count += 1

    wall_ms = 1000 * (time() - total_begin)
    image_count = len(image_paths)
    logger.info("Processed images: %d, objects: %d", image_count, total_objects)

    if opt.label_dir is not None and labeled_image_count > 0:
        end_to_end_fps = 1000 * labeled_image_count / labeled_image_ms
        logger.info("Labeled images for FPS: %d", labeled_image_count)
        logger.info("End-to-end average time: %.2f ms/labeled image, FPS: %.2f", labeled_image_ms / labeled_image_count, end_to_end_fps)
        logger.info(
            "Forward average time: %.2f ms/labeled image, FPS: %.2f",
            labeled_forward_ms / labeled_image_count,
            1000 * labeled_image_count / labeled_forward_ms,
        )
    else:
        end_to_end_fps = 0.0
        logger.info("FPS skipped: no labeled images were provided.")

    if opt.benchmark_bin and labeled_image_count > 0:
        print_bin_benchmark_table(labeled_image_count, labeled_forward_ms, labeled_image_ms)

    if opt.label_dir is not None:
        precision, recall, map50 = compute_ap50(all_predictions, all_ground_truths, opt.eval_iou_thres)
        print_metrics_table(precision, recall, map50, end_to_end_fps)


if __name__ == "__main__":
    main()
