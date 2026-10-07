import cv2
import numpy as np

# tflite_runtime may not exist on every Limelight OS build. Fail soft so the
# pipeline still runs and reports the problem through llpython[0] = -1.
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    Interpreter = None

MODEL_PATH = "/home/root/model.tflite"
CONF_THRESHOLD = 0.5      # minimum confidence to keep a detection
MAX_DETECTIONS = 5        # detections sent back
VALUES_PER_DET = 6        # class_id, confidence, cx, cy, w, h
# llpython layout: [count, (class, conf, cx, cy, w, h) * MAX_DETECTIONS]
# cx, cy, w, h are in pixels of the ORIGINAL camera frame.

interpreter = None
input_details = None
output_details = None
in_h = in_w = 0


def runSetup():
    """Runs once at pipeline start: load the model and allocate tensors."""
    global interpreter, input_details, output_details, in_h, in_w
    if Interpreter is None:
        return
    interpreter = Interpreter(model_path=MODEL_PATH)
    interpreter.allocate_tensors()
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    # TFLite input shape is NHWC: [1, height, width, 3]
    shape = input_details[0]["shape"]
    in_h, in_w = int(shape[1]), int(shape[2])


def _prepare_input(frame):
    """Resize to the model's input size and match its dtype/quantization."""
    det = input_details[0]
    resized = cv2.resize(frame, (in_w, in_h))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)  # Limelight frames are BGR
    dtype = det["dtype"]
    if dtype == np.float32:
        data = rgb.astype(np.float32) / 255.0
    else:
        # Quantized model: real = scale * (q - zero_point), with real in 0..1
        scale, zero = det["quantization"]
        if scale == 0:
            data = rgb.astype(dtype)
        else:
            data = np.clip(np.round(rgb / 255.0 / scale + zero),
                           np.iinfo(dtype).min, np.iinfo(dtype).max).astype(dtype)
    return np.expand_dims(data, axis=0)


def _read_output():
    """Return the output as a float array, dequantizing if needed."""
    det = output_details[0]
    out = interpreter.get_tensor(det["index"])
    if det["dtype"] != np.float32:
        scale, zero = det["quantization"]
        if scale != 0:
            out = (out.astype(np.float32) - zero) * scale
    return np.squeeze(out.astype(np.float32), axis=0)


def _parse_detections(out):
    """
    YOLO26 end-to-end (NMS-free) output is [N, 6] = x1, y1, x2, y2, conf, class
    (N is usually 300). Some exports give the transposed [6, N], so handle both.
    Returns a list of (class_id, conf, x1, y1, x2, y2) in 0..1 coordinates.
    """
    if out.ndim != 2:
        return []
    if out.shape[1] != 6 and out.shape[0] == 6:
        out = out.T
    if out.shape[1] != 6:
        # Probably a non-end2end export ([4+nc, N]) that needs NMS. Not handled.
        return []
    dets = []
    for x1, y1, x2, y2, conf, cls in out:
        if conf < CONF_THRESHOLD:
            continue
        # Boxes may be normalized (0..1) or in model-input pixels; normalize both.
        if max(x1, y1, x2, y2) > 1.5:
            x1, x2 = x1 / in_w, x2 / in_w
            y1, y2 = y1 / in_h, y2 / in_h
        dets.append((int(cls), float(conf), x1, y1, x2, y2))
    dets.sort(key=lambda d: d[1], reverse=True)  # highest confidence first
    return dets[:MAX_DETECTIONS]


def runPipeline(image, llrobot):
    """Runs on every frame. Returns (contour, annotated image, llpython)."""
    llpython = [0.0] * (1 + MAX_DETECTIONS * VALUES_PER_DET)
    empty_contour = np.array([[]])

    if interpreter is None:
        llpython[0] = -1  # -1 means the model or tflite_runtime is unavailable
        return empty_contour, image, llpython

    h, w = image.shape[:2]
    interpreter.set_tensor(input_details[0]["index"], _prepare_input(image))
    interpreter.invoke()
    dets = _parse_detections(_read_output())

    llpython[0] = len(dets)
    for i, (cls, conf, x1, y1, x2, y2) in enumerate(dets):
        # Scale back to the original frame size.
        px1, px2 = x1 * w, x2 * w
        py1, py2 = y1 * h, y2 * h
        base = 1 + i * VALUES_PER_DET
        llpython[base:base + VALUES_PER_DET] = [
            cls, conf, (px1 + px2) / 2, (py1 + py2) / 2, px2 - px1, py2 - py1
        ]
        cv2.rectangle(image, (int(px1), int(py1)), (int(px2), int(py2)), (0, 255, 0), 2)
        cv2.putText(image, "%d %.2f" % (cls, conf), (int(px1), max(int(py1) - 5, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # The first detection is also returned as the "primary" contour for the dashboard.
    if dets:
        _, _, x1, y1, x2, y2 = dets[0]
        box = np.array([[[x1 * w, y1 * h]], [[x2 * w, y1 * h]],
                        [[x2 * w, y2 * h]], [[x1 * w, y2 * h]]], dtype=np.int32)
        return box, image, llpython
    return empty_contour, image, llpython
