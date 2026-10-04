package robotkit.perception;

import haxe.Int64;
import haxe.io.Bytes;
import robotkit.inference.InferenceSession;
import robotkit.inference.InferenceSession.InferenceResult;
import robotkit.core.SensorFrame;
import visionkit.CameraModel;
import visionkit.ImageView;
import visionkit.UndistortMap;

private class FrameMeta {
  public final frame:SensorFrame;
  public final nativeSequence:Int64;
  public function new(frame:SensorFrame, nativeSequence:Int64) {
    this.frame = frame;
    this.nativeSequence = nativeSequence;
  }
}

/** YOLO-style [cx,cy,w,h,score,class] detector over one rgb8 camera.
 * With calibration, output boxes use ideal pinhole pixels. A rectified-input model
 * uses UndistortMap.rectified intrinsics; otherwise the original K is retained.
 */
class ObjectDetectorPipeline implements PerceptionPipeline {
  final pipelineId:String;
  final inputSensorId:String;
  final producerId:String;
  final modelId:String;
  final modelDigest:String;
  final session:InferenceSession;
  final calibration:Null<CameraModel>;
  final rectification:Null<UndistortMap>;
  final threshold:Float;
  final iouThreshold:Float;
  final targetWidth:Int;
  final targetHeight:Int;
  final maxRateHz:Float;
  final submitted:Array<FrameMeta> = [];
  var lastSubmittedTimeNs:Int64 = Int64.ofInt(0);
  var lastSubmittedClockId:String = "";
  var filtered:Int = 0;
  var carriedDrops:Int = 0;
  var nextNativeSequence:Int64 = Int64.ofInt(1);
  var closed:Bool = false;

  public function new(producerId:String, pipelineId:String, sensorId:String,
      modelId:String, modelPath:String, expectedDigest:String,
      ?scoreThreshold:Float = 0.4, ?iouThreshold:Float = 0.5,
      ?maxRateHz:Float = 0.0, ?threads:Int = 1,
      ?dynamicWidth:Int = 0, ?dynamicHeight:Int = 0,
      ?calibration:CameraModel, ?rectifyInput:Bool = false) {
    if (producerId == null || producerId.length == 0 || pipelineId == null || pipelineId.length == 0 ||
        sensorId == null || sensorId.length == 0 || modelId == null || modelId.length == 0 ||
        modelPath == null || modelPath.length == 0 ||
        !Math.isFinite(scoreThreshold) || scoreThreshold < 0 || scoreThreshold > 1 ||
        !Math.isFinite(iouThreshold) || iouThreshold < 0 || iouThreshold > 1 ||
        !Math.isFinite(maxRateHz) || maxRateHz < 0)
      throw "Object detector needs IDs, a model, and valid thresholds";
    if (rectifyInput && calibration == null)
      throw "Rectified detector input requires calibration";
    this.calibration = calibration;
    this.rectification = rectifyInput ? new UndistortMap(calibration) : null;
    this.producerId = producerId; this.pipelineId = pipelineId;
    this.inputSensorId = sensorId; this.modelId = modelId;
    if (expectedDigest == null || !~/^[0-9a-fA-F]{64}$/.match(expectedDigest))
      throw "Object detector needs a SHA-256 model digest";
    modelDigest = expectedDigest.toLowerCase();
    session = new InferenceSession(modelPath, threads, dynamicWidth, dynamicHeight, modelDigest);
    if (session.inputs.length != 1 || session.outputs.length != 1 ||
        session.inputs[0].elementType != 1 || session.inputs[0].shape.length != 4 ||
        Int64.toInt(session.inputs[0].shape[1]) != 3 ||
        Int64.toInt(session.inputs[0].shape[0]) != 1 ||
        session.outputs[0].shape.length != 3 ||
        Int64.toInt(session.outputs[0].shape[2]) != 6) {
      session.dispose();
      throw "Object detector needs one NCHW float32 input and [N,6] output";
    }
    targetHeight = Int64.toInt(session.inputs[0].shape[2]);
    targetWidth = Int64.toInt(session.inputs[0].shape[3]);
    threshold = scoreThreshold; this.iouThreshold = iouThreshold;
    this.maxRateHz = maxRateHz;
  }
  public function id():String return pipelineId;
  public function sensorId():String return inputSensorId;

  public function submit(frame:SensorFrame):Void {
    if (closed) throw "Object detector is closed";
    if (frame == null || frame.sensorId != inputSensorId) throw "Object detector received the wrong sensor";
    var image = frame.image;
    if (image == null || image.encoding != "rgb8")
      throw "Object detector accepts rgb8 camera frames only";
    if (maxRateHz > 0 && frame.receivedClockId == lastSubmittedClockId &&
        Int64.compare(lastSubmittedTimeNs, Int64.ofInt(0)) > 0 &&
        Int64.compare(frame.receivedTimestampNs, lastSubmittedTimeNs) >= 0) {
      var elapsed = Int64.toFloat(Int64.sub(frame.receivedTimestampNs, lastSubmittedTimeNs));
      if (elapsed < 1000000000.0 / maxRateHz) { filtered++; return; }
    }
    var nativeSequence = nextNativeSequence;
    nextNativeSequence = Int64.add(nextNativeSequence, Int64.ofInt(1));
    if (calibration != null && (image.width != calibration.width || image.height != calibration.height))
      throw "Detector image does not match calibration dimensions";
    var pixels = image.bytes();
    if (rectification != null) {
      var rectified = Bytes.alloc(pixels.length);
      rectification.remap(new ImageView(image.width, image.height, image.width * 3, 1, pixels),
        new ImageView(image.width, image.height, image.width * 3, 1, rectified));
      pixels = rectified;
    }
    session.submitRgb8(nativeSequence, image.width, image.height,
      pixels, targetWidth, targetHeight);
    submitted.push(new FrameMeta(frame, nativeSequence));
    if (submitted.length > 1024) submitted.shift();
    lastSubmittedTimeNs = frame.receivedTimestampNs;
    lastSubmittedClockId = frame.receivedClockId;
  }

  public function poll():Array<ImageDetectionObservation> {
    if (closed) throw "Object detector is closed";
    var ready = session.poll();
    if (ready == null) return [];
    var index = -1;
    for (i in 0...submitted.length)
      if (Int64.compare(submitted[i].nativeSequence, ready.sequence) == 0) { index = i; break; }
    if (index < 0) { carriedDrops += Int64.toInt(ready.dropped); return []; }
    var frame = submitted[index].frame;
    submitted.splice(0, index + 1);
    if (ready.status != 0) {
      carriedDrops += Int64.toInt(ready.dropped) + 1;
      Sys.println('perception pipeline $pipelineId: inference worker status ${ready.status}');
      return [];
    }
    var detections = decode(ready.output, ready, threshold, iouThreshold);
    if (calibration != null && rectification == null)
      detections = undistortBoxes(calibration, detections);
    var dropped = Int64.toInt(ready.dropped) + filtered + carriedDrops;
    filtered = 0; carriedDrops = 0;
    return [new ImageDetectionObservation(producerId, pipelineId, inputSensorId,
      modelId, modelDigest, frame.frameId, frame.sequence,
      frame.sourceTimestampNs, frame.receivedTimestampNs,
      frame.sourceClockId, frame.receivedClockId,
      ready.completedTimestampNs, "robotkit.monotonic", detections, dropped)];
  }

  public function dispose():Void { if (!closed) {
    closed = true; session.dispose();
    if (rectification != null) rectification.dispose();
  } }

  /** Enclosing boxes in ideal pinhole pixels, using the four source corners. */
  public static function undistortBoxes(camera:CameraModel, boxes:Array<ImageDetection>):Array<ImageDetection> {
    if (camera == null || boxes == null) throw "Undistortion needs a camera and boxes";
    var result:Array<ImageDetection> = [];
    for (box in boxes) {
      var rays = camera.unproject([
        {x: box.x, y: box.y}, {x: box.x + box.width, y: box.y},
        {x: box.x, y: box.y + box.height},
        {x: box.x + box.width, y: box.y + box.height}]);
      var left = Math.POSITIVE_INFINITY, top = Math.POSITIVE_INFINITY;
      var right = Math.NEGATIVE_INFINITY, bottom = Math.NEGATIVE_INFINITY;
      for (ray in rays) {
        var x = camera.cx - camera.fx * ray.y / ray.x;
        var y = camera.cy - camera.fy * ray.z / ray.x;
        left = Math.min(left, x); top = Math.min(top, y);
        right = Math.max(right, x); bottom = Math.max(bottom, y);
      }
      left = Math.max(0, Math.min(camera.width, left));
      right = Math.max(0, Math.min(camera.width, right));
      top = Math.max(0, Math.min(camera.height, top));
      bottom = Math.max(0, Math.min(camera.height, bottom));
      if (right > left && bottom > top)
        result.push(new ImageDetection(box.label, box.score, left, top, right - left, bottom - top));
    }
    return result;
  }

  /** Decode and map fixture-style boxes; suppression is per class label. */
  public static function decode(output:Bytes, transform:InferenceResult,
      scoreThreshold:Float, iouThreshold:Float):Array<ImageDetection> {
    if (output.length % 24 != 0 || transform.imageScale <= 0)
      throw "Detector output or letterbox transform is invalid";
    var candidates:Array<ImageDetection> = [];
    for (i in 0...Std.int(output.length / 24)) {
      var base = i * 24;
      var score = output.getFloat(base + 16);
      if (!Math.isFinite(score) || score < scoreThreshold || score > 1) continue;
      var cx = output.getFloat(base), cy = output.getFloat(base + 4);
      var w = output.getFloat(base + 8), h = output.getFloat(base + 12);
      var classValue = output.getFloat(base + 20);
      var classId = Math.round(classValue);
      if (!Math.isFinite(cx) || !Math.isFinite(cy) || !Math.isFinite(w) || !Math.isFinite(h) ||
          !Math.isFinite(classValue) || Math.abs(classValue - classId) > 0.001 ||
          w <= 0 || h <= 0 || classId < 0) continue;
      var left = Math.max(0, Math.min(transform.sourceWidth,
        (cx - w / 2 - transform.padX) / transform.imageScale));
      var top = Math.max(0, Math.min(transform.sourceHeight,
        (cy - h / 2 - transform.padY) / transform.imageScale));
      var right = Math.max(0, Math.min(transform.sourceWidth,
        (cx + w / 2 - transform.padX) / transform.imageScale));
      var bottom = Math.max(0, Math.min(transform.sourceHeight,
        (cy + h / 2 - transform.padY) / transform.imageScale));
      if (right > left && bottom > top)
        candidates.push(new ImageDetection('class-$classId', score, left, top, right - left, bottom - top));
    }
    candidates.sort(function(a, b) return a.score > b.score ? -1 : a.score < b.score ? 1 : 0);
    var accepted:Array<ImageDetection> = [];
    for (candidate in candidates) {
      var suppressed = false;
      for (prior in accepted)
        if (prior.label == candidate.label && overlap(prior, candidate) > iouThreshold)
          { suppressed = true; break; }
      if (!suppressed) accepted.push(candidate);
    }
    return accepted;
  }

  static function overlap(a:ImageDetection, b:ImageDetection):Float {
    var width = Math.max(0, Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x));
    var height = Math.max(0, Math.min(a.y + a.height, b.y + b.height) - Math.max(a.y, b.y));
    var intersection = width * height;
    return intersection / (a.width * a.height + b.width * b.height - intersection);
  }
}
