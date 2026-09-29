package robotkit.inference;

import RobotKitInference;
import RobotKitRuntime;
import haxe.Int64;
import haxe.io.Bytes;

typedef InferenceTensor = {
  name:String,
  elementType:Int,
  shape:Array<Int64>,
  byteOffset:Int64,
  byteCount:Int64
};

class InferenceResult {
  public final sequence:Int64;
  public final dropped:Int64;
  public final completedTimestampNs:Int64;
  public final output:Bytes;
  public final sourceWidth:Int;
  public final sourceHeight:Int;
  public final imageScale:Float;
  public final padX:Float;
  public final padY:Float;
  public function new(sequence:Int64, dropped:Int64, completedTimestampNs:Int64, output:Bytes,
      sourceWidth:Int, sourceHeight:Int, imageScale:Float, padX:Float, padY:Float) {
    this.sequence = sequence; this.dropped = dropped;
    this.completedTimestampNs = completedTimestampNs; this.output = output;
    this.sourceWidth = sourceWidth; this.sourceHeight = sourceHeight;
    this.imageScale = imageScale; this.padX = padX; this.padY = padY;
  }
}

/** Caller-owned model session; its worker performs image conversion and inference. */
class InferenceSession {
  final owner:Ownedrk_inference_session;
  public final inputBytes:Int;
  public final outputBytes:Int;
  public final inputs:Array<InferenceTensor> = [];
  public final outputs:Array<InferenceTensor> = [];
  var closed = false;

  public function new(path:String, ?threads:Int = 1, ?dynamicWidth:Int = 0, ?dynamicHeight:Int = 0) {
    var options = new rk_inference_options();
    options.set_struct_size(rk_inference_options.size());
    options.set_intra_op_threads(threads);
    options.set_dynamic_width(dynamicWidth);
    options.set_dynamic_height(dynamicHeight);
    var opened = RobotKitInference.rk_inference_create(path, options);
    check(opened.status, 'inference.open($path)');
    owner = opened.out_session;
    var info = new rk_inference_info(); info.set_struct_size(rk_inference_info.size());
    check(RobotKitInference.rk_inference_get_info(owner.borrow(), info).status, "inference.info");
    inputBytes = Int64.toInt(info.get_input_bytes());
    outputBytes = Int64.toInt(info.get_output_bytes());
    for (index in 0...info.get_input_count()) inputs.push(tensor(RobotKitInferenceConstants.RK_INFERENCE_INPUT, index));
    for (index in 0...info.get_output_count()) outputs.push(tensor(RobotKitInferenceConstants.RK_INFERENCE_OUTPUT, index));
  }

  function tensor(direction:Int, index:Int):InferenceTensor {
    var value = new rk_inference_tensor(); value.set_struct_size(rk_inference_tensor.size());
    check(RobotKitInference.rk_inference_get_tensor(owner.borrow(), direction, index, value).status, "inference.tensor");
    var name = new StringBuf();
    for (i in 0...128) { var code = value.get_name(i); if (code == 0) break; name.addChar(code); }
    return {name:name.toString(), elementType:value.get_element_type(),
      shape:[for (i in 0...value.get_rank()) value.get_shape(i)],
      byteOffset:value.get_byte_offset(), byteCount:value.get_byte_count()};
  }

  public static function modelDigest(path:String):String {
    var result = RobotKitInference.rk_inference_model_sha256(path);
    check(result.status, 'inference.digest($path)');
    return result.digest.getString(0, 64);
  }

  public function run(input:Bytes):Bytes {
    ensureOpen();
    var result = RobotKitInference.rk_inference_run(owner.borrow(), input);
    check(result.status, "inference.run");
    return result.output;
  }

  public function submit(sequence:Int64, input:Bytes):Void {
    ensureOpen();
    check(RobotKitInference.rk_inference_submit(owner.borrow(), sequence, input), "inference.submit");
  }

  public function submitRgb8(sequence:Int64, width:Int, height:Int, pixels:Bytes,
      targetWidth:Int, targetHeight:Int, ?layout:Int = 1, ?padding:Float = 0.0,
      ?scale:Float = 1.0 / 255.0, ?mean:Array<Float>, ?std:Array<Float>):Void {
    ensureOpen();
    var options = new rk_inference_image_options();
    options.set_struct_size(rk_inference_image_options.size());
    options.set_target_width(targetWidth); options.set_target_height(targetHeight);
    options.set_layout(layout); options.set_padding_value(padding); options.set_scale(scale);
    var m = mean == null ? [0.0, 0.0, 0.0] : mean;
    var s = std == null ? [1.0, 1.0, 1.0] : std;
    if (m.length != 3 || s.length != 3) throw "Inference mean/std need three channels";
    for (i in 0...3) { options.set_mean(i, m[i]); options.set_std(i, s[i]); }
    check(RobotKitInference.rk_inference_submit_rgb8(owner.borrow(), sequence,
      width, height, width * 3, pixels, options), "inference.submitRgb8");
  }

  public function poll():Null<InferenceResult> {
    ensureOpen();
    var meta = new rk_inference_result(); meta.set_struct_size(rk_inference_result.size());
    var found = RobotKitInference.rk_inference_poll_result(owner.borrow(), meta);
    if (found.status == RobotKitRuntimeConstants.RK_ERROR_STALE_STATE) return null;
    check(found.status, "inference.poll");
    check(meta.get_status(), "inference.worker");
    return new InferenceResult(meta.get_sequence(), meta.get_dropped(),
      meta.get_completed_timestamp_ns(), found.output, meta.get_source_width(),
      meta.get_source_height(), meta.get_image_scale(), meta.get_pad_x(), meta.get_pad_y());
  }

  public function dispose():Void { if (!closed) { closed = true; owner.close(); } }
  function ensureOpen():Void if (closed) throw "Inference session is closed";
  static function check(status:Int, operation:String):Void
    if (status != RobotKitRuntimeConstants.RK_OK) throw '$operation failed with RobotKit status $status';
}
