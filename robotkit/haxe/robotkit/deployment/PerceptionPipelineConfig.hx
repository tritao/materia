package robotkit.deployment;

/** Validated deployment data. Pipeline implementation is selected in code. */
class PerceptionPipelineConfig {
  public final id:String;
  public final input:String;
  public final pipeline:String;
  public final modelPath:String;
  public final modelSha256:String;
  public final host:String;
  public final consumers:Array<String>;
  public final scoreThreshold:Float;
  public final maxRateHz:Float;
  public final iouThreshold:Float;

  public function new(id:String, input:String, pipeline:String, modelPath:String,
      modelSha256:String, host:String, consumers:Array<String>, scoreThreshold:Float,
      maxRateHz:Float, iouThreshold:Float) {
    this.id = id; this.input = input; this.pipeline = pipeline;
    this.modelPath = modelPath; this.modelSha256 = modelSha256;
    this.host = host; this.consumers = consumers.copy();
    this.scoreThreshold = scoreThreshold; this.maxRateHz = maxRateHz;
    this.iouThreshold = iouThreshold;
  }
}
