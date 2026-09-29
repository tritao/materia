package robotkit.perception;

import robotkit.world.SensorFrame;

/** Routes camera frames to configured pipelines; submit does not wait for inference. */
class PerceptionHost {
  final pipelines:Array<PerceptionPipeline>;
  public function new(pipelines:Array<PerceptionPipeline>) {
    if (pipelines == null) throw "PerceptionHost needs pipelines";
    this.pipelines = pipelines.copy();
    var ids = new Map<String, Bool>();
    for (pipeline in this.pipelines) {
      if (pipeline == null || pipeline.id() == null || pipeline.id().length == 0 || ids.exists(pipeline.id()))
        throw "PerceptionHost needs distinct pipeline IDs";
      ids.set(pipeline.id(), true);
    }
  }
  public function submit(frame:SensorFrame):Void {
    if (frame == null) throw "PerceptionHost needs a sensor frame";
    for (pipeline in pipelines) if (pipeline.sensorId() == frame.sensorId) pipeline.submit(frame);
  }
  public function poll():Array<ImageDetectionObservation> {
    var output:Array<ImageDetectionObservation> = [];
    for (pipeline in pipelines) for (value in pipeline.poll()) output.push(value);
    return output;
  }
  public function dispose():Void for (pipeline in pipelines) pipeline.dispose();
}
