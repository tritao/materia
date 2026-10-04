package robotkit.perception;



import robotkit.deployment.PerceptionPipelineConfig;

/** The only place deployment names are bound to pipeline code. */
class PerceptionPipelineRegistry {
  public static function supports(name:String):Bool return name == "object_detector";

  public static function create(config:PerceptionPipelineConfig, producerId:String):PerceptionPipeline {
    return switch config.pipeline {
      case "object_detector": new ObjectDetectorPipeline(producerId, config.id, config.input,
        config.id, config.modelPath, config.modelSha256, config.scoreThreshold,
        config.iouThreshold, config.maxRateHz, config.threads,
        config.dynamicWidth, config.dynamicHeight);
      case _: throw 'Unknown perception pipeline ${config.pipeline}';
    };
  }
}
