package robotkit.perception;

import robotkit.deployment.PerceptionPipelineConfig;

/** Deployment metadata is independent of optional pipeline implementations. */
class PerceptionPipelineRegistry {
  static final factories:Map<String, PerceptionPipelineConfig->String->PerceptionPipeline> = [];
  public static function supports(name:String):Bool
    return name == "object_detector" || factories.exists(name);
  public static function register(name:String, factory:PerceptionPipelineConfig->String->PerceptionPipeline):Void {
    if (name == null || name.length == 0 || factory == null) throw "Pipeline registration requires a name and factory";
    factories.set(name, factory);
  }
  public static function create(config:PerceptionPipelineConfig, producerId:String):PerceptionPipeline {
    var factory = factories.get(config.pipeline);
    if (factory == null) throw 'Perception pipeline ${config.pipeline} requires its optional implementation package';
    return factory(config, producerId);
  }
}
