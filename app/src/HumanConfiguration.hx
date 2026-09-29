package app;

import nativekit.sim.SimPose;

/** Runtime worker entry in a sensor and simulation configuration. */
class HumanConfiguration {
  public final id:String;
  public final assetPath:String;
  public final position:Array<Float>;
  public final rotation:Array<Float>;
  public final jobName:Null<String>;

  public function new(id:String, assetPath:String, position:Array<Float>, rotation:Array<Float>,
      ?jobName:String) {
    if (id == null || id.length == 0 || assetPath == null || assetPath.length == 0 ||
        position == null || position.length != 3 || rotation == null || rotation.length != 4)
      throw "Human configuration needs an ID, asset, and pose";
    for (value in position.concat(rotation)) if (!Math.isFinite(value))
      throw "Human start pose must be finite";
    this.id = id;
    this.assetPath = assetPath;
    this.position = position.copy();
    this.rotation = rotation.copy();
    this.jobName = jobName;
  }

  public function startPose():SimPose
    return new SimPose(position[0], position[1], position[2],
      rotation[0], rotation[1], rotation[2], rotation[3]);

  public function record():Dynamic return {
    id:id, assetPath:assetPath, position:position.copy(), rotation:rotation.copy(), jobName:jobName
  };
}
