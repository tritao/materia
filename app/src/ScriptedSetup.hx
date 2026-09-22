package app;

/** Data-only output populated by a compiled setup script. */
class ScriptedSetup {
  public var reference(default,null):String="";
  public var version(default,null):Int=0;
  public var sensors(default,null):Null<SensorConfiguration> = null;
  public var objects(default,null):Array<SceneObjectData> = [];
  public var backend(default,null):Int=ApplicationSimulation.MUJOCO;
  public var timestep(default,null):Float=0.01;
  public function new() {}
  public function define(reference:String,version:Int,sensors:SensorConfiguration,
      objects:Array<SceneObjectData>,backend:Int,timestep:Float):Void {
    if(this.sensors!=null)throw "Setup script output was defined more than once";
    this.reference=reference;this.version=version;this.sensors=sensors;this.objects=objects;
    this.backend=backend;this.timestep=timestep;
  }
  public function dispose():Void {if(sensors!=null)sensors.dispose();sensors=null;}
}
