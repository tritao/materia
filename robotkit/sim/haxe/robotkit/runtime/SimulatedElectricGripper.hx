package robotkit.runtime;

/** Position/effort controlled parallel jaws. Grasp feedback requires contact on both jaws;
 * a close command or nominal width alone never confirms possession. No kinematic attachment. */
class SimulatedElectricGripper implements SimulationStepObserver {
  final simulation:Simulation;
  final robot:Int;
  final joint:Int;
  final jaws:Array<Int>;
  final openPosition:Float;
  final graspPosition:Float;
  final effort:Float;
  final maxSpeed:Float;
  var commanded:Float;
  var desired:Float;
  var lastTime:Null<haxe.Int64> = null;
  final observer:Int;
  public var closed(default,null)=false;
  public var stopped(default,null)=false;
  public function new(simulation:Simulation,robot:Int,joint:Int,jaws:Array<Int>,openPosition:Float,graspPosition:Float,effort:Float,maxSpeed:Float=0.005,?initialPosition:Float) {
    if(simulation==null || joint<0 || jaws.length!=2 || jaws[0]==jaws[1] ||
      !Math.isFinite(openPosition) || !Math.isFinite(graspPosition) || openPosition==graspPosition || !Math.isFinite(effort) || effort<=0 || !Math.isFinite(maxSpeed) || maxSpeed<=0)
      throw "Electric gripper needs two jaws, measured positions and an effort cap";
    this.simulation=simulation;this.robot=robot;this.joint=joint;this.jaws=jaws.copy();
    this.openPosition=openPosition;this.graspPosition=graspPosition;this.effort=effort;this.maxSpeed=maxSpeed;
    commanded=initialPosition==null ? openPosition : initialPosition;desired=commanded;observer=simulation.addStepObserver(this);
  }
  public function command(close:Bool):Void {
    if(stopped)throw "Electric gripper is stopped; automatic retry is inhibited";
    closed=close;desired=close ? graspPosition : openPosition;
    simulation.driveProcessJoint(robot,joint,commanded,0,effort);
  }
  /** Position an empty gripper without claiming a grasp. */
  public function position(target:Float):Void {
    if(stopped)throw "Electric gripper is stopped";
    if(!Math.isFinite(target) || target<Math.min(openPosition,graspPosition) || target>Math.max(openPosition,graspPosition))throw "Invalid empty-gripper position";
    closed=false;desired=target;simulation.driveProcessJoint(robot,joint,commanded,0,effort);
  }
  /** Leaves the last jaw command energized; a stop must never issue an opening command. */
  public function stopAndHold():Void {stopped=true;desired=commanded;}
  public function afterSimulationStep(sourceTimestampNs:haxe.Int64):Void {
    var dt=lastTime==null ? simulation.fixedTimestepSeconds : haxe.Int64.toFloat(haxe.Int64.sub(sourceTimestampNs,cast lastTime))*1e-9;
    lastTime=sourceTimestampNs;
    if(stopped || !(dt>0) || desired==commanded)return;
    var delta=desired-commanded,step=Math.min(Math.abs(delta),maxSpeed*dt);
    commanded+=delta<0 ? -step : step;
    simulation.driveProcessJoint(robot,joint,commanded,step==Math.abs(delta) ? 0 : delta<0 ? -maxSpeed : maxSpeed,effort);
  }
  /** Command-ramp completion is an admission condition, never evidence of possession. */
  public function commandComplete():Bool return commanded==desired;
  public function dispose():Void simulation.removeStepObserver(observer);
  public function grippedObjects(contacts:Array<RobotContact>):Array<Int> {
    if(!closed)return [];
    var left:Array<Int> = [],right:Array<Int> = [];
    for(contact in contacts)if(contact.active && contact.wrenchValid && contact.distance>=-0.0001 && contact.force.norm()>0.1 && contact.otherKind==RobotContactOtherKind.Object) {
      var pad:Null<Array<Int>> = contact.linkIndex==jaws[0] ? left : contact.linkIndex==jaws[1] ? right : null;
      if(pad!=null && pad.indexOf(contact.otherObject)<0)pad.push(contact.otherObject);
    }
    return [for(id in left)if(right.indexOf(id)>=0)id];
  }
  public function jawContact(link:Int):Bool return jaws.indexOf(link)>=0;
  public function opening():Float return simulation.jointSample(robot,joint).get_position();
  public function released(contacts:Array<RobotContact>,object:Int,tolerance:Float=0.00001):Bool {
    if(closed || Math.abs(opening()-openPosition)>tolerance)return false;
    for(contact in contacts)if(contact.active && contact.otherObject==object && jaws.indexOf(contact.linkIndex)>=0)return false;
    return true;
  }
}
