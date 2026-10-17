package machinekit.motion;

import machinekit.motion.PrecisionPart;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Frameless direct-drive torque motor. Reference row is Assumed; dimensions are simplified envelopes. */
class TorqueMotor extends PrecisionPart implements machinekit.motion.MotorDrive {
  public static function parameters():Array<ComponentParameter>return PrecisionPart.common().concat([
    PrecisionPart.quantity("outside",150,"mm"),
    PrecisionPart.quantity("bore",80,"mm"),
    PrecisionPart.quantity("height",45,"mm"),
    PrecisionPart.quantity("continuousTorque",10,"N m"),
    PrecisionPart.quantity("peakTorque",30,"N m"),
    PrecisionPart.quantity("ratedSpeed",60*2*Math.PI/60,"rad/s"),
    PrecisionPart.quantity("maxSpeed",120*2*Math.PI/60,"rad/s"),
    PrecisionPart.quantity("rotorInertia",0.003,"kg m2"),
    PrecisionPart.quantity("coggingTorque",0.03,"N m"),
    P.count("poleCount",24),
    P.count("coggingOrder",48),
    PrecisionPart.quantity("torqueConstant",1.2,"N m/A rms"),
    PrecisionPart.quantity("backEmfConstant",0.6928203230275509,"V/(rad/s)"),
    PrecisionPart.quantity("phaseResistance",0.3,"ohm")]);
  static var recipe:Null<ComponentType>;
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.motion.torque-motor",parameters(),v -> new TorqueMotor(v));
    return recipe;
  }
  public static function catalog():machinekit.catalog.Catalog<ComponentValues>
    return PrecisionPart.assumedCatalog("Frameless direct-drive torque motor","TORQUE-MOTOR-10NM",parameters());
  public function new(?overrides:ComponentValues) {
    var v=PrecisionPart.resolve(parameters(),overrides);
    super("TORQUE-MOTOR-10NM","Frameless direct-drive torque motor (reference envelope)","aluminium 6061",v,true,[{name:"coggingTorque",unit:"N m"}]);
    if(n("continuousTorque")<=0||n("peakTorque")<n("continuousTorque")||n("ratedSpeed")<=0||n("maxSpeed")<=n("ratedSpeed")
      ||count("poleCount")<2||count("poleCount")%2!=0||count("coggingOrder")<1)throw "invalid torque motor ratings";
    for(k in ["rotorInertia","torqueConstant","backEmfConstant","phaseResistance"])if(n(k)<=0)throw "motor rating must be positive";
  }
  override public function componentType():Null<ComponentType>return recipeType();
  public var continuousTorque(get,never):Float;
  function get_continuousTorque():Float return n("continuousTorque");
  public var peakTorque(get,never):Float;
  function get_peakTorque():Float return n("peakTorque");
  public var ratedSpeed(get,never):Float;
  function get_ratedSpeed():Float return n("ratedSpeed");
  public var maxSpeed(get,never):Float;
  function get_maxSpeed():Float return n("maxSpeed");
  public var rotorInertia(get,never):Float;
  function get_rotorInertia():Float return n("rotorInertia");
  public var coggingTorque(get,never):Float;
  function get_coggingTorque():Float return n("coggingTorque");
  override public function motorDrive():Null<machinekit.motion.MotorDrive>return this;
  public function electricalRating():Null<machinekit.motion.ServoElectricalRating>
    return {torqueConstant:n("torqueConstant"),backEmfConstant:n("backEmfConstant"),phaseResistance:n("phaseResistance"),mechanicalEfficiency:0.95};
  public function jointTorque(motorTorque:Float):Float {
    if(!Math.isFinite(motorTorque))throw "invalid torque";return motorTorque;
  }
  public function actuator(id:String,joint:String,volts:Float,margin:Float,?current:Float):materia.assembly.AssemblyDefinition.AssemblyActuator {
    return {id:id,joint:joint,drive:"servo",maxEffort:peakTorque,maxRate:maxSpeed,rotorInertia:rotorInertia,
      ratedTorque:continuousTorque,peakTorque:peakTorque,ratedSpeed:ratedSpeed,maxSpeed:maxSpeed,
      torqueSpeed:[0,peakTorque,ratedSpeed,peakTorque,maxSpeed,continuousTorque],
      assumed:ratingsBasis==Assumed?["torque motor ratings"]:null};
  }
}
