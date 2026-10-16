package machinekit.control.erob;

/** Deterministic command-loop stub. Never opens a NIC or enables a physical brake/drive. */
class ERobDriveStub implements ERobDriveInterface {
  var count=0;
  var closed=false;
  var live=false;
  var time = -1.0;
  var q:Array<Float> = [];
  var v:Array<Float> = [];
  var tau:Array<Float> = [];
  public var commands(default,null)=0;
  public function new() {}
  public function configure(joints:Int):Void {
    if(closed || joints!=6) throw "eRob reference stub requires six joints";
    count=joints;live=true;time=-1;commands=0;
    q=[for(_ in 0...count) 0.0];v=q.copy();tau=q.copy();
  }
  public static function requireHardwareReady(profile:Dynamic):Void {
    if(profile.hardwareArmAllowed!=true || profile.limitsConfirmed!=true || profile.driveCurvesConfirmed!=true ||
       profile.vendorPositionLimitsRad==null || profile.commandSigns==null || profile.encoderZeroCounts==null || profile.esiRevision==null)
      throw "Uncommissioned eRob profile: physical operation unavailable";
    // A backend with a real master, ESI/PDO validation and tested safety/brake sequencing is still required.
    throw "Physical EtherCAT backend is not implemented";
  }
  public function command(timeSeconds:Float,position:Array<Float>,velocity:Array<Float>,torqueNm:Array<Float>):Void {
    if(closed || !live || count!=6) throw "eRob stub is not enabled";
    if(!Math.isFinite(timeSeconds) || timeSeconds<=time) throw "eRob command time must increase";
    for(values in [position,velocity,torqueNm]) {
      if(values==null || values.length!=count) throw "eRob command dimension mismatch";
      for(value in values) if(!Math.isFinite(value)) throw "Nonfinite eRob command";
    }
    time=timeSeconds;q=position.copy();v=velocity.copy();tau=torqueNm.copy();commands++;
  }
  public function feedback():{timeSeconds:Float,position:Array<Float>,velocity:Array<Float>,torqueNm:Array<Float>,enabled:Bool}
    return {timeSeconds:time,position:q.copy(),velocity:v.copy(),torqueNm:tau.copy(),enabled:live && !closed};
  public function stop():Void {live=false;v=[for(_ in 0...count) 0.0];tau=v.copy();}
  public function close():Void {stop();closed=true;}
}
