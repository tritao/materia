package camkit;

/** Hobbing inputs use GearKit's mm, radians, rpm and m/min; ToolpathKit output uses SI. */
class HobbingSettings {
  public var cuttingSpeed:Float = 300; // m/min, assumed prototype value, not a material recommendation
  public var feedPerRevolution:Float = 0.5; // mm/work revolution
  public var climb:Bool = true; // right-hand hob rotating positive B: climb feeds +Z
  /** Incremental radial depths in mm, strictly increasing; empty chooses one full-depth pass. */
  public var depths:Array<Float> = [];
  /** One static hob-axis shift per pass; empty chooses zero. Shifting happens retracted. */
  public var shifts:Array<Float> = [];
  public var blankOuterRadius:Null<Float> = null;
  public var faceBottom:Float = 0;
  public var clearance:Float = 1; // additional radial and axial clearance
  /** Middle-face outward X bump; zero at both face ends, parabolic inside, zero outside. */
  public var crown:Float = 0;
  public var crownChordTolerance:Float = 0.001;
  public var rapidFeed:Float = 20; // mm/s
  public var infeed:Float = 0.5; // mm/s, outside the axial stock envelope
  public var signalTimeout:Float = 30;
  public var xAxis:String = "X";
  public var yAxis:String = "Y";
  public var zAxis:String = "Z";
  public var tiltJoint:String = "swivel/A";
  public var hobJoint:String = "spindle/B";
  public var tableJoint:String = "table/C";
  /** Startup witness: hob encoder datum B=0, released EGB, and machine ready to accept this job. */
  public var readySignal:String = "hob.job_ready";
  public var stoppedSignal:String = "hob.stopped";
  public function new() {}
  public function copy():HobbingSettings {
    var r=new HobbingSettings();
    r.cuttingSpeed=cuttingSpeed;r.feedPerRevolution=feedPerRevolution;r.climb=climb;
    r.depths=depths.copy();r.shifts=shifts.copy();r.blankOuterRadius=blankOuterRadius;
    r.faceBottom=faceBottom;r.clearance=clearance;r.crown=crown;r.crownChordTolerance=crownChordTolerance;
    r.rapidFeed=rapidFeed;r.infeed=infeed;r.signalTimeout=signalTimeout;
    r.xAxis=xAxis;r.yAxis=yAxis;r.zAxis=zAxis;r.tiltJoint=tiltJoint;r.hobJoint=hobJoint;r.tableJoint=tableJoint;
    r.readySignal=readySignal;r.stoppedSignal=stoppedSignal;
    return r;
  }
}
