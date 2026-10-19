package app;

/** Deterministic, presentation-only handling timeline on the simulation clock.
    Not a contact/gripper solver or a hydraulic deformation calculation. */
class HobbingHandling {
  public static inline final LIFT_MM=120.0;
  public static inline final LOAD_SECONDS=4.0;
  public static inline final UNLOAD_SECONDS=4.0;
  public var phase(default,null)="Blank";
  public var liftMm(default,null):Float=LIFT_MM;
  public var clamp(default,null)=0.0;
  public var inspection(default,null)="sequence";
  public function new() {}
  static function smooth(value:Float):Float {var x=Math.max(0,Math.min(1,value));return x*x*(3-2*x);}
  public function load(seconds:Float):Bool {
    if(seconds<1){phase="Blank above mandrel";liftMm=LIFT_MM;clamp=0;}
    else if(seconds<3){phase="Lowering cup";liftMm=LIFT_MM*(1-smooth((seconds-1)/2));clamp=0;}
    else {phase="Expanding sleeve (illustrative)";liftMm=0;clamp=smooth(seconds-3);}
    return seconds>=LOAD_SECONDS;
  }
  public function cutting():Void {phase="Clamped / cutting";liftMm=0;clamp=1;}
  public function retracting():Void {phase="Retracting head before release";liftMm=0;clamp=1;}
  public function unload(seconds:Float):Bool {
    if(seconds<1){phase="Releasing sleeve (illustrative)";liftMm=0;clamp=1-smooth(seconds);}
    else {phase=seconds<UNLOAD_SECONDS?"Lifting finished cup":"Finished cup";liftMm=LIFT_MM*smooth((seconds-1)/3);clamp=0;}
    return seconds>=UNLOAD_SECONDS;
  }
  public function inspect(mode:String):Void {
    if(["sequence","blank","clamped","finished"].indexOf(mode)<0)throw "Unknown cup inspection view";
    inspection=mode;
  }
  public function displayedLift():Float return inspection=="blank"||inspection=="finished"?LIFT_MM:inspection=="clamped"?0:liftMm;
  public function displayedClamp():Float return inspection=="clamped"?1:inspection=="blank"||inspection=="finished"?0:clamp;
}
