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
  public var frontMm(default,null)=0.0;
  public final frontApproachMm:Float;
  public final loadingLiftMm:Float;
  public var inspection(default,null)="sequence";
  public function new(frontApproachMm:Float=0,loadingLiftMm:Float=LIFT_MM) {
    if(!Math.isFinite(frontApproachMm)||frontApproachMm<0||!Math.isFinite(loadingLiftMm)||loadingLiftMm<=0)throw "Invalid loading envelope";
    this.frontApproachMm=frontApproachMm;this.loadingLiftMm=loadingLiftMm;frontMm=frontApproachMm;liftMm=loadingLiftMm;
  }
  static function smooth(value:Float):Float {var x=Math.max(0,Math.min(1,value));return x*x*(3-2*x);}
  public function load(seconds:Float):Bool {
    if(frontApproachMm>0){
      if(seconds<1){phase="Blank in front of machine";frontMm=frontApproachMm;liftMm=loadingLiftMm;clamp=0;}
      else if(seconds<2.5){phase="Transferring cup from front";frontMm=frontApproachMm*(1-smooth((seconds-1)/1.5));liftMm=loadingLiftMm;clamp=0;}
      else if(seconds<3){phase="Seating cup vertically";frontMm=0;liftMm=loadingLiftMm*(1-smooth((seconds-2.5)/0.5));clamp=0;}
      else {phase="Expanding sleeve (illustrative)";frontMm=0;liftMm=0;clamp=smooth(seconds-3);}
      return seconds>=LOAD_SECONDS;
    }
    if(seconds<1){phase="Blank above mandrel";liftMm=loadingLiftMm;clamp=0;}
    else if(seconds<3){phase="Lowering cup";liftMm=loadingLiftMm*(1-smooth((seconds-1)/2));clamp=0;}
    else {phase="Expanding sleeve (illustrative)";liftMm=0;clamp=smooth(seconds-3);}
    return seconds>=LOAD_SECONDS;
  }
  public function cutting():Void {phase="Clamped / cutting";liftMm=0;frontMm=0;clamp=1;}
  public function retracting():Void {phase="Retracting head before release";liftMm=0;frontMm=0;clamp=1;}
  public function unload(seconds:Float):Bool {
    if(frontApproachMm>0){
      if(seconds<1){phase="Releasing sleeve (illustrative)";frontMm=0;liftMm=0;clamp=1-smooth(seconds);}
      else if(seconds<1.5){phase="Clearing mandrel vertically";frontMm=0;liftMm=loadingLiftMm*smooth((seconds-1)/0.5);clamp=0;}
      else {phase=seconds<UNLOAD_SECONDS?"Transferring finished cup to front":"Finished cup";frontMm=frontApproachMm*smooth((seconds-1.5)/2.5);liftMm=loadingLiftMm;clamp=0;}
      return seconds>=UNLOAD_SECONDS;
    }
    if(seconds<1){phase="Releasing sleeve (illustrative)";liftMm=0;clamp=1-smooth(seconds);}
    else {phase=seconds<UNLOAD_SECONDS?"Lifting finished cup":"Finished cup";liftMm=loadingLiftMm*smooth((seconds-1)/3);clamp=0;}
    return seconds>=UNLOAD_SECONDS;
  }
  public function inspect(mode:String):Void {
    if(["sequence","blank","clamped","finished"].indexOf(mode)<0)throw "Unknown cup inspection view";
    inspection=mode;
  }
  public function displayedLift():Float return inspection=="blank"||inspection=="finished"?loadingLiftMm:inspection=="clamped"?0:liftMm;
  public function displayedFront():Float return inspection=="blank"||inspection=="finished"?frontApproachMm:inspection=="clamped"?0:frontMm;
  public function displayedClamp():Float return inspection=="clamped"?1:inspection=="blank"||inspection=="finished"?0:clamp;
}
