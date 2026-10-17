package stockkit.hobbing;
import gearkit.GearSpec;
import gearkit.HobDesign;
import stockkit.RingGrid;

/** External spur-gear driver, no machine/runtime coupling or error sources.
    Reference rack module/pressure angle must match; positive feed advances +Z. */
class IdealHobbingJob {
  public final spec:GearSpec;public final hob:HobDesign;
  public final rpm:Float;public final duration:Float;public final workTurns:Float;
  public final feed:Float;public final zStart:Float;public final shift:Float;
  public final centerDistance:Float;public final tablePhase:Float;
  public function new(spec:GearSpec,hob:HobDesign,rpm:Float=3000,workTurns:Float=2,
      feed:Float=0,zStart:Float=0,shift:Float=0) {
    if(spec.internal)throw "ideal hobbing driver supports external spur gears";
    if(Math.abs(spec.module-hob.rack.module)>1e-12||Math.abs(spec.pressureAngle-hob.rack.pressureAngle)>1e-12)
      throw "hob and gear rack must match";
    for(v in [rpm,workTurns,feed,zStart,shift])if(!Math.isFinite(v))throw "job values must be finite";
    if(rpm<=0||workTurns<=0)throw "job speed and turns must be positive";
    this.spec=spec;this.hob=hob;this.rpm=rpm;this.workTurns=workTurns;
    this.feed=feed;this.zStart=zStart;this.shift=shift;
    duration=workTurns*spec.teeth/hob.starts*60/rpm;
    if(!Math.isFinite(duration)||duration<=0)throw "job duration must be finite and positive";
    centerDistance=spec.pitchRadius()+hob.pitchDiameter/2+spec.profileShift*spec.module;
    var pitch=2*Math.PI/spec.teeth;
    var phase=(Math.PI/spec.teeth-Math.PI/2)%pitch;if(phase<0)phase+=pitch;
    tablePhase=phase-shift*Math.cos(hob.leadAngle)/spec.pitchRadius();
  }
  public function axesAt(time:Float):HobbingAxes {
    var b=rpm*2*Math.PI/60*time,c=b*hob.starts/spec.teeth;
    return new HobbingAxes(centerDistance,shift,zStart+feed*c/(2*Math.PI),-hob.leadAngle,b,c+tablePhase);
  }
  public function schedule(grid:RingGrid):Array<GashPass> {
    var limit=(centerDistance-grid.radiusMax)/(hob.outsideDiameter/2);
    if(limit>=1)throw "hob does not reach stock";
    var half=Math.acos(Math.max(-1,limit))+Math.abs(hob.rake)+0.01;
    return GashPass.constantSpeed(hob.gashes,rpm,duration,Math.min(Math.PI/2,half));
  }
}
