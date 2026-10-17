package stockkit.hobbing;

/** An exact master-phase crossing and its engagement window (seconds).
    A window may straddle many controller ticks. center may lie outside the
    requested run when an already engaged edge crosses a run boundary. */
class GashPass {
  public final gash:Int;public final pass:Int;
  public final start:Float;public final center:Float;public final end:Float;
  public function new(gash:Int,pass:Int,start:Float,center:Float,end:Float) {
    if(gash<0||pass<0||!Math.isFinite(start)||!Math.isFinite(center)||!Math.isFinite(end)||start>=end)
      throw "invalid gash pass";
    this.gash=gash;this.pass=pass;this.start=start;this.center=center;this.end=end;
  }
  /** Constant-speed events come from integer phase indices, never tick accumulation. */
  public static function constantSpeed(gashes:Int,rpm:Float,duration:Float,halfAngle:Float):Array<GashPass> {
    if(gashes<2||!Math.isFinite(rpm)||rpm<=0||!Math.isFinite(duration)||duration<=0
      ||!Math.isFinite(halfAngle)||halfAngle<=0||halfAngle>=Math.PI)
      throw "invalid constant-speed gash schedule";
    var omega=rpm*2*Math.PI/60,step=2*Math.PI/gashes;
    if(!Math.isFinite(omega)||!Math.isFinite(duration*omega)||duration*omega/step>1999000)
      throw "gash schedule exceeds finite phase limit";
    var first=Math.floor(-halfAngle/step),last=Math.ceil((duration*omega+halfAngle)/step);
    if(last-first>2000000)throw "gash schedule exceeds two million passes";
    var result:Array<GashPass> =[];
    for(k in first...last+1) {
      var center=k*step/omega,start=Math.max(0,(k*step-halfAngle)/omega),end=Math.min(duration,(k*step+halfAngle)/omega);
      if(start>=end)continue;
      var gash=(-k)%gashes;if(gash<0)gash+=gashes;
      result.push(new GashPass(gash,k-first,start,center,end));
    }
    return result;
  }
  /** Exact phase crossings under piecewise-linear master interpolation. */
  public static function fromMotion(motion:HobbingMotion,gashes:Int,halfAngle:Float):Array<GashPass> {
    if(gashes<2||!Math.isFinite(halfAngle)||halfAngle<=0||halfAngle>=Math.PI)throw "invalid gash schedule";
    var frames=motion.samples(),start=frames[0],end=frames[frames.length-1],step=2*Math.PI/gashes;
    if(Math.abs(start.axes.b/step)>1000000000||Math.abs(end.axes.b/step)>1000000000
      ||(end.axes.b-start.axes.b)/step>1999000)throw "master phase outside scheduling limits";
    motion.timeAtMaster(start.axes.b);
    var first=Math.floor((start.axes.b-halfAngle)/step),last=Math.ceil((end.axes.b+halfAngle)/step),result:Array<GashPass> = [];
    for(k in first...last+1) {
      var t0=Math.max(start.time,motion.timeAtMaster(k*step-halfAngle));
      var t1=Math.min(end.time,motion.timeAtMaster(k*step+halfAngle));
      if(t0>=t1)continue;
      var gash=(-k)%gashes;if(gash<0)gash+=gashes;
      result.push(new GashPass(gash,k-first,t0,motion.timeAtMaster(k*step),t1));
    }
    return result;
  }
}
