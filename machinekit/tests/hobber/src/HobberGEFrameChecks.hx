import stockkit.*;
import stockkit.hobbing.*;
import materia.assembly.AssemblyFrames as F;

private class InstalledHobAxes extends HobbingAxes {
  final values:Array<Float>;
  public function new(axes:HobbingAxes,frame:materia.assembly.AssemblyRecord.AssemblyFrame){
    super(axes.x,axes.y,axes.z,axes.a,axes.b,axes.c);values=[for(i in 0...12)0.0];
    for(i in 0...3){var v=F.transformVector(frame,i==0?1:0,i==1?1:0,i==2?1:0);values[i]=v.x;values[3+i]=v.y;values[6+i]=v.z;}
    values[9]=frame.x;values[10]=frame.y;values[11]=frame.z;
  }
  override public function poseValues():Array<Float> return values.copy();
}

/** Short physical HB3/HB10 regression, using the actual assembly frame at every knot. */
class HobberGEFrameChecks {
  public static function run():Void {
    var machine=new FlexsplineHobber(),state=FlexsplineHobberPreview.inspectionState(machine);
    var x=state.joint("X"),c=state.joint(FlexsplineHobber.C),a=state.joint(FlexsplineHobber.A),omega=3000*2*Math.PI/60;
    function ideal(t:Float):HobbingAxes return new HobbingAxes(x,0,4,a,omega*t,c+omega*t/200);
    function installed(t:Float):HobbingAxes {var axes=ideal(t);
      state.setJoint(FlexsplineHobber.B,axes.b);state.setJoint(FlexsplineHobber.C,axes.c);state.forwardKinematics();
      return new InstalledHobAxes(axes,machine.hobToWork(state));
    }
    var grid=new RingGrid(machine.spec.rootRadius()+0.01,machine.spec.tipRadius()-0.01,3.9,4.1,0.08,0.1);
    var before=RingStock.annulus(grid),after=RingStock.annulus(grid),sweep=new HobbingSweep(machine.hob.design,-2,2,0.00002);
    before.setThreads(1);after.setThreads(1);
    var fa=new HobbingForces(before,30,machine.spec.pressureAngle,1),fb=new HobbingForces(after,30,machine.spec.pressureAngle,1);
    var pass=new GashPass(0,0,0,0.00005,0.0001),maxPose=0.0,maxArc=0.0;
    try {
      for(t in [0.0,0.00005,0.0001]){var p=ideal(t).poseValues(),q=installed(t).poseValues();for(i in 0...12)maxPose=Math.max(maxPose,Math.abs(p[i]-q[i]));}
      if(maxPose>1e-8)throw "GE machine/work frame regression";
      sweep.cutPass(before,ideal,pass,0.000005);sweep.cutPass(after,installed,pass,0.000005);
      fa.afterPass(pass,ideal(pass.start),ideal(pass.end),16);fb.afterPass(pass,installed(pass.start),installed(pass.end),16);
      var removed=0;
      for(r in 0...grid.radialCount)for(z in 0...grid.axialCount){var u=before.circle(r,z),v=after.circle(r,z);
        if(u.length!=v.length)throw "GE frame changes cut topology";
        for(i in 0...u.length){maxArc=Math.max(maxArc,grid.radius(r)*Math.max(Math.abs(u[i].lo-v[i].lo),Math.abs(u[i].hi-v[i].hi)));}
        if(u.length!=1||u[0].lo!=0||u[0].hi<2*Math.PI-1e-12)removed++;
      }
      if(removed==0||maxArc>1e-8)throw "GE reference chip fixture empty or changed";
      var forceDelta=Math.max(Math.abs(fa.maxTangential-fb.maxTangential),Math.abs(fa.maxRadial-fb.maxRadial));
      if(forceDelta>1e-6)throw "GE frame changes HB10 loads";
      Sys.println("GE_FRAME_PASS "+haxe.Json.stringify({threads:1,sweepToleranceMm:0.00002,cutPassToleranceMm:0.000005,poseToleranceMmRad:1e-8,maxPoseDifference:maxPose,arcToleranceMm:1e-8,maxArcDifferenceMm:maxArc,forceToleranceN:1e-6,maxForceDifferenceN:forceDelta,cutCircles:removed,scope:"short reference-geometry cut using measured assembly transforms; no full-reference grade claim"}));
    }catch(error:Dynamic){fa.dispose();fb.dispose();sweep.dispose();before.dispose();after.dispose();throw error;}
    fa.dispose();fb.dispose();sweep.dispose();before.dispose();after.dispose();
  }
}
