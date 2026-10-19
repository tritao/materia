import stockkit.hobbing.*;
/** Analytic table axial/tilt signatures; no full reference-budget claim. */
class HobberTableTiltChecks {
  public static function run():Void {
    var count=0;
    function near(x:Float,y:Float,tol:Float):Void {count++;if(Math.abs(x-y)>tol)throw 'table tilt signature $x != $y';}
    var depth=178.0,height=182.0,force=100.0,stiffness=1e6,theta=force*height/(1000*stiffness);
    var source=HobbingErrorSource.tableTilt("table/bearing",stiffness,depth,height,t->force);
    for(c in [0.0,Math.PI/2,Math.PI]){
      var axes=new HobbingAxes(45,0,4,0,0,c),pose=source.apply(0,axes).poseValues();
      // Machine-fixed radial load: inverse positive work lean, rotates with C.
      var radial=45*Math.cos(theta)+(4+depth)*Math.sin(theta);
      near(pose[9],radial*Math.cos(c),1e-10);near(pose[10],-radial*Math.sin(c),1e-10);
      near(pose[11],-45*Math.sin(theta)+(4+depth)*Math.cos(theta)-depth,1e-9);
      for(i in 0...3)for(j in 0...3){var dot=0.0;for(k in 0...3)dot+=pose[3*k+i]*pose[3*k+j];near(dot,i==j?1:0,1e-12);}
      var axial=HobbingErrorSource.tableAxial("table/bearing",0.002).apply(0,axes).poseValues();
      near(axial[11],4-0.001*Math.cos(c),1e-12);
      var zero=HobbingErrorSource.tableTilt("table/bearing",stiffness,depth,height,t->0).apply(0,axes).poseValues(),ideal=axes.poseValues();
      for(i in 0...12)near(zero[i],ideal[i],0);
    }
    var tilt=new HobbingPoseOffset();tilt.workTiltX=0.00002;tilt.workTiltY=-0.00003;tilt.workPivotZ=-depth;
    var start=new HobbingAxes(45,0,4,0,0,0,tilt),finish=new HobbingAxes(45,0,4,0,1,0,tilt.copy());
    var motion=new HobbingMotion([new HobbingKeyframe(0,start),new HobbingKeyframe(1,finish)]);
    var midpoint=motion.at(0.5),direct=new HobbingAxes(45,0,4,0,0.5,0,tilt).poseValues(),sampled=midpoint.poseValues();
    for(i in 0...12)near(sampled[i],direct[i],0);
    Sys.println('TABLE_TILT_PASS $count assertions; load-induced bearing tilt distinct from 2 um axial TIR; assumed stiffness, full source/combined HB11 acceptance open');
  }
}
