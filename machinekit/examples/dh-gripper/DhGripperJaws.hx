import cadkit.Shape;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.AssemblyState;
import cadkit.modeling.Vector;
import materia.assembly.AssemblyFrames;

/** Kinematic reference inferred from the pinned supplier pads and 50 mm total stroke. */
class DhGripperJaws {
  public static final TOTAL_STROKE_MM=50.0;
  public static final MASTER="jaw-opening";
  public static function configure(model:AssemblyModel):Void {
    for(id in ["jaw-left","jaw-right"]) {
      model.add(id);
      model.connector("pgc-140-50",id,AssemblyFrames.identity());
      model.connector(id,"slide",AssemblyFrames.identity());
      model.mateOnAxis(id=="jaw-left" ? MASTER : "jaw-opening-follower","prismatic",
        "pgc-140-50",id,id,"slide",{x:id=="jaw-left" ? -1.0 : 1.0,y:0,z:0},
        TOTAL_STROKE_MM/2,{lower:0,upper:TOTAL_STROKE_MM/2,velocity:null,effort:null});
    }
    model.couple("jaw-synchronization",MASTER,"jaw-opening-follower",1);
    model.ground("pgc-140-50");
  }
  static function near(a:Float,b:Float,label:String):Void {
    if(!Math.isFinite(a) || Math.abs(a-b)>0.001) throw '$label: $a != $b';
  }
  /** Analytic pad separation plus exact CAD interference and independent kinematic checks. */
  public static function check(model:AssemblyModel,left:Shape,right:Shape,padLeft:Shape,padRight:Shape,gapOffset:Float=0):Void {
    var state=model.initialState("dh-pgc-articulated-reference");
    for(sample in 0...11) {
      var opening=sample*5.0+gapOffset, travel=(opening-gapOffset)/2;
      state.setJoint(MASTER,travel);
      var a=state.worldPose("jaw-left"), b=state.worldPose("jaw-right");
      near(a.x,-travel,"Left jaw displacement"); near(b.x,travel,"Right jaw displacement");
      if(gapOffset>0) {
        for(id in ["finger-left","finger-right"]) {
          var pose=state.worldPose(id);
          near(pose.x,id=="finger-left" ? a.x : b.x,"Finger/jaw attachment");
          near(pose.y,0,"Finger Y drift");near(pose.z,0,"Finger Z drift");
        }
      }
      for(pose in [a,b]) {near(pose.y,0,"Jaw Y drift"); near(pose.z,0,"Jaw Z drift");near(pose.qw,1,"Jaw rotation");}
      var l=left.translate(new Vector(-travel,0,0).native());
      var r=right.translate(new Vector(travel,0,0).native());
      var pl=padLeft.translate(new Vector(-travel,0,0).native());
      var pr=padRight.translate(new Vector(travel,0,0).native());
      try {
        near(pr.bounds().get_min().get_x()-pl.bounds().get_max().get_x(),opening,"Pad gap");
        near(pl.distance(pr),opening,"Exact pad distance");
        var overlap=l.common(r);
        var volume=overlap.subshapeCount(CadKit.ShapeKind.Solid)==0 ? 0 : overlap.volume();
        overlap.close(); if(volume>0.001) throw 'Jaw interference at $opening mm: $volume mm3';
      } catch(error:Dynamic) {l.close();r.close();pl.close();pr.close();throw error;}
      l.close();r.close();pl.close();pr.close();
      var fixed=state.worldPose("pgc-140-50"); near(fixed.x,0,"Housing movement");
      var cable=state.worldPose("electrical-cable");near(cable.x,0,"Cable movement");
    }
    for(invalid in [-0.01,25.01]) {
      var rejected=false;try {state.setJoint(MASTER,invalid);} catch(_:Dynamic) {rejected=true;}
      if(!rejected) throw "Out-of-range jaw position accepted";
    }
    var rejected=false;try {state.setJoint("jaw-opening-follower",1);} catch(_:Dynamic) {rejected=true;}
    if(!rejected) throw "Coupled jaw accepted independent motion";
    Sys.println('Jaw checks passed: 11 openings from $gapOffset to ${gapOffset+50} mm; CAD pad gap, interference, synchronization and limits');
  }
}
