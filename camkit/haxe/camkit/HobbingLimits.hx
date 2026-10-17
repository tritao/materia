package camkit;

import toolpathkit.path.ToolpathOp;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;

/** Installed logical-axis travel in mm/radians plus speed limits. Supplied by the machine adapter. */
class HobbingLimits {
  public final lower:Point3;
  public final upper:Point3;
  public final minTilt:Float;
  public final maxTilt:Float;
  public final maxHobRpm:Float;
  public final maxTableRpm:Float;
  public function new(lower:Point3,upper:Point3,minTilt:Float,maxTilt:Float,maxHobRpm:Float,maxTableRpm:Float) {
    for(v in [lower.x,lower.y,lower.z,upper.x,upper.y,upper.z,minTilt,maxTilt,maxHobRpm,maxTableRpm])
      if(!Math.isFinite(v))throw "Hobber limits must be finite";
    if(lower.x>upper.x||lower.y>upper.y||lower.z>upper.z||minTilt>maxTilt||maxHobRpm<=0||maxTableRpm<=0)
      throw "Invalid hobber travel or spindle limits";
    this.lower=lower;this.upper=upper;this.minTilt=minTilt;this.maxTilt=maxTilt;
    this.maxHobRpm=maxHobRpm;this.maxTableRpm=maxTableRpm;
  }
  public function check(job:HobbingJob):Void {
    if(job.rpm>maxHobRpm)throw 'Hob spindle overspeed: ${job.rpm} rpm exceeds $maxHobRpm';
    if(job.tableRpm>maxTableRpm)throw 'Table overspeed: ${job.tableRpm} rpm exceeds $maxTableRpm';
    if(-job.hob.leadAngle<minTilt||-job.hob.leadAngle>maxTilt)throw "Hob tilt exceeds swivel travel";
    for(op in job.program.ops)switch op {
      case Move(_,Line(a,b),_,_,_): point(a);point(b);
      case _: // Auxiliary named positions checked by their derived pass envelope below.
    }
    for(pass in job.passes){point(new Point3(job.safeX/1000,pass.shift/1000,pass.zStart/1000));point(new Point3(job.safeX/1000,pass.shift/1000,pass.zEnd/1000));}
  }
  function point(p:Point3):Void {
    var value=[p.x*1000,p.y*1000,p.z*1000],lo=[lower.x,lower.y,lower.z],hi=[upper.x,upper.y,upper.z],axis=["X","Y","Z"];
    for(i in 0...3)if(value[i]<lo[i]-1e-9||value[i]>hi[i]+1e-9)throw 'Hobber ${axis[i]} position ${value[i]} mm exceeds travel [${lo[i]},${hi[i]}]';
  }
}
