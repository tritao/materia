/** The open vise constrains the support normal; closed jaws must establish the full orientation. */
enum abstract TendingSeatOrientation(String) to String {
  var SupportNormal="support-normal";
  var FullPose="full-pose";
}
class TendingSeatPolicy {
  public static function parseOrientation(value:String):TendingSeatOrientation return switch(value) {
    case "support-normal":TendingSeatOrientation.SupportNormal;
    case "full-pose":TendingSeatOrientation.FullPose;
    default:throw "Unknown seating orientation constraint: "+value;
  };
  public static function orientationError(q:Array<Float>,reference:Array<Float>,mode:TendingSeatOrientation):Float {
    for(v in [q,reference]) {
      if(v==null || v.length!=4)throw "Seating requires a quaternion";
      var norm=0.0;for(x in v){if(!Math.isFinite(x))throw "Non-finite seating orientation";norm+=x*x;}
      if(Math.abs(norm-1)>0.000001)throw "Seating requires a unit quaternion";
    }
    if(mode==TendingSeatOrientation.FullPose) {
      var dot=0.0;for(i in 0...4)dot+=q[i]*reference[i];
      return 2*Math.acos(Math.min(1,Math.abs(dot)));
    }
    if(mode!=TendingSeatOrientation.SupportNormal)throw "Unknown seating orientation constraint";
    function normal(a:Array<Float>):Array<Float> return [2*(a[0]*a[2]+a[3]*a[1]),2*(a[1]*a[2]-a[3]*a[0]),1-2*(a[0]*a[0]+a[1]*a[1])];
    var a=normal(q),b=normal(reference),dot=0.0;for(i in 0...3)dot+=a[i]*b[i];
    return Math.acos(Math.max(-1,Math.min(1,dot)));
  }
  public static function checks():Int {
    var identity=[0.0,0,0,1],yaw=[0.0,0,Math.sin(0.00125),Math.cos(0.00125)];
    if(orientationError(yaw,identity,TendingSeatOrientation.SupportNormal)>1e-7 ||
      orientationError(yaw,identity,TendingSeatOrientation.FullPose)<0.0024)
      throw "Support-normal seating must not claim final jaw alignment";
    var tilt=[Math.sin(0.0006),0.0,0,Math.cos(0.0006)];
    if(orientationError(tilt,identity,TendingSeatOrientation.SupportNormal)<=0.001)
      throw "Support-normal seating accepted excessive tilt";
    if(orientationError([0.0,0,0,-1],identity,TendingSeatOrientation.FullPose)>1e-7)
      throw "Quaternion sign changed the full-pose result";
    if(orientationError([1.0,0,0,0],identity,TendingSeatOrientation.SupportNormal)<3)
      throw "Inverted stock passed the support-normal check";
    var rejected=false;try parseOrientation("anything") catch(_:Dynamic)rejected=true;
    if(!rejected)throw "Unknown seating constraint accepted";
    return 5;
  }
}
