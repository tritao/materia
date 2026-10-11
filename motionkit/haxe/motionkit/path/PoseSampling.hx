package motionkit.path;

/** Shared authored grid density for selection and retained execution. */
class PoseSampling {
  public static function pieces(primitive:PosePrimitive,resolution:Float,angularResolution:Float=0.25):Int {
    if(primitive==null || !Math.isFinite(resolution) || resolution<=0 || !Math.isFinite(angularResolution) || angularResolution<=0)
      throw "Pose sampling requires a primitive and positive finite resolution";
    var length=primitive.length(),curvature=0.0,angularRate=0.0;
    for(at in [0.0,0.5*length,length]){
      var derivatives=primitive.derivativesAt(at),second=derivatives.linearSecond,angular=derivatives.angular;
      curvature=Math.max(curvature,Math.sqrt(second[0]*second[0]+second[1]*second[1]+second[2]*second[2]));
      angularRate=Math.max(angularRate,Math.sqrt(angular[0]*angular[0]+angular[1]*angular[1]+angular[2]*angular[2]));
    }
    return Std.int(Math.max(1,Math.max(Math.ceil(length*angularRate/angularResolution),
      Math.max(Math.ceil(length/resolution),Math.ceil(length*curvature/0.25)))));
  }
}
