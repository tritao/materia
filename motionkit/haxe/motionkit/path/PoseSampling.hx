package motionkit.path;

/** Shared authored grid density for selection and retained execution. */
class PoseSampling {
  /**
    Pieces at most `resolution` long, a quarter radian of curvature apart and `angularResolution` radians of turning
    apart. `exactAffine` says the joints are an exact affine function of the pose (`AffineJointMap`): a straight piece
    that does not turn the tool is then a straight line in joint space too, however long, and one piece is all it needs.
  **/
  public static function pieces(primitive:PosePrimitive,resolution:Float,angularResolution:Float=0.25,?exactAffine:Bool=false):Int {
    if(primitive==null || !Math.isFinite(resolution) || resolution<=0 || !Math.isFinite(angularResolution) || angularResolution<=0)
      throw "Pose sampling requires a primitive and positive finite resolution";
    var length=primitive.length(),curvature=0.0,angularRate=0.0,turning=0.0;
    for(at in [0.0,0.5*length,length]){
      var derivatives=primitive.derivativesAt(at),second=derivatives.linearSecond,angular=derivatives.angular;
      curvature=Math.max(curvature,Math.sqrt(second[0]*second[0]+second[1]*second[1]+second[2]*second[2]));
      angularRate=Math.max(angularRate,Math.sqrt(angular[0]*angular[0]+angular[1]*angular[1]+angular[2]*angular[2]));
      for(value in derivatives.angular.concat(derivatives.angularSecond))turning=Math.max(turning,Math.abs(value));
    }
    if(exactAffine && curvature==0.0 && turning==0.0)return 1;
    return Std.int(Math.max(1,Math.max(Math.ceil(length*angularRate/angularResolution),
      Math.max(Math.ceil(length/resolution),Math.ceil(length*curvature/0.25)))));
  }
}
