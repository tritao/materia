package motionkit.path;

import motionkit.kinematics.Pose3;

class PoseMath {
  public static function distance(a:Pose3, b:Pose3):Float {
    var x = b.x - a.x, y = b.y - a.y, z = b.z - a.z;
    return Math.sqrt(x * x + y * y + z * z);
  }

  public static function angle(a:Pose3, b:Pose3):Float {
    var dot = Math.abs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw);
    return 2.0 * Math.acos(Math.min(1.0, dot));
  }

  public static function interpolate(a:PoseWaypoint, b:PoseWaypoint, t:Float,
      policy:OrientationPolicy, x:Float, y:Float, z:Float):PoseWaypoint {
    var q = quaternion(a.pose, b.pose, t, policy);
    return new PoseWaypoint(new Pose3(x, y, z, q[0], q[1], q[2], q[3]),
      a.positionTolerance + t * (b.positionTolerance - a.positionTolerance),
      a.orientationTolerance + t * (b.orientationTolerance - a.orientationTolerance));
  }

  /** Spatial angular derivatives of the exact interpolation used below. */
  public static function angularRates(a:Pose3,b:Pose3,t:Float,policy:OrientationPolicy,
      length:Float):{first:Array<Float>,second:Array<Float>} {
    if(!Math.isFinite(t) || t<0 || t>1 || !Math.isFinite(length) || length<=0)
      throw "Orientation derivatives require a valid interpolation distance";
    switch policy {
      case Fixed:return {first:[0.0,0.0,0.0],second:[0.0,0.0,0.0]};
      case Cone(axis,halfAngle):
        if(axis==null || axis.length!=3 || !Math.isFinite(halfAngle) || halfAngle<0)throw "Invalid orientation cone";
      default:
    }
    var dot=a.qx*b.qx+a.qy*b.qy+a.qz*b.qz+a.qw*b.qw;
    var sign=dot<0?-1.0:1.0;dot=Math.min(1.0,Math.abs(dot));
    // Vector part of b * conjugate(a), in the path reference frame.
    var axis=[sign*(-b.qw*a.qx+b.qx*a.qw-b.qy*a.qz+b.qz*a.qy),
      sign*(-b.qw*a.qy+b.qx*a.qz+b.qy*a.qw-b.qz*a.qx),
      sign*(-b.qw*a.qz-b.qx*a.qy+b.qy*a.qx+b.qz*a.qw)];
    var sine=Math.sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
    if(sine==0)return {first:[0.0,0.0,0.0],second:[0.0,0.0,0.0]};
    var rate=2*Math.atan2(sine,dot)/length,acceleration=0.0;
    if(dot>=0.9995){
      var x=1+t*(dot-1),y=t*sine,denominator=x*x+y*y;
      rate=2*sine/(denominator*length);
      acceleration=-4*sine*(x*(dot-1)+y*sine)/(denominator*denominator*length*length);
    }
    return {first:[for(value in axis)value*rate/sine],second:[for(value in axis)value*acceleration/sine]};
  }

  static function quaternion(a:Pose3, b:Pose3, t:Float, policy:OrientationPolicy):Array<Float> {
    switch (policy) {
      case Fixed: return a.rotationArray();
      case Cone(axis, halfAngle):
        if (axis == null || axis.length != 3 || !Math.isFinite(halfAngle) || halfAngle < 0.0)
          throw "Invalid orientation cone";
      case Interpolated | FreeAboutTool | Free:
    }
    var dot = a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw;
    var sign = dot < 0.0 ? -1.0 : 1.0;
    dot = Math.abs(dot);
    var left = 1.0 - t, right = t;
    if (dot < 0.9995) {
      var angle = Math.acos(Math.min(1.0, dot));
      var scale = Math.sin(angle);
      left = Math.sin((1.0 - t) * angle) / scale;
      right = Math.sin(t * angle) / scale;
    }
    var q = [left * a.qx + right * sign * b.qx,
      left * a.qy + right * sign * b.qy,
      left * a.qz + right * sign * b.qz,
      left * a.qw + right * sign * b.qw];
    var norm = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return [q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm];
  }
}
