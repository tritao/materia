package motionkit.planner;

/** The normalized quintic Hermite polynomial consumed by native path.cpp. */
class JointPathPolynomial {
  public static function coefficients(path:JointPathSamples,span:Int):Array<Array<Float>> {
    var h=path.s[span+1]-path.s[span];
    return [for(j in 0...path.jointCount){
      var a=path.q[span][j],b=h*path.qPrime[span][j],c=h*h*path.qDoublePrime[span][j]/2;
      var r0=path.q[span+1][j]-a-b-c,r1=h*path.qPrime[span+1][j]-b-2*c,
        r2=h*h*path.qDoublePrimeBefore[span+1][j]-2*c;
      [a,b,c,10*r0-4*r1+r2/2,-15*r0+7*r1-r2,6*r0-3*r1+r2/2];
    }];
  }
  public static function at(power:Array<Array<Float>>,u:Float):Array<Float>
    return [for(row in power){var v=0.0;for(k in 0...row.length)v=v*u+row[row.length-1-k];v;}];
  static function choose(n:Int,k:Int):Float {
    var value=1.0;for(i in 0...k)value*= (n-i)/(i+1);return value;
  }
  public static function bernstein(power:Array<Array<Float>>):Array<Array<Float>>
    return [for(row in power)[for(k in 0...6){var v=0.0;for(i in 0...k+1)v+=choose(k,i)/choose(5,i)*row[i];v;}]];
  public static function split(control:Array<Array<Float>>):Array<Array<Array<Float>>> {
    var left:Array<Array<Float>> = [],right:Array<Array<Float>> = [];
    for(row in control){var levels=row.copy(),a=[levels[0]],b=[levels[5]];
      for(n in 1...6){levels=[for(i in 0...levels.length-1)(levels[i]+levels[i+1])/2];a.push(levels[0]);b.unshift(levels[levels.length-1]);}
      left.push(a);right.push(b);
    }
    return [left,right];
  }
}
