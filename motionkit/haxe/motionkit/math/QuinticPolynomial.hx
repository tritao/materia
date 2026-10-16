package motionkit.math;

/** Shared normalized quintics and Bernstein controls for path interpolation and geometric bounds. */
class QuinticPolynomial {
  public static function hermite(start:Array<Float>,end:Array<Float>,first:Array<Float>,last:Array<Float>,
      second:Array<Float>,before:Array<Float>,h:Float):Array<Array<Float>> {
    return [for(j in 0...start.length){
      var a=start[j],b=h*first[j],c=h*h*second[j]/2;
      var r0=end[j]-a-b-c,r1=h*last[j]-b-2*c,r2=h*h*before[j]-2*c;
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
  public static function split(control:Array<Array<Float>>,at:Float=0.5):Array<Array<Array<Float>>> {
    if(!Math.isFinite(at) || at<0 || at>1)throw "Polynomial split outside unit interval";
    var left:Array<Array<Float>> = [],right:Array<Array<Float>> = [];
    for(row in control){var levels=row.copy(),a=[levels[0]],b=[levels[5]];
      for(n in 1...6){levels=[for(i in 0...levels.length-1)(at==0.5 ? (levels[i]+levels[i+1])/2 : (1-at)*levels[i]+at*levels[i+1])];a.push(levels[0]);b.unshift(levels[levels.length-1]);}
      left.push(a);right.push(b);
    }
    return [left,right];
  }
  /** Restrict Bernstein controls to an exact subinterval, still parameterized on [0,1]. */
  public static function slice(control:Array<Array<Float>>,from:Float,to:Float):Array<Array<Float>> {
    if(!Math.isFinite(from) || !Math.isFinite(to) || from<0 || to>1 || from>=to)
      throw "Polynomial slice outside unit interval";
    var left=to==1 ? control : split(control,to)[0];
    return from==0 ? left : split(left,from/to)[1];
  }
  /** Convexity bounds the Euclidean norm of a vector polynomial by its controls. */
  public static function differenceBound(a:Array<Array<Float>>,b:Array<Array<Float>>):Float {
    var bound=0.0,scale=0.0;
    for(k in 0...6){
      var sum=0.0;
      for(axis in 0...a.length){var d=a[axis][k]-b[axis][k];sum+=d*d;
        scale=Math.max(scale,Math.abs(a[axis][k])+Math.abs(b[axis][k]));}
      bound=Math.max(bound,Math.sqrt(sum));
    }
    return bound+512*2.220446049250313e-16*scale;
  }
}
