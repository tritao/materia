package motionkit.robot;

/** Natural C2 cubic spline in path distance. Periodic values are unwrapped
 * before fitting; outputs remain unwrapped so their derivatives stay continuous.
 * Bounds are checked by the downstream analytic refinement, since cubic
 * interpolation can overshoot its knots. */
class RedundancySpline implements RedundancyCurve {
  final distances:Array<Float>;
  final values:Array<Float>;
  final coefficients:Array<Array<Float>>;
  public function new(distances:Array<Float>,values:Array<Float>,period:Float=0) {
    if(distances==null || values==null || distances.length<2 || values.length!=distances.length ||
        !Math.isFinite(period) || period<0)throw "Spline needs aligned finite knots and a nonnegative period";
    this.distances=distances.copy();this.values=values.copy();
    var n=distances.length;
    for(i in 0...n){
      if(!Math.isFinite(distances[i]) || !Math.isFinite(values[i]) || i>0 && distances[i]<=distances[i-1])
        throw "Spline knots must be finite with increasing path distance";
      if(period>0 && i>0){var delta=values[i]-this.values[i-1];
        if(!Math.isFinite(delta/period))throw "Spline periodic difference is unrepresentable";
        delta-=period*Math.floor(delta/period+0.5);this.values[i]=this.values[i-1]+delta;
      }
    }
    var h=[for(i in 0...n-1)distances[i+1]-distances[i]];
    for(span in h)if(!Math.isFinite(span))throw "Spline interval is unrepresentable";
    var diagonal=[for(_ in 0...n)0.0],rhs=[for(_ in 0...n)0.0],second=[for(_ in 0...n)0.0];
    for(i in 1...n-1){diagonal[i]=2*(h[i-1]+h[i]);
      rhs[i]=6*((this.values[i+1]-this.values[i])/h[i]-(this.values[i]-this.values[i-1])/h[i-1]);}
    for(i in 2...n-1){var factor=h[i-1]/diagonal[i-1];diagonal[i]-=factor*h[i-1];rhs[i]-=factor*rhs[i-1];}
    var i=n-2;while(i>=1){second[i]=(rhs[i]-h[i]*second[i+1])/diagonal[i];i--;}
    coefficients=[];
    for(i in 0...n-1){var row=[this.values[i],(this.values[i+1]-this.values[i])/h[i]-h[i]*(2*second[i]+second[i+1])/6,
      second[i]/2,(second[i+1]-second[i])/(6*h[i])];
      for(value in row)if(!Math.isFinite(value))throw "Spline coefficients are unrepresentable";
      coefficients.push(row);
    }
  }
  public function evaluate(distance:Float):SplineSample {
    if(!Math.isFinite(distance) || distance<distances[0] || distance>distances[distances.length-1])
      throw "Spline evaluation is outside its path range";
    var lo=0,hi=distances.length-1;
    while(hi-lo>1){var mid=Std.int((lo+hi)/2);if(distance<distances[mid])hi=mid;else lo=mid;}
    var x=distance-distances[lo],c=coefficients[lo];
    var sample=new SplineSample(c[0]+x*(c[1]+x*(c[2]+x*c[3])),c[1]+x*(2*c[2]+3*x*c[3]),2*c[2]+6*x*c[3]);
    if(!Math.isFinite(sample.value) || !Math.isFinite(sample.first) || !Math.isFinite(sample.second))
      throw "Spline sample is unrepresentable";
    return sample;
  }
}
class SplineSample {
  public final value:Float;
  public final first:Float;
  public final second:Float;
  public function new(value:Float,first:Float,second:Float){this.value=value;this.first=first;this.second=second;}
}
