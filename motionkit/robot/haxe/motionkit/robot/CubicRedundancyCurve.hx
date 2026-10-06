package motionkit.robot;

import motionkit.robot.RedundancySpline.SplineSample;

/** C2 Hermite cubic supplied by the native constrained optimizer. */
class CubicRedundancyCurve implements RedundancyCurve {
  final distances:Array<Float>;
  final coefficients:Array<Array<Float>>;
  public function new(distances:Array<Float>,values:Array<Float>,first:Array<Float>,?lower:Float,?upper:Float) {
    if(distances==null || distances.length<2 || values==null || first==null ||
        values.length!=distances.length || first.length!=distances.length)
      throw "Constrained curve requires aligned values and derivatives";
    this.distances=distances.copy();coefficients=[];
    for(i in 0...distances.length)if(!Math.isFinite(distances[i]) || !Math.isFinite(values[i]) ||
        !Math.isFinite(first[i]) || i>0 && distances[i]<=distances[i-1])throw "Invalid constrained curve knot";
    for(i in 0...distances.length-1){
      var h=distances[i+1]-distances[i],delta=values[i+1]-values[i];
      coefficients.push([values[i],first[i],3*delta/(h*h)-(2*first[i]+first[i+1])/h,
        -2*delta/(h*h*h)+(first[i]+first[i+1])/(h*h)]);
    }
    if(lower!=null && upper!=null){
      var lo:Float=cast lower,hi:Float=cast upper,min=Math.POSITIVE_INFINITY,max=Math.NEGATIVE_INFINITY;
      if(!Math.isFinite(lo) || !Math.isFinite(hi) || lo>hi)throw "Invalid constrained curve interval";
      for(i in 0...coefficients.length){var h=distances[i+1]-distances[i],c=coefficients[i];
        for(value in [values[i],values[i]+h*first[i]/3,values[i+1]-h*first[i+1]/3,values[i+1]]){
          min=Math.min(min,value);max=Math.max(max,value);
        }
      }
      // QP feasibility has a numerical tolerance. A uniform affine correction
      // puts its Bernstein polygon inside the exact physical interval while
      // preserving C2 and derivative consistency. Exact arm IK is checked after.
      if(min<lo || max>hi){
        if(min<lo-1e-7 || max>hi+1e-7)throw "Constrained curve violates its physical interval";
        var scale=1.0,shift=0.0;
        if(hi==lo){scale=0;shift=lo;}
        else if(max-min>hi-lo){scale=(hi-lo)/(max-min);shift=lo-scale*min;}
        else shift=min<lo?lo-min:hi-max;
        for(c in coefficients){c[0]=scale*c[0]+shift;for(j in 1...4)c[j]*=scale;}
      }
    }
  }
  public function evaluate(distance:Float):SplineSample {
    if(!Math.isFinite(distance) || distance<distances[0] || distance>distances[distances.length-1])
      throw "Constrained curve evaluation is outside its path range";
    var lo=0,hi=distances.length-1;
    while(hi-lo>1){var mid=Std.int((lo+hi)/2);if(distance<distances[mid])hi=mid;else lo=mid;}
    var x=distance-distances[lo],c=coefficients[lo];
    return new SplineSample(c[0]+x*(c[1]+x*(c[2]+x*c[3])),c[1]+x*(2*c[2]+3*x*c[3]),2*c[2]+6*x*c[3]);
  }
}
