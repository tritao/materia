package cadkit.drawing;

import cadkit.sheetmetal.Point2;
import cadkit.sheetmetal.Segment;

/** Positive-weight rational B-splines, split into Bezier spans before tolerance-bounded flattening. */
class DxfSpline {
  static function mix(a:Array<Float>,b:Array<Float>,t:Float):Array<Float>
    return [a[0]*(1-t)+b[0]*t,a[1]*(1-t)+b[1]*t,a[2]*(1-t)+b[2]*t];
  static function point(p:Array<Float>):Point2 return new Point2(p[0]/p[2],p[1]/p[2]);
  static function distance(p:Point2,a:Point2,b:Point2):Float {
    var dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
    var t=den==0?0:Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/den));
    return p.distance(new Point2(a.x+t*dx,a.y+t*dy));
  }
  static function flatten(control:Array<Array<Float>>,tolerance:Float,out:Array<Segment>,depth:Int):Void {
    var a=point(control[0]),b=point(control[control.length-1]),error=0.0;
    for(p in control) error=Math.max(error,distance(point(p),a,b));
    if(error<=tolerance) {
      if(a.distance(b)>1e-9) out.push(Segment.line(a,b));
      return;
    }
    if(depth>=32||out.length>=1000000)throw "DXF spline subdivision limit exceeded";
    var row=control.copy(),left=[row[0]],right=[row[row.length-1]];
    while(row.length>1) {
      row=[for(i in 0...row.length-1) mix(row[i],row[i+1],0.5)];
      left.push(row[0]);right.push(row[row.length-1]);
    }
    right.reverse();flatten(left,tolerance,out,depth+1);flatten(right,tolerance,out,depth+1);
  }
  public static function sample(points:Array<Point2>,weights:Array<Float>,knots:Array<Float>,degree:Int,tolerance:Float):Array<Segment> {
    if(degree<1||degree>16||points.length<=degree||knots.length!=points.length+degree+1||
      weights.length!=points.length||!Math.isFinite(tolerance)||tolerance<=0)throw "Invalid DXF spline dimensions";
    for(i in 0...knots.length)if(!Math.isFinite(knots[i])||(i>0&&knots[i]<knots[i-1]))throw "Invalid DXF spline knots";
    for(w in weights)if(!Math.isFinite(w)||w<=0)throw "DXF spline requires positive rational weights";
    var p=[for(i in 0...points.length) [points[i].x*weights[i],points[i].y*weights[i],weights[i]]];
    var u=knots.copy(),lo=u[degree],hi=u[points.length];
    if(hi<=lo)throw "Empty DXF spline domain";
    // Insert each domain knot until adjacent spans have independent Bezier controls.
    var unique:Array<Float> = [];
    for(value in u)if(value>=lo&&value<=hi&&unique.indexOf(value)<0)unique.push(value);
    for(value in unique) {
      var multiplicity=0;for(v in u)if(v==value)multiplicity++;
      if(multiplicity>degree+1)throw "Invalid DXF spline knot multiplicity";
      while(multiplicity<degree) {
        var n=p.length-1,k=degree;
        // At an unclamped domain end the insertion span can lie beyond n;
        // clamping it to n would change the endpoint instead of refining it.
        while(k+1<u.length&&u[k+1]<=value)k++;
        var q:Array<Array<Float>> = [for(i in 0...p.length+1) [0.0,0.0,1.0]];
        for(i in 0...k-degree+1)q[i]=p[i];
        for(i in k-multiplicity...n+1)q[i+1]=p[i];
        for(i in k-degree+1...k-multiplicity+1) {
          var den=u[i+degree]-u[i];
          if(den<=0)throw "Invalid DXF spline knot insertion";
          q[i]=mix(p[i-1],p[i],(value-u[i])/den);
        }
        u.insert(k+1,value);p=q;multiplicity++;
      }
    }
    var out:Array<Segment> = [];
    for(k in degree...p.length)if(u[k]>=lo&&u[k+1]<=hi&&u[k+1]>u[k])
      flatten([for(i in k-degree...k+1)p[i]],tolerance,out,0);
    if(out.length==0)throw "DXF spline is degenerate at the requested tolerance";
    return out;
  }
}
