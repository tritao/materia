package collisionkit;

/** Axis-aligned bounds after a shape pose, with a conservative radius about its origin. */
class CollisionBounds {
  public final centre:{x:Float, y:Float, z:Float};
  public final halfExtents:{x:Float, y:Float, z:Float};
  public final radius:Float;

  function new(lo:Array<Float>, hi:Array<Float>, radius:Float) {
    centre = {x:(lo[0]+hi[0])/2, y:(lo[1]+hi[1])/2, z:(lo[2]+hi[2])/2};
    halfExtents = {x:(hi[0]-lo[0])/2, y:(hi[1]-lo[1])/2, z:(hi[2]-lo[2])/2};
    this.radius = radius;
  }

  /** Eight conservative corners, using the same bounds as placement and motion queries. */
  public function corners():Array<Float> {
    var c = centre, h = halfExtents;
    return [for (i in 0...8) for (a in 0...3)
      (a == 0 ? c.x : a == 1 ? c.y : c.z) +
      ((i & (1 << a)) == 0 ? -1 : 1) * (a == 0 ? h.x : a == 1 ? h.y : h.z)];
  }

  public static function of(geometry:CollisionGeometry, pose:CollisionPose, inflation:Float = 0):CollisionBounds {
    if (geometry == null || pose == null || !Math.isFinite(inflation) || inflation < 0)
      throw "Collision bounds require geometry, pose and nonnegative inflation";
    for (v in [pose.x,pose.y,pose.z,pose.qx,pose.qy,pose.qz,pose.qw])
      if (!Math.isFinite(v)) throw "Collision bounds pose must be finite";
    var norm = pose.qx*pose.qx+pose.qy*pose.qy+pose.qz*pose.qz+pose.qw*pose.qw;
    if (Math.abs(norm-1)>1e-6) throw "Collision bounds rotation must be unit";
    var lo = [Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY,Math.POSITIVE_INFINITY];
    var hi = [Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY,Math.NEGATIVE_INFINITY];
    function point(x:Float,y:Float,z:Float):Void {
      if (!Math.isFinite(x)||!Math.isFinite(y)||!Math.isFinite(z)) throw "Collision bounds point must be finite";
      var tx=2*(pose.qy*z-pose.qz*y), ty=2*(pose.qz*x-pose.qx*z), tz=2*(pose.qx*y-pose.qy*x);
      var p=[pose.x+x+pose.qw*tx+pose.qy*tz-pose.qz*ty,
        pose.y+y+pose.qw*ty+pose.qz*tx-pose.qx*tz,
        pose.z+z+pose.qw*tz+pose.qx*ty-pose.qy*tx];
      for (a in 0...3) {lo[a]=Math.min(lo[a],p[a]); hi[a]=Math.max(hi[a],p[a]);}
    }
    function positive(v:Float):Void if (!Math.isFinite(v)||v<=0) throw "Collision bounds dimensions must be positive";
    var radius=0.0;
    switch geometry {
      case Box(x,y,z):
        positive(x); positive(y); positive(z);
        for (sx in [-1,1]) for (sy in [-1,1]) for (sz in [-1,1]) point(sx*x,sy*y,sz*z);
        radius=Math.sqrt(x*x+y*y+z*z);
      case Sphere(r):
        positive(r); lo=[pose.x-r,pose.y-r,pose.z-r]; hi=[pose.x+r,pose.y+r,pose.z+r]; radius=r;
      case Capsule(r,h), Cylinder(r,h):
        positive(r); positive(h);
        var axis=[2*(pose.qx*pose.qz+pose.qw*pose.qy),2*(pose.qy*pose.qz-pose.qw*pose.qx),1-2*(pose.qx*pose.qx+pose.qy*pose.qy)];
        var capsule=switch geometry {case Capsule(_,_):true;case _:false;};
        var c=[pose.x,pose.y,pose.z];
        for (a in 0...3) {
          var extent=h*Math.abs(axis[a])+r*(capsule?1:Math.sqrt(Math.max(0,1-axis[a]*axis[a])));
          lo[a]=c[a]-extent; hi[a]=c[a]+extent;
        }
        radius=r+h;
      case Convex(points), Mesh(points,_):
        if (points==null||points.length<3||points.length%3!=0) throw "Collision bounds need XYZ points";
        for (i in 0...Std.int(points.length/3)) {
          var x=points[3*i],y=points[3*i+1],z=points[3*i+2]; point(x,y,z);
          radius=Math.max(radius,Math.sqrt(x*x+y*y+z*z));
        }
      case HeightField(x,y,heights,rows,min):
        positive(x);positive(y);
        if (heights==null||heights.length==0) throw "Collision bounds need heights";
        var max=min; for (h in heights) {if (!Math.isFinite(h)) throw "Collision bounds height must be finite";max=Math.max(max,h);}
        for (sx in [-1,1]) for (sy in [-1,1]) for (z in [min,max]) point(sx*x/2,sy*y/2,z);
        radius=Math.sqrt(x*x/4+y*y/4+Math.max(min*min,max*max));
      case HalfSpace(_,_,_,_): throw "A half-space has no finite bounds";
    }
    for (a in 0...3) {lo[a]-=inflation;hi[a]+=inflation;}
    return new CollisionBounds(lo,hi,radius+inflation);
  }
}
