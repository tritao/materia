package materia.sheet;

typedef SheetPoint = {x:Float,y:Float};
typedef SheetShape = Array<Array<SheetPoint>>;
typedef SheetShapePlacement = {id:String,requirementId:String,x:Float,y:Float,angle:Float,loops:SheetShape};
typedef SheetRemnant = {id:String,x:Float,y:Float,width:Float,height:Float};
typedef SheetBounds2 = {minX:Float,minY:Float,maxX:Float,maxY:Float};

/** Polygonal layouts in mm on a 1 µm grid. Predicates use exact integer products within the bounded domain. */
class SheetPolygon {
  public static inline final QUANTUM=0.001;
  public static inline final EPS=0.0000001;
  public static function snap(value:Float):Float {
    if(!Math.isFinite(value)||Math.abs(value)>10000)throw "Sheet geometry exceeds the finite ±10 m domain";
    return Math.round(value/QUANTUM)*QUANTUM;
  }
  public static function point(x:Float,y:Float):SheetPoint return {x:snap(x),y:snap(y)};
  public static function cross(a:SheetPoint,b:SheetPoint,c:SheetPoint):Float {
    var ax:Float=Math.round(a.x/QUANTUM),ay:Float=Math.round(a.y/QUANTUM);
    return (Math.round(b.x/QUANTUM)-ax)*(Math.round(c.y/QUANTUM)-ay)-
      (Math.round(b.y/QUANTUM)-ay)*(Math.round(c.x/QUANTUM)-ax);
  }
  public static function area(loop:Array<SheetPoint>):Float {
    var sum=0.0;for(i in 0...loop.length){var a=loop[i],b=loop[(i+1)%loop.length];sum+=a.x*b.y-b.x*a.y;}return sum/2;
  }
  public static function materialArea(shape:SheetShape):Float {
    var sum=Math.abs(area(shape[0]));for(i in 1...shape.length)sum-=Math.abs(area(shape[i]));return sum;
  }
  public static function bounds(shape:SheetShape):SheetBounds2 {
    var b:SheetBounds2={minX:Math.POSITIVE_INFINITY,minY:Math.POSITIVE_INFINITY,maxX:Math.NEGATIVE_INFINITY,maxY:Math.NEGATIVE_INFINITY};
    for(p in shape[0]){b.minX=Math.min(b.minX,p.x);b.minY=Math.min(b.minY,p.y);b.maxX=Math.max(b.maxX,p.x);b.maxY=Math.max(b.maxY,p.y);}return b;
  }
  public static function on(a:SheetPoint,b:SheetPoint,p:SheetPoint):Bool return cross(a,b,p)==0&&
    p.x>=Math.min(a.x,b.x)-EPS&&p.x<=Math.max(a.x,b.x)+EPS&&p.y>=Math.min(a.y,b.y)-EPS&&p.y<=Math.max(a.y,b.y)+EPS;
  public static function intersects(a:SheetPoint,b:SheetPoint,c:SheetPoint,d:SheetPoint):Bool {
    var x=cross(a,b,c),y=cross(a,b,d),u=cross(c,d,a),v=cross(c,d,b);
    return ((x<0&&y>0)||(x>0&&y<0))&&((u<0&&v>0)||(u>0&&v<0))||
      x==0&&on(a,b,c)||y==0&&on(a,b,d)||u==0&&on(c,d,a)||v==0&&on(c,d,b);
  }
  public static function inside(loop:Array<SheetPoint>,p:SheetPoint):Bool {
    var winding=0;
    for(i in 0...loop.length) {
      var a=loop[i],b=loop[(i+1)%loop.length];
      if(on(a,b,p))return true;
      if(a.y<=p.y&&b.y>p.y&&cross(a,b,p)>0)winding++;
      if(a.y>p.y&&b.y<=p.y&&cross(a,b,p)<0)winding--;
    }
    return winding!=0;
  }
  public static function contains(shape:SheetShape,p:SheetPoint):Bool {
    if(!inside(shape[0],p))return false;
    for(i in 1...shape.length)if(inside(shape[i],p))return false;return true;
  }
  public static function overlap(a:SheetShape,b:SheetShape):Bool {
    var x=bounds(a),y=bounds(b);
    if(x.maxX<y.minX-EPS||y.maxX<x.minX-EPS||x.maxY<y.minY-EPS||y.maxY<x.minY-EPS)return false;
    for(l in a)for(k in b)for(i in 0...l.length)for(j in 0...k.length)
      if(intersects(l[i],l[(i+1)%l.length],k[j],k[(j+1)%k.length]))return true;
    return contains(a,b[0][0])||contains(b,a[0][0]);
  }
  static function pointDistance(p:SheetPoint,a:SheetPoint,b:SheetPoint):Float {
    var dx=b.x-a.x,dy=b.y-a.y,t=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy)));
    var x=p.x-a.x-t*dx,y=p.y-a.y-t*dy;return Math.sqrt(x*x+y*y);
  }
  public static function distance(a:SheetShape,b:SheetShape):Float {
    if(overlap(a,b))return 0;
    var best=Math.POSITIVE_INFINITY;
    for(l in a)for(k in b)for(i in 0...l.length)for(j in 0...k.length) {
      var p=l[i],q=l[(i+1)%l.length],r=k[j],s=k[(j+1)%k.length];
      best=Math.min(best,Math.min(Math.min(pointDistance(p,r,s),pointDistance(q,r,s)),Math.min(pointDistance(r,p,q),pointDistance(s,p,q))));
    }
    return best;
  }
  public static function separated(a:SheetShape,b:SheetShape,spacing:Float):Bool {
    var x=bounds(a),y=bounds(b);
    if(x.maxX+spacing<y.minX||y.maxX+spacing<x.minX||x.maxY+spacing<y.minY||y.maxY+spacing<x.minY)return true;
    return !overlap(a,b)&&distance(a,b)+EPS>=spacing;
  }
  public static function transform(shape:SheetShape,angle:Float,x:Float,y:Float):SheetShape {
    if(!Math.isFinite(angle))throw "Nonfinite sheet angle";
    var a=(angle%360+360)%360,c=Math.cos(a*Math.PI/180),s=Math.sin(a*Math.PI/180);
    if(a==0){c=1;s=0;}else if(a==90){c=0;s=1;}else if(a==180){c=-1;s=0;}else if(a==270){c=0;s=-1;}
    return [for(loop in shape)[for(p in loop)point(c*p.x-s*p.y+x,s*p.x+c*p.y+y)]];
  }
  public static function validate(shape:SheetShape):Void {
    if(shape==null||shape.length==0||shape.length>128)throw "Sheet shape requires an outer polygon and optional holes";
    var edges=0;
    for(loop in shape) {
      if(loop==null||loop.length<3)throw "Sheet polygon requires three vertices";
      edges+=loop.length;if(edges>4096)throw "Sheet polygon exceeds 4096 edges";
      for(i in 0...loop.length) {
        var p=loop[i],q=loop[(i+1)%loop.length];
        if(p==null||Math.abs(p.x-snap(p.x))>EPS||Math.abs(p.y-snap(p.y))>EPS||p.x==q.x&&p.y==q.y)
          throw "Sheet vertices must be distinct and lie on the 1 µm grid";
        for(j in i+1...loop.length) {
          if(j==i+1||i==0&&j==loop.length-1)continue;
          if(intersects(p,q,loop[j],loop[(j+1)%loop.length]))throw "Sheet polygon crosses or touches itself";
        }
      }
      if(Math.abs(area(loop))<QUANTUM*QUANTUM)throw "Sheet polygon has no material area";
    }
    for(i in 1...shape.length) {
      if(!inside(shape[0],shape[i][0]))throw "Sheet hole lies outside its outer boundary";
      for(j in 0...i) {
        for(a in 0...shape[i].length)for(b in 0...shape[j].length)
          if(intersects(shape[i][a],shape[i][(a+1)%shape[i].length],shape[j][b],shape[j][(b+1)%shape[j].length]))throw "Sheet holes touch or cross boundaries";
        if(j>0&&(inside(shape[i],shape[j][0])||inside(shape[j],shape[i][0])))throw "Sheet holes overlap or nest";
      }
    }
    if(materialArea(shape)<=0)throw "Sheet holes consume the part";
  }
  public static function normalized(shape:SheetShape):SheetShape {
    var result=transform(shape,0,0,0);validate(result);
    for(i in 0...result.length)if((area(result[i])>0)!=(i==0))result[i].reverse();return result;
  }
  public static function rectangle(x:Float,y:Float,w:Float,h:Float):SheetShape return [[point(x,y),point(x+w,y),point(x+w,y+h),point(x,y+h)]];
}
