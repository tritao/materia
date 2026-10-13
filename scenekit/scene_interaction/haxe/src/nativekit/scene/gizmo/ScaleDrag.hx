package nativekit.scene.gizmo;

/** Absolute dimensionless factor from pointer-down; invalid/zero/negative factors retain the last proposal. */
class ScaleDrag {
  public final handle:ScaleHandle;
  public var lastFactor(default,null):Float=1;
  public var closed(default,null):Bool=false;
  final startX:Float; final startY:Float;
  final directionX:Float; final directionY:Float;
  final span:Float; final snap:Float;
  @:allow(nativekit.scene.gizmo.ScaleGizmo)
  private function new(handle:ScaleHandle,x:Float,y:Float,dx:Float,dy:Float,span:Float,snap:Float) {
    if(!Math.isFinite(snap) || snap<0 || snap>1) throw "Invalid scale factor snap";
    this.handle=handle;startX=x;startY=y;directionX=dx;directionY=dy;this.span=span;this.snap=snap;
  }
  public function update(x:Float,y:Float):Null<Float> {
    if(closed) throw "Scale drag is closed";
    if(!Math.isFinite(x) || !Math.isFinite(y)) return null;
    var movement=((x-startX)*directionX+(y-startY)*directionY)/span;
    var factor=1+(snap==0 ? movement : Math.fround(movement/snap)*snap);
    if(!Math.isFinite(factor) || factor<=0 || factor>1000000) return null;
    lastFactor=factor;return factor;
  }
  public function factors(factor:Float):Array<Float> {
    if(!Math.isFinite(factor) || factor<=0 || factor>1000000) throw "Invalid scale factor";
    return switch handle {case ScaleX:[factor,1.0,1.0];case ScaleY:[1.0,factor,1.0];case ScaleZ:[1.0,1.0,factor];case Uniform:[factor,factor,factor];};
  }
  public function finish():Float {if(closed) throw "Scale drag is closed";closed=true;return lastFactor;}
  public function cancel():Void {if(closed) throw "Scale drag is closed";closed=true;}
}
