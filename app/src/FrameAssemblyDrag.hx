package app;

/** Keeps the latest pointer target until a host frame; release always applies the final target. */
class FrameAssemblyDrag implements SceneAssemblyDrag {
  final drag:SceneAssemblyDrag;
  var pending:Null<Array<Float>>;
  var freeDetents:Bool = false;
  var finished:Bool = false;

  public function new(drag:SceneAssemblyDrag) this.drag = drag;

  public function update(x:Float, y:Float, z:Float):Void {
    if (!finished && Math.isFinite(x) && Math.isFinite(y) && Math.isFinite(z)) pending = [x, y, z];
  }

  public function flush():Bool {
    var target = pending;
    if (finished || target == null) return false;
    pending = null;
    drag.setFreeDetents(freeDetents);
    drag.update(target[0], target[1], target[2]);
    return true;
  }

  public function setFreeDetents(free:Bool):Void freeDetents = free;
  public function grabbedPoint():Array<Float> return drag.grabbedPoint();
  public function target():Array<Float> return pending == null ? drag.target() : pending.copy();
  public function following():Bool return drag.following();
  public function message():String return drag.message();

  public function commit():Bool {
    if (finished) return false;
    flush();
    finished = true;
    return drag.commit();
  }

  public function cancel():Void {
    if (finished) return;
    pending = null;
    finished = true;
    drag.cancel();
  }
}
