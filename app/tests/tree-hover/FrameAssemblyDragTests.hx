import app.FrameAssemblyDrag;
import app.SceneAssemblyDrag;

private class DragProbe implements SceneAssemblyDrag {
  public var updates = 0;
  public var commits = 0;
  public var cancels = 0;
  public var free = false;
  var point:Array<Float> = [0, 0, 0];
  public function new() {}
  public function update(x:Float, y:Float, z:Float):Void { updates++; point = [x, y, z]; }
  public function grabbedPoint():Array<Float> return point.copy();
  public function target():Array<Float> return point.copy();
  public function following():Bool return true;
  public function message():String return "Following";
  public function setFreeDetents(value:Bool):Void free = value;
  public function commit():Bool { commits++; return updates > 0; }
  public function cancel():Void cancels++;
}

class FrameAssemblyDragTests {
  static function check(value:Bool, message:String):Void if (!value) throw message;
  public static function run():Void {
    var probe = new DragProbe(), drag = new FrameAssemblyDrag(probe);
    for (i in 0...100) drag.update(i, 0, 0);
    check(probe.updates == 0, "pointer events defer solving until a frame");
    drag.setFreeDetents(true);
    check(drag.flush() && probe.updates == 1 && probe.grabbedPoint()[0] == 99 && probe.free,
      "one frame applies the latest target and modifiers");
    check(!drag.flush() && probe.updates == 1, "an idle frame does not solve again");
    drag.update(-3, 2, 1);
    check(drag.commit() && probe.updates == 2 && probe.grabbedPoint()[0] == -3 && probe.commits == 1,
      "release between frames flushes the final target before committing");
    drag.update(5, 0, 0); drag.flush(); drag.commit(); drag.cancel();
    check(probe.updates == 2 && probe.commits == 1 && probe.cancels == 0, "finished drags ignore further input");
    probe = new DragProbe(); drag = new FrameAssemblyDrag(probe);
    drag.update(2, 0, 0); drag.cancel();
    check(!drag.flush() && probe.updates == 0 && probe.cancels == 1, "cancel discards an unpainted target");
    probe = new DragProbe(); drag = new FrameAssemblyDrag(probe);
    drag.update(2, 0, 0); drag.update(Math.NaN, 0, 0); drag.flush();
    check(probe.updates == 1 && probe.grabbedPoint()[0] == 2, "invalid input cannot replace a valid pending target");
  }
}
