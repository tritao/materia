package app;

import cadkit.modeling.AssemblyDrag;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;

/** The document session's `SceneAssemblyDrag`: an `AssemblyDrag` previewed in the scene, committed as one edit. */
class ProjectAssemblyDrag implements SceneAssemblyDrag {
  final session:ProjectDocumentSession;
  final drag:AssemblyDrag;
  final before:AssemblyStateRecord;
  final metresPerUnit:Float;
  var currentTarget:Array<Float>;
  var lastMessage = "Following";
  var onTarget = true;
  var moved = false;
  var finished = false;

  public function new(session:ProjectDocumentSession, drag:AssemblyDrag, before:AssemblyStateRecord,
      metresPerUnit:Float) {
    this.session = session;
    this.drag = drag;
    this.before = before;
    this.metresPerUnit = metresPerUnit;
    currentTarget = grabbedPoint();
  }

  public function update(x:Float, y:Float, z:Float):Void {
    if (finished) return;
    if (!Math.isFinite(x) || !Math.isFinite(y) || !Math.isFinite(z)) return;
    currentTarget = [x, y, z];
    var result = drag.drag({x: x / metresPerUnit, y: y / metresPerUnit, z: z / metresPerUnit,
      qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0});
    onTarget = result.status == Following;
    lastMessage = result.message;
    moved = true;
    session.previewAssemblyDrag(drag);
  }

  public function grabbedPoint():Array<Float> {
    var pose = drag.grabbedPose();
    return [pose.x * metresPerUnit, pose.y * metresPerUnit, pose.z * metresPerUnit];
  }

  public function target():Array<Float> return currentTarget.copy();

  public function following():Bool return onTarget;

  public function message():String return lastMessage;

  public function commit():Bool {
    if (finished) return false;
    finished = true;
    return session.finishAssemblyDrag("Drag " + drag.occurrence, before, moved ? drag.commit() : null);
  }

  public function cancel():Void {
    if (finished) return;
    finished = true;
    session.finishAssemblyDrag("Drag " + drag.occurrence, before, null);
  }
}
