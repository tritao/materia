package app;

import materia.assembly.AssemblyDefinition.AssemblyStateRecord;

/** A live coordinate preview which records only the final position on release. */
class ProjectJointEdit {
  final session:ProjectDocumentSession;
  final before:AssemblyStateRecord;
  final generation:Int;
  final joint:String;
  var finished = false;
  public function new(session:ProjectDocumentSession,joint:String,before:AssemblyStateRecord) {
    this.session=session;this.joint=joint;this.before=before;generation=session.generation;
  }
  public function update(value:Float):Void {
    if (finished || session.generation != generation) return;
    session.previewJointEdit(before,joint,value);
  }
  public function commit():Bool {
    if (finished) return false;
    finished=true;
    if (session.generation != generation) return false;
    return session.finishAssemblyDrag("Set joint "+joint,before,session.projectAssemblyState);
  }
  public function cancel():Void {
    if (finished) return;
    finished=true;
    if (session.generation == generation) session.finishAssemblyDrag("Set joint "+joint,before,null);
  }
}
