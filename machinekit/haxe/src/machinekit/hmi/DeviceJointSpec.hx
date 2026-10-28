package machinekit.hmi;

import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** A free joint between two bodies of a device, about the device's own z axis through its origin (a wheel on its bearing). */
class DeviceJointSpec {
  public final id:String;
  public final kind:AssemblyJointType;
  /** The id of the fixed body and of the one that turns. */
  public final parent:String;
  public final child:String;

  public function new(id:String, kind:AssemblyJointType, parent:String, child:String) {
    this.id = id;
    this.kind = kind;
    this.parent = parent;
    this.child = child;
  }
}
