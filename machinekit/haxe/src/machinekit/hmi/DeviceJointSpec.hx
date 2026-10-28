package machinekit.hmi;

import materia.assembly.AssemblyDefinition.AssemblyJointLimits;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/**
 * A joint between two bodies of a device, about the device's own z axis through its origin: free (a wheel on its bearing) or
 * limited with detents (a rotary switch that clicks between its positions).
 */
class DeviceJointSpec {
  public final id:String;
  public final kind:AssemblyJointType;
  /** The id of the fixed body and of the one that turns. */
  public final parent:String;
  public final child:String;
  /** Travel limits and detents, in radians; null for a free joint. */
  public final limits:Null<AssemblyJointLimits>;
  public final defaultValue:Float;

  public function new(
    id:String,
    kind:AssemblyJointType,
    parent:String,
    child:String,
    ?limits:AssemblyJointLimits,
    defaultValue:Float = 0.0
  ) {
    this.id = id;
    this.kind = kind;
    this.parent = parent;
    this.child = child;
    this.limits = limits;
    this.defaultValue = defaultValue;
  }
}
