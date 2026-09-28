package cadbridge;

import cadkit.modeling.AssemblyState;
import cadkit.InertiaTensor;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssembly.MachineAssemblyMassProperties;
import robotkit.material.LoadLimits;
import robotkit.material.Payload;
import robotkit.model.Link;

/** Maps a complete MachineKit mass rollup into RobotKit's SI conventions. */
class MachineAssemblyMassBridge {
  static function complete(assembly:MachineAssembly, state:Null<AssemblyState>, needInertia:Bool):MachineAssemblyMassProperties {
    if (assembly == null) throw "Machine assembly is required";
    var properties = assembly.massProperties(state);
    if (properties.unaccounted.length != 0)
      throw 'Machine assembly has unaccounted BOM mass: ${properties.unaccounted.join(", ")}';
    if (!Math.isFinite(properties.mass) || properties.mass <= 0)
      throw "Machine assembly requires positive mass";
    if (needInertia && properties.inertia == null)
      throw 'Machine assembly has missing inertia: ${properties.unaccountedInertia.join(", ")}';
    return properties;
  }

  /** Set a RobotKit link's mass, centre and centroidal inertia; mm and kg mm² become SI. */
  public static function applyToLink(assembly:MachineAssembly, link:Link, ?state:AssemblyState):Void {
    if (link == null) throw "Robot link is required";
    var properties = complete(assembly, state, true);
    var tensor:InertiaTensor = cast properties.inertia;
    link.mass = properties.mass;
    link.centerOfMass = [properties.centreOfMass.x * 1e-3,
      properties.centreOfMass.y * 1e-3, properties.centreOfMass.z * 1e-3];
    link.inertiaTensor = [for (value in [tensor.xx, tensor.xy, tensor.xz,
      tensor.xy, tensor.yy, tensor.yz, tensor.xz, tensor.yz, tensor.zz]) value * 1e-6];
  }

  /** Evaluate the RobotKit load envelope using the assembly's mass and centre of mass. */
  public static function payloadViolation(assembly:MachineAssembly, limits:LoadLimits,
      liftHeightMeters:Float, lengthMeters:Float, widthMeters:Float, heightMeters:Float,
      ?state:AssemblyState):Null<String> {
    if (limits == null) throw "Load limits are required";
    var properties = complete(assembly, state, false);
    var centre = properties.centreOfMass;
    var payload = new Payload(properties.mass, lengthMeters, widthMeters, heightMeters,
      centre.x * 1e-3, centre.y * 1e-3, centre.z * 1e-3);
    return limits.violation(payload, liftHeightMeters);
  }
}
