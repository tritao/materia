package app;

import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyJointRole;
import materia.assembly.AssemblyDefinitionCodec;

typedef MachineControlAxis = {
  var id:String;
  var value:Float;
  var minimum:Null<Float>;
  var maximum:Null<Float>;
  var initial:Float;
  var unit:String;
  var step:Float;
  var factor:Float;
  var follows:Null<String>;
}

/** Joint controls use the document's validated kinematics and undo, in human-facing units. */
class MachineControlModel {
  public static function axes(session:ProjectDocumentSession):Array<MachineControlAxis> {
    var definition = session.projectAssemblyDefinition, state = session.projectAssemblyState;
    if (definition == null || state == null) return [];
    var positions:Map<String,Float> = [];
    for (coordinate in state.jointCoordinates) positions.set(coordinate.joint,coordinate.value);
    var coupled:Map<String,String> = [];
    if (definition.couplings != null) for (coupling in definition.couplings)
      coupled.set(coupling.target, coupling.source);
    var result:Array<MachineControlAxis> = [];
    for (joint in definition.joints) if (joint.role == Tree && AssemblyDefinitionCodec.hasCoordinate(joint.type)) {
      var linear = joint.type == Prismatic;
      var factor = linear ? session.assemblyUnitMetres()*1000 : 180/Math.PI;
      var value = positions.get(joint.id);
      result.push({id:joint.id,value:(value == null ? joint.defaultValue : value)*factor,
        minimum:joint.limits.lower == null ? null : joint.limits.lower*factor,
        maximum:joint.limits.upper == null ? null : joint.limits.upper*factor,
        initial:joint.defaultValue*factor,unit:linear ? "mm" : "deg",step:linear ? 0.1 : 1.0,factor:factor,
        follows:coupled.exists(joint.id) ? coupled.get(joint.id) : session.assemblyJointIsDependent(joint.id) ? "closure" : null});
    }
    return result;
  }

  public static function parsePosition(text:String):Float {
    var clean = StringTools.trim(text);
    if (!~/^[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$/.match(clean))
      throw "Enter a numeric axis position";
    var value = Std.parseFloat(clean);
    if (!Math.isFinite(value)) throw "Enter a finite axis position";
    return value;
  }

  public static function set(session:ProjectDocumentSession,id:String,value:Float,editable:Bool):Bool {
    if (!editable || session.scriptOwnership != null) throw "Machine controls are currently locked";
    if (!Math.isFinite(value)) throw "Enter a finite axis position";
    for (axis in axes(session)) if (axis.id == id) {
      if (axis.follows != null) throw 'Axis "$id" follows ${axis.follows}';
      if ((axis.minimum != null && value < axis.minimum-1e-8) || (axis.maximum != null && value > axis.maximum+1e-8))
        throw 'Position is outside the travel limits for $id';
      // Clamp only rounding error introduced by conversion at the exact stops.
      var native = value/axis.factor;
      var definition = session.projectAssemblyDefinition;
      if (definition != null) for (joint in definition.joints) if (joint.id == id) {
        if (joint.limits.lower != null) native = Math.max(native,joint.limits.lower);
        if (joint.limits.upper != null) native = Math.min(native,joint.limits.upper);
      }
      return session.setAssemblyJointCoordinate(id,native);
    }
    throw 'Unknown machine axis "$id"';
  }
}
