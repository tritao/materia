package materia.presentation;

import materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;
import materia.presentation.AssemblyPresentation;

/** Producer helper. The viewer always consumes explicit steps in the stored order. */
class Procedures {
  /** Initial coordinates are required for every SetJoints key so reversal never guesses engineering state. */
  public static function reverse(procedure:Procedure, id:String, label:String,
      initial:Array<AssemblyJointCoordinate>):Procedure {
    var coordinates = new Map<String, Float>();
    for (coordinate in initial) coordinates.set(coordinate.joint, coordinate.value);
    var reversed:Array<ProcedureStep> = [];
    for (step in procedure.steps) {
      var action:StepAction = switch step.action {
        case Remove(select, motion): Install(select, motion);
        case Install(select, motion): Remove(select, motion);
        case Note: Note;
        case SetJoints(keys):
          var previous:Array<AssemblyJointCoordinate> = [];
          for (key in keys) {
            var value = coordinates.get(key.joint);
            if (value == null) throw 'Reversing procedure needs initial coordinate "${key.joint}"';
            previous.push({joint:key.joint,value:value});
            coordinates.set(key.joint,key.value);
          }
          SetJoints(previous);
      }
      var title = step.title, instruction = step.instruction;
      switch action {
        case Install(select, _):
          title = "Install " + select.occurrence;
          instruction = "Follow the recorded withdrawal path in reverse to install " + select.occurrence + ".";
        case Remove(select, _):
          title = "Remove " + select.occurrence;
          instruction = "Follow the recorded withdrawal path to remove " + select.occurrence + ".";
        case SetJoints(_): title = "Restore joint coordinates"; instruction = "Restore the recorded joint coordinates.";
        case Note:
      }
      reversed.unshift({id:step.id,title:title,instruction:instruction,view:step.view,
        highlight:step.highlight,action:action});
    }
    // Clone through the wire boundary to prevent edits to the derived procedure altering the original.
    var result:Procedure = {id:id,label:label,kind:switch procedure.kind {
      case Assembly: Disassembly; case Disassembly: Assembly; case Service: Service;
    },start:procedure.start == Assembled ? Disassembled : Assembled,steps:reversed,source:procedure.source};
    return haxeon.wire.JsonWire.decode(haxeon.wire.JsonWire.encode(result));
  }
}
