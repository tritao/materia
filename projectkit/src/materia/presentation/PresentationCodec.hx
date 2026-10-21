package materia.presentation;

import haxeon.wire.JsonWire;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.presentation.AssemblyPresentation;

/** Strict boundary for both generated and authored presentation records. */
class PresentationCodec {
  public static inline var VERSION = 1;
  public static function encode(definition:AssemblyDefinition, value:AssemblyPresentation):String {
    validate(definition, value);
    return JsonWire.encode(value);
  }
  public static function decode(definition:AssemblyDefinition, text:String):AssemblyPresentation {
    var value:AssemblyPresentation = JsonWire.decode(text);
    validate(definition, value);
    return value;
  }
  public static function resolve(definition:AssemblyDefinition, select:PartSelector):Array<String> {
    if (select == null || select.occurrence == null || select.occurrence.length == 0)
      throw "Presentation selector needs an occurrence";
    var result = [];
    for (part in definition.occurrences)
      if (part.id == select.occurrence || select.subtree && StringTools.startsWith(part.id, select.occurrence + "/"))
        result.push(part.id);
    if (result.length == 0) throw 'Unknown presentation occurrence "${select.occurrence}"';
    return result;
  }
  static function text(value:String):Void {
    if (value == null || StringTools.trim(value).length == 0) throw "Presentation text must not be empty";
  }
  static function source(value:Null<PresentationSource>):Void {
    if (value != null) switch value { case Generated(producer): text(producer); case Authored: }
  }
  static function finite(value:Float):Void {
    if (!Math.isFinite(value)) throw "Presentation values must be finite";
  }
  static function vector(value:materia.assembly.AssemblyDefinition.AssemblyVector, nonzero:Bool = false):Void {
    if (value == null) throw "Missing presentation vector";
    finite(value.x); finite(value.y); finite(value.z);
    if (nonzero && value.x * value.x + value.y * value.y + value.z * value.z < 1e-18)
      throw "Presentation direction must be nonzero";
  }
  public static function validate(definition:AssemblyDefinition, value:AssemblyPresentation):Void {
    AssemblyDefinitionCodec.validate(definition);
    var flat = AssemblyDefinitionFlattener.flattenView(definition);
    if (value == null || value.schemaVersion != VERSION) throw "Unsupported presentation schemaVersion";
    if (value.definition != definition.id) throw "Presentation belongs to another assembly";
    if (value.views == null || value.explode == null || value.procedures == null) throw "Missing presentation arrays";
    var views = new Map<String, Bool>();
    function selectors(items:Array<PartSelector>):Void {
      if (items == null) throw "Missing presentation selectors";
      for (item in items) resolve(flat, item);
    }
    for (view in value.views) {
      text(view.id); text(view.label); source(view.source);
      if (views.exists(view.id)) throw 'Duplicate presentation view "${view.id}"';
      views.set(view.id, true);
      finite(view.explode);
      if (view.explode < 0 || view.explode > 1) throw "View explode must be in 0..1";
      if (view.camera == null || view.visibility == null || view.sections == null) throw "Incomplete presentation view";
      switch view.camera {
        case Frame(select, yaw, pitch, margin):
          selectors(select); if (select.length == 0) throw "Frame camera needs selectors";
          finite(yaw); finite(pitch); finite(margin); if (margin <= 0) throw "Camera margin must be positive";
        case Anchored(anchor, offset, span, yaw, pitch):
          resolve(flat, anchor); vector(offset); finite(span); finite(yaw); finite(pitch);
          if (span <= 0) throw "Camera span must be positive";
      }
      for (rule in view.visibility) {
        resolve(flat, rule.select); if (rule.mode == null) throw "Missing visibility mode";
      }
      for (plane in view.sections) {
        if (plane.occurrence != null) resolve(flat, {occurrence:plane.occurrence, subtree:false});
        vector(plane.point); vector(plane.normal, true);
      }
    }
    if (!views.exists(value.defaultView)) throw "Presentation defaultView does not exist";
    for (offset in value.explode) {
      source(offset.source);
      resolve(flat, offset.select); vector(offset.direction, true); finite(offset.distance);
      if (offset.distance < 0 || offset.stage < 1) throw "Invalid presentation explode distance or stage";
    }
    var joints = new Map<String, materia.assembly.AssemblyDefinition.KinematicJoint>();
    for (joint in flat.joints) joints.set(joint.id, joint);
    var procedures = new Map<String, Bool>();
    for (procedure in value.procedures) {
      text(procedure.id); text(procedure.label); source(procedure.source);
      if (procedures.exists(procedure.id)) throw "Duplicate procedure id";
      procedures.set(procedure.id, true);
      if (procedure.kind == null || procedure.start == null || procedure.steps == null) throw "Incomplete procedure";
      var steps = new Map<String, Bool>();
      for (step in procedure.steps) {
        text(step.id); text(step.title); text(step.instruction);
        if (steps.exists(step.id)) throw "Duplicate procedure step id";
        steps.set(step.id, true);
        if (step.view != null && !views.exists(step.view)) throw "Unknown procedure view";
        selectors(step.highlight);
        if (step.action == null) throw "Missing procedure action";
        var moving=StepMotions.members(step.action);
        switch step.action {case RemoveGroup(items)|InstallGroup(items):if(items==null||items.length==0)throw "Movement group must not be empty";case _:}
        var moved=new Map<String,Bool>();
        for(member in moving) {
            if(member==null)throw "Missing group member";
            var select=member.select,motion=member.motion;
            for(id in resolve(flat,select)) {
              if(moved.exists(id))throw "Movement group contains overlapping selectors";
              moved.set(id,true);
            }
            if (motion == null || motion.path == null || motion.path.length < 2 || motion.evidence == null)
              throw "Movement needs a path and evidence";
            for (point in motion.path) vector(point);
            switch motion.evidence {
              case Illustrative:
              case Checked(checker, status, fingerprint, report):
                text(checker); text(fingerprint); text(report);
                if (status == null) throw "Missing movement check status";
            }
            if (procedure.source == Authored && motion.evidence != Illustrative)
              throw "Authored movement evidence must be Illustrative";
        }
        switch step.action {
          case SetJoints(coordinates):
            if (coordinates == null) throw "Missing joint coordinates";
            var seen = new Map<String, Bool>();
            for (coordinate in coordinates) {
              if (!joints.exists(coordinate.joint) || seen.exists(coordinate.joint)) throw "Unknown or duplicate procedure joint";
              finite(coordinate.value);
              var joint = joints.get(coordinate.joint);
              if (joint == null || joint.role != "tree" || joint.type == "fixed") throw "Procedure joint is not a tree coordinate";
              if (joint.limits.lower != null && coordinate.value < joint.limits.lower ||
                  joint.limits.upper != null && coordinate.value > joint.limits.upper) throw "Procedure coordinate exceeds joint limits";
              seen.set(coordinate.joint, true);
            }
          case _:
        }
      }
    }
  }
}
