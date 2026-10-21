package materia.presentation;

import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionFlattener;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.presentation.AssemblyPresentation;

typedef PresentationCursor = {
  var view:String;
  var explode:Float;
  var procedure:Null<String>;
  /** Completed step count; the following step is active. */
  var step:Int;
  var progress:Float;
}
typedef PresentationFrame = {
  var poses:Map<String, AssemblyFrame>;
  var visibility:Map<String, VisibilityMode>;
  var highlight:Array<String>;
  var jointOverrides:Map<String, Float>;
  var sections:Array<SectionPlane>;
  var camera:ViewCamera;
  var activeStep:Null<ProcedureStep>;
  var evidence:Null<String>;
}

/** Stateless evaluation. Input poses and engineering records are never modified. */
class PresentationEvaluator {
  public static function reset(value:AssemblyPresentation):PresentationCursor {
    for (view in value.views) if (view.id == value.defaultView)
      return {view:view.id, explode:view.explode, procedure:null, step:0, progress:0};
    throw "Missing default presentation view";
  }
  public static function badge(evidence:MotionEvidence, fingerprint:String):String return switch evidence {
    case Illustrative: "Illustrative";
    case Checked(_, status, saved, _): saved != fingerprint ? "Stale" : status == Failed ? "Failed" : "Checked";
  }
  public static function evaluate(definition:AssemblyDefinition, solved:Map<String, AssemblyFrame>,
      value:AssemblyPresentation, cursor:PresentationCursor, fingerprint:String, playing:Bool = false):PresentationFrame {
    PresentationCodec.validate(definition, value);
    var flat = AssemblyDefinitionFlattener.flattenView(definition);
    if (cursor == null || !Math.isFinite(cursor.explode) || cursor.explode < 0 || cursor.explode > 1 ||
        !Math.isFinite(cursor.progress) || cursor.progress < 0 || cursor.progress > 1) throw "Invalid presentation cursor";
    function findView(id:String):PresentationView {
      for (candidate in value.views) if (candidate.id == id) return candidate;
      throw "Unknown cursor view";
    }
    var view = findView(cursor.view);
    var procedure:Null<Procedure> = null;
    if (cursor.procedure != null) {
      for (candidate in value.procedures) if (candidate.id == cursor.procedure) procedure = candidate;
      if (procedure == null) throw "Unknown cursor procedure";
      if (cursor.step < 0 || cursor.step > procedure.steps.length) throw "Invalid procedure step";
      // The most recently referenced view remains active until the next view reference.
      for (index in 0...Std.int(Math.min(cursor.step + 1, procedure.steps.length))) {
        var name = procedure.steps[index].view;
        if (name != null) view = findView(name);
      }
    }
    var result:PresentationFrame = {poses:new Map(), visibility:new Map(), highlight:[], jointOverrides:new Map(),
      sections:[], camera:view.camera, activeStep:null, evidence:null};
    for (part in flat.occurrences) {
      var pose = solved.get(part.id);
      if (pose == null) throw 'Missing solved presentation pose "${part.id}"';
      result.poses.set(part.id, AssemblyFrames.compose(AssemblyFrames.identity(), pose));
      result.visibility.set(part.id, Shown);
    }
    for (rule in view.visibility) for (id in PresentationCodec.resolve(flat, rule.select)) result.visibility.set(id, rule.mode);
    // Flattened solves omit subassembly nodes. Recover each scope from an internal root's
    // solved pose and its authored placement within that scope. Supplied scope poses take precedence.
    var scopePoses = new Map<String, AssemblyFrame>();
    for (id in solved.keys()) { var pose = solved.get(id); if (pose != null) scopePoses.set(id, pose); }
    var library = new Map<String, materia.assembly.AssemblyDefinition.AssemblySubdefinition>();
    if (definition.assemblies != null) for (entry in definition.assemblies) library.set(entry.id,entry);
    function scopes(members:Array<materia.assembly.AssemblyDefinition.AssemblyComponentOccurrence>,
        prefix:String, authored:AssemblyFrame):Void {
      for (member in members) if (member.assembly != null) {
        var id = prefix + member.id;
        var initial = AssemblyFrames.compose(authored,member.initialPose);
        if (!scopePoses.exists(id)) {
          for (leaf in flat.occurrences) if (StringTools.startsWith(leaf.id,id + "/")) {
            var internalChild = false;
            for (joint in flat.joints) if (joint.role == "tree" && joint.child == leaf.id &&
                StringTools.startsWith(joint.parent,id + "/")) internalChild = true;
            if (internalChild) continue;
            var pose = solved.get(leaf.id);
            if (pose != null) {
              var local = AssemblyFrames.compose(AssemblyFrames.inverse(initial),leaf.initialPose);
              scopePoses.set(id,AssemblyFrames.compose(pose,AssemblyFrames.inverse(local)));
              break;
            }
          }
        }
        var nested = library.get(member.assembly);
        if (nested == null) throw "Missing presentation assembly scope";
        scopes(nested.occurrences,id + "/",initial);
      }
    }
    scopes(definition.occurrences,"",AssemblyFrames.identity());
    // All offsets are translations. Rotate them by the solved parent frame, never the child's rotation.
    function move(select:PartSelector, direction:AssemblyVector, scale:Float):Void {
      var parent:Null<String> = null;
      for (joint in flat.joints) if (joint.child == select.occurrence && joint.role == "tree") parent = joint.parent;
      if (parent == null) {
        var slash = select.occurrence.lastIndexOf("/");
        if (slash >= 0) parent = select.occurrence.substr(0, slash);
      }
      var rotation = parent == null ? null : scopePoses.get(parent);
      var delta = rotation == null ? direction : AssemblyFrames.transformPoint(
        {x:0,y:0,z:0,qx:rotation.qx,qy:rotation.qy,qz:rotation.qz,qw:rotation.qw}, direction.x, direction.y, direction.z);
      for (id in PresentationCodec.resolve(flat, select)) {
        var pose = result.poses.get(id);
        if (pose == null) throw "Missing display pose";
        pose.x += delta.x * scale; pose.y += delta.y * scale; pose.z += delta.z * scale;
      }
    }
    if (!playing) {
      var stages = 1;
      for (offset in value.explode) stages = Std.int(Math.max(stages, offset.stage));
      for (offset in value.explode)
        move(offset.select, offset.direction, offset.distance * Math.max(0, Math.min(1, cursor.explode * stages - offset.stage + 1)));
      if (procedure != null) {
        var removed = new Map<String, PresentationMotion>();
        if (procedure.start == Disassembled) for (step in procedure.steps) switch step.action {
          case Install(select, motion) | Remove(select, motion): for (id in PresentationCodec.resolve(flat, select)) removed.set(id, motion);
          case _:
        }
        for (index in 0...cursor.step) switch procedure.steps[index].action {
          case Remove(select, motion): for (id in PresentationCodec.resolve(flat, select)) removed.set(id, motion);
          case Install(select, _): for (id in PresentationCodec.resolve(flat, select)) removed.remove(id);
          case SetJoints(coordinates): for (coordinate in coordinates) result.jointOverrides.set(coordinate.joint, coordinate.value);
          case Note:
        }
        for (id in removed.keys()) result.visibility.set(id, Hidden);
        if (cursor.step < procedure.steps.length) {
          var step = procedure.steps[cursor.step]; result.activeStep = step;
          for (select in step.highlight) for (id in PresentationCodec.resolve(flat, select))
            if (result.highlight.indexOf(id) < 0) result.highlight.push(id);
          switch step.action {
            case Remove(select, motion) | Install(select, motion):
              result.evidence = badge(motion.evidence, fingerprint);
              var t = switch step.action { case Install(_, _): 1 - cursor.progress; case _: cursor.progress; };
              var segment = t * (motion.path.length - 1);
              var index = Std.int(Math.min(motion.path.length - 2, Math.floor(segment)));
              var a = motion.path[index], b = motion.path[index + 1], blend = segment - index;
              move(select, {x:a.x + (b.x-a.x)*blend, y:a.y + (b.y-a.y)*blend, z:a.z + (b.z-a.z)*blend}, 1);
              for (id in PresentationCodec.resolve(flat, select)) result.visibility.set(id, Shown);
            case _:
          }
        }
      }
    }
    for (plane in view.sections) {
      var pose = plane.occurrence == null ? AssemblyFrames.identity() : result.poses.get(plane.occurrence);
      if (pose == null) throw "Missing section frame";
      var point = AssemblyFrames.transformPoint(pose, plane.point.x, plane.point.y, plane.point.z);
      var normal = AssemblyFrames.transformPoint({x:0,y:0,z:0,qx:pose.qx,qy:pose.qy,qz:pose.qz,qw:pose.qw},
        plane.normal.x, plane.normal.y, plane.normal.z);
      result.sections.push({occurrence:null, point:point, normal:normal});
    }
    return result;
  }
}
