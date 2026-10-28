package materia.tooling.casting;

import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;

enum abstract MoldAssemblyStepKind(String) to String {
  var Base = "base";
  var Panel = "panel";
  var Tooling = "tooling";
  var Steel = "steel";
  var Fastening = "fastening";
  var Protection = "protection";
  var Preparation = "preparation";
  var Ready = "ready";
}

enum abstract MoldAssemblyEvidenceKind(String) to String {
  var CheckedSweep = "checked-assembly-sweep";
  var ManualOperation = "manual-operation";
}

typedef MoldAssemblyEvidence = {
  var kind:MoldAssemblyEvidenceKind;
  var checkedMember:Null<String>;
  var blockers:Array<String>;
  var travelMm:Float;
  var toleranceMm3:Float;
}

typedef MoldAssemblyStep = {
  var id:String;
  var kind:MoldAssemblyStepKind;
  var title:String;
  var instruction:String;
  var memberIds:Array<String>;
  var highlightIds:Array<String>;
  var dependsOn:Array<String>;
  /** World displacement from the fitted position to the approach position. Manual steps have no invented path. */
  var clearance:AssemblyVector;
  var evidence:MoldAssemblyEvidence;
}

typedef MoldAssemblyPlan = {
  var steps:Array<MoldAssemblyStep>;
}



/** Neutral manufacturing plan. CAD supplies checked sweeps; presentations only adapt this explicit record. */
class MoldAssemblyPlanner {
  public static function build(
    p:CastingMoldPlan,
    sweep:ToolingSweepReport,
    motions:Array<{id:String, x:Float, y:Float, z:Float}>,
    groups:Array<{id:String, members:Array<String>}>
  ):MoldAssemblyPlan {
    var steps:Array<MoldAssemblyStep> = [], owner = new Map<String, MoldAssemblyStep>();
    var groupMap = new Map<String, Array<String>>();
    for (group in groups) {
      if (groupMap.exists(group.id)) throw "Duplicate assembly group: " + group.id;
      groupMap.set(group.id, group.members);
    }
    function leaves(id:String, visiting:Array<String>):Array<String> {
      if (visiting.indexOf(id) >= 0) throw "Cyclic assembly group: " + id;
      var members = groupMap.get(id);
      if (members == null) return [id];
      var result:Array<String> = [];
      for (member in members) result = result.concat(leaves(member, visiting.concat([id])));
      return result;
    }
    function add(step:MoldAssemblyStep):Void {
      for (member in step.memberIds) {
        if (owner.exists(member)) throw "Duplicate assembly operation: " + member;
        owner.set(member, step);
      }
      steps.push(step);
    }
    function manual(id:String,
      kind:MoldAssemblyStepKind, members:Array<String>, title:String, instruction:String):MoldAssemblyStep {
      var step:MoldAssemblyStep = {
        id: id,
        kind: kind,
        title: title,
        instruction: instruction,
        memberIds: members,
        highlightIds: members.copy(),
        dependsOn: [],
        clearance: {x: 0, y: 0, z: 0},
        evidence: {kind: ManualOperation, checkedMember: null, blockers: [], travelMm: 0, toleranceMm3: 0}
      };
      add(step);
      return step;
    }
    function operation(member:String):MoldAssemblyStep {
      var step = owner.get(member);
      if (step == null) throw "Missing explicit assembly operation: " + member;
      return step;
    }
    function requires(step:MoldAssemblyStep, dependency:MoldAssemblyStep):Void {
      if (step != dependency && step.dependsOn.indexOf(dependency.id) < 0) step.dependsOn.push(dependency.id);
    }
    function after(member:String, support:String):Void requires(operation(member), operation(support));
    var byMotion = new Map<String, MoldAssemblyStep>();
    for (id in sweep.assemblyOrder) {
      var matches = motions.filter(m -> m.id == id);
      if (matches.length != 1) throw "Missing or duplicate checked assembly motion: " + id;
      var motion = matches[0], members = leaves(id, []);
      if (members.length == 0) throw "Empty assembly movement: " + id;
      var step:MoldAssemblyStep = {
        id: "install-" + id,
        kind: Tooling,
        title: "Install " + id,
        instruction: "Install the highlighted tooling along its checked assembly approach.",
        memberIds: members,
        highlightIds: members.copy(),
        dependsOn: [],
        clearance: {x: motion.x * sweep.travelMm, y: motion.y * sweep.travelMm, z: motion.z * sweep.travelMm},
        evidence: {
          kind: CheckedSweep,
          checkedMember: id,
          blockers: [for (edge in sweep.blocking) if (edge.movingId == id) edge.blockerId],
          travelMm: sweep.travelMm,
          toleranceMm3: sweep.contactVolumeToleranceMm3
        }
      };
      add(step);
      byMotion.set(id, step);
    }
    for (edge in sweep.blocking) {
      var before = byMotion.get(edge.movingId), later = byMotion.get(edge.blockerId);
      if (before == null || later == null) throw "Unknown checked assembly dependency";
      requires(later, before);
    }
    var base = p.panels.members[0].panel.id;
    for (panel in p.panels.members.concat(p.supportPanels)) operation(panel.panel.id).kind = Panel;
    operation(base).kind = Base;
    var prepare = manual(
      "prepare-base",
      Preparation,
      [],
      "Prepare the mould base",
      "Prepare the locating tooling and apply the specified release system. Keep insert bonding faces uncoated."
    );
    requires(prepare, operation(base));
    for (step in steps) if (step != prepare && step != operation(base)) requires(step, prepare);
    // The CAD producer expands these declared fasteners using the same occurrence contract.
    for (hardware in p.hardware) if (!owner.exists(hardware.id)) {
      var protection = hardware.kind == BoreMask || hardware.kind == BorePlug;
      manual(
        "fit-" + hardware.id,
        protection ? Protection : Fastening,
        ToolingHardwareMembers.of(hardware.id, hardware.kind),
        (protection ? "Protect bore with " : "Fit and secure ") + hardware.id,
        protection ? "Fit the declared bore protection using the tooling instructions. This is a manual operation."
        : "Fit and secure this fastener set after its joined members are located. This is a manual operation."
      );
    }
    var attachments = new Map<String, Array<String>>();
    function attach(id:String, parts:Array<String>):Void {
      var known = attachments.get(id);
      if (known == null) {
        known = [];
        attachments.set(id, known);
      }
      for (part in parts) if (known.indexOf(part) < 0) known.push(part);
    }
    for (hardware in p.hardware) if (hardware.attachmentIds != null) attach(hardware.id, hardware.attachmentIds);
    for (fixture in p.fixtures) {
      var insert = DatumFixtures.memberId(p, fixture.insertId), steel = operation(insert);
      var support = switch fixture.support {
        case OnPanel(id):
          id;
        case OnDatum(volume, _, _):
          volume.id;
      };
      steel.kind = Steel;
      steel.title = "Place steel insert " + fixture.insertId;
      steel.instruction =
        "Locate the steel insert on its highlighted support and align its receiving holes before securing it.";
      steel.highlightIds.push(support);
      after(insert, support);
      switch fixture.support {
        case OnDatum(volume, _, location):
          for (pin in [location.primaryId, location.secondaryId]) {
            attach(pin, [base, volume.id]);
            after(insert, pin);
          }
        case OnPanel(_):
      }
      for (fixing in fixture.fixings) {
        attach(fixing.hardwareId, [insert, fixing.panelId]);
        var plug = DatumFixtures.plugId(p, fixture.insertId, fixing.sourceHoleId);
        if (owner.exists(plug)) attach(plug, [insert]);
      }
      for (mask in fixture.boreMasks) {
        attach(mask.hardwareId, [insert]);
        if (mask.farEndId != null) attach(mask.farEndId, [insert]);
      }
    }
    for (hardware in p.hardware) {
      var joined = attachments.get(hardware.id);
      if (joined == null || joined.length == 0) throw "Missing hardware attachment relationship: " + hardware.id;
      var step = operation(hardware.id);
      for (part in joined) {
        after(hardware.id, part);
        if (step.highlightIds.indexOf(part) < 0) step.highlightIds.push(part);
      }
    }
    for (core in p.cores) {
      operation(core.id);
      for (panel in core.support.panelIds) after(core.id, panel);
      if (core.support.foot != null) {
        var foot = materia.tooling.ToolingIds.derive("", core.id, "-foot");
        if (!owner.exists(foot)) manual(
          "bond-" + foot,
          Tooling,
          [foot],
          "Bond the locating foot",
          "Bond and locate the declared EPS foot before installing its core. This is a manual operation."
        );
        after(foot, base);
        after(core.id, foot);
      }
    }
    for (former in p.formers) operation(former.volume.id);
    for (insert in p.document.input.inserts) operation(DatumFixtures.memberId(p, insert.id));
    for (hardware in p.hardware) for (member in ToolingHardwareMembers.of(hardware.id,
      hardware.kind)) operation(member);
    // Every operation starts from the supported base, rather than a floating initial scene.
    for (step in steps) if (step != prepare && step != operation(base)) requires(step, prepare);
    var ready = manual(
      "ready-to-pour",
      Ready,
      [],
      "Confirm tooling readiness",
      "Verify insert locations, fixings, bore protection and completed box closure before pouring."
    );
    for (step in steps) if (step != ready) requires(ready, step);
    return {steps: order(steps)};
  }

  /** Deterministic dependency order, preferring accessible box construction and promptly securing located parts. */
  public static function order(steps:Array<MoldAssemblyStep>):Array<MoldAssemblyStep> {
    var byId = new Map<String, MoldAssemblyStep>();
    for (step in steps) {
      if (byId.exists(step.id)) throw "Duplicate assembly step: " + step.id;
      byId.set(step.id, step);
    }
    for (step in steps) for (id in step.dependsOn) if (!byId.exists(id)) throw "Unknown assembly dependency: " + id;
    function rank(kind:MoldAssemblyStepKind):Int return switch kind {
      case Base, Preparation:
        0;
      case Fastening, Protection:
        1;
      case Panel:
        2;
      case Tooling:
        3;
      case Steel:
        4;
      case Ready:
        5;
      default:
        throw "Unknown assembly step kind";
    };
    var pending = steps.copy();
    pending.sort((a, b) -> {var difference = rank(a.kind) - rank(b.kind);
    return difference == 0 ? Reflect.compare(a.id, b.id) : difference;
    });
    var done = new Map<String, Bool>(), result:Array<MoldAssemblyStep> = [];
    while (result.length < steps.length) {
      var next:Null<MoldAssemblyStep> = null;
      for (step in pending) if (!done.exists(step.id)) {
        var ready = true;
        for (id in step.dependsOn) if (!done.exists(id)) ready = false;
        if (ready) {
          next = step;
          break;
        }
      }
      if (next == null) throw "Cyclic assembly dependencies: "
        + [for (step in pending) if (!done.exists(step.id)) step.id].join(", ");
      done.set(next.id, true);
      result.push(next);
    }
    return result;
  }
}
