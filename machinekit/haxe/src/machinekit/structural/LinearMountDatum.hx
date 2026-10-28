package machinekit.structural;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;

/** Host process, independent of the part's mounting-hole coordinates.
 * A welded pad is never disguised as a cast insert. References name the actual
 * anchorage/relief operation, not an assertion of measured acceptance.
 */
enum LinearMountHost {
  RetainedInsert(member:String, retentionReference:String);
  RelievedWeldedPad(member:String, reliefReference:String);
}

typedef ReplicationLayerInputs = {
  var compoundReference:String;
  var minimumMm:Null<Float>;
  var maximumMm:Null<Float>;
  /** Bounds on deviation from the nominated thickness, not peak-to-peak. */
  var fixtureErrorMm:Null<Float>;
  var cureDistortionMm:Null<Float>;
}

enum ReplicationAllowance {
  AwaitLayerInputs(missing:Array<String>);
  LayerInterval(minimumNominalMm:Float, maximumNominalMm:Float);
}

/** Shared mounted-element contract. It declares actual steel, compound and
 * adjustment members supplied by RM2 geometry. No dimensions or hole patterns
 * are inferred from a rack/rail bounding box, and no part is created here.
 */
class LinearMountDatum {
  public final host:LinearMountHost;
  public final plane:AssemblyFrame;
  public final sideBar:String;
  public final receiver:String;
  public final layer:String;
  public final pushes:Array<String>;
  public final locks:Array<String>;
  public final layerInputs:ReplicationLayerInputs;
  /** Cleats, carrier, washers and nuts owned by this mount, excluding the host. */
  public final otherMembers:Array<String>;

  public function new(
    host:LinearMountHost,
    plane:AssemblyFrame,
    sideBar:String,
    receiver:String,
    layer:String,
    pushes:Array<String>,
    locks:Array<String>,
    inputs:ReplicationLayerInputs,
    ?otherMembers:Array<String>
  ) {
    machinekit.design.DesignGeometry.validateFrame(plane);
    this.host = host;
    this.plane = materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.identity(), plane);
    this.sideBar = sideBar;
    this.receiver = receiver;
    this.layer = layer;
    this.pushes = pushes.copy();
    this.locks = locks.copy();
    this.otherMembers = otherMembers == null ? [] : otherMembers.copy();
    this.layerInputs = {
      compoundReference: inputs.compoundReference,
      minimumMm: inputs.minimumMm,
      maximumMm: inputs.maximumMm,
      fixtureErrorMm: inputs.fixtureErrorMm,
      cureDistortionMm: inputs.cureDistortionMm
    };
    if (StringTools.trim(sideBar) == ""
      || StringTools.trim(receiver) == "" || StringTools.trim(layer) == "" || pushes.length == 0
      || locks.length == 0) throw "Linear mount needs receiver steel, layer, side bar, push screws and locks";
    // Validate known inputs now; unknowns stay explicit.
    allowance(inputs);
  }

  public static function allowance(inputs:ReplicationLayerInputs):ReplicationAllowance {
    var missing:Array<String> = [];
    if (StringTools.trim(inputs.compoundReference) == "") missing.push("compoundReference");
    for (name in ["minimumMm", "maximumMm", "fixtureErrorMm", "cureDistortionMm"]) {
      var value:Null<Float> = Reflect.field(inputs, name);
      if (value == null) missing.push(name);
      else if (!Math.isFinite(value) || value < 0) throw "Invalid replication input: " + name;
    }
    if (inputs.minimumMm != null && inputs.minimumMm <= 0) throw "Replication must have a positive minimum thickness";
    if (inputs.minimumMm != null && inputs.maximumMm != null
      && inputs.maximumMm < inputs.minimumMm) throw "Replication maximum is below its minimum";
    if (missing.length > 0) return AwaitLayerInputs(missing);
    var fixture:Float = cast inputs.fixtureErrorMm, cure:Float = cast inputs.cureDistortionMm;
    var minimum:Float = cast inputs.minimumMm, maximum:Float = cast inputs.maximumMm;
    var error = fixture + cure;
    var low = minimum + error, high = maximum - error;
    if (high < low) throw "Replication error envelope cannot fit the compound thickness range";
    return LayerInterval(low, high);
  }

  public function hostMember():String return switch host {
    case RetainedInsert(member, _), RelievedWeldedPad(member, _):
      member;
  };

  /** Validate identities/material roles before physical contact/fastener checks.
	 * Does not claim relief, cure, alignment or load-bearing evidence exists.
	 */
  public function validateMembers(assembly:MachineAssembly):Void {
    var members:Map<String, MachineComponent> = [];
    for (m in assembly.components()) members.set(m.id, m.component);
    function require(id:String):MachineComponent {
      var c = members.get(id);
      if (c == null) throw "Missing linear-mount member: " + id;
      return c;
    }
    var hostPart = require(hostMember());
    switch host {
      case RetainedInsert(_, reference):
        if (!Std.isOfType(hostPart, CastInsert) || StringTools.trim(reference)
          == "") throw "Cast mount requires its retained source insert and anchorage reference";
      case RelievedWeldedPad(_, reference):
        if (Std.isOfType(hostPart, CastInsert)
          || StringTools.trim(reference) == "") throw "Welded mount requires a real pad and relief operation reference";
    }
    for (id in [hostMember(), sideBar, receiver]) {
      var c = require(id), material = materia.project.MaterialLibrary.require(c.materialId);
      if (material.physical.density < 7000 || c.materialSpec().toLowerCase().indexOf("steel")
        < 0) throw "Linear-mount clamp/load path must engage steel: " + id;
    }
    for (id in otherMembers) {
      if (id == null || StringTools.trim(id) == "" || id == hostMember()) throw "Invalid mount-owned member: " + id;
      require(id);
    }
    require(layer);
    if (layer == hostMember()
      || layer == receiver || layer == sideBar) throw "Replication compound cannot stand in for clamp steel";
    var seen:Map<String, Bool> = [];
    for (id in pushes.concat(locks)) {
      if (seen.exists(id)) throw "A linear-mount fastener is listed twice: " + id;
      seen.set(id, true);
      require(id);
    }
    for (id in pushes) if (!Std.isOfType(require(id),
      machinekit.standard.SocketSetScrew)) throw "Linear-mount adjuster must be a real push screw: " + id;
    for (id in locks) if (!Std.isOfType(require(id),
      machinekit.standard.ScrewFastener)) throw "Linear-mount lock must be a real service fastener: " + id;
  }

  /** Each actual member contributes mass once, even when the side bar also
	 * receives the element's bolts. The element and common host are excluded.
	 */
  public function addedMassKg(assembly:MachineAssembly):Float {
    validateMembers(assembly);
    var owned = [sideBar,
      receiver, layer].concat(pushes).concat(locks).concat(otherMembers), seen:Map<String, Bool> = [];
    var mass = 0.0;
    for (id in owned) if (id != hostMember() && !seen.exists(id)) {
      seen.set(id, true);
      for (m in assembly.components()) if (m.id == id) {
        var value = m.component.massProperties().mass;
        if (!Math.isFinite(value) || value <= 0) throw "Invalid linear-mount mass: " + id;
        mass += value;
      }
    }
    return mass;
  }
}
