package machinekit.structural;

import machinekit.component.MachineComponent;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InsertManufacturing.InsertHoleRoute;

typedef MountHoleErrorAllocation = {
  var placementOffsetMm:materia.casting.CastingInput;
  var placementAngleRad:materia.casting.CastingInput;
  var assemblyAdjustmentMm:materia.casting.CastingInput;
  var cureBowMm:materia.casting.CastingInput;
  var machiningErrorMm:materia.casting.CastingInput;
  var pitchAccumulationMm:materia.casting.CastingInput;
  var requiredMarginMm:materia.casting.CastingInput;
}

/** Source authoring only. Mounting positions come from placed component
 * connectors; the one stored hole frame is the intersection with finished steel.
 */
class InsertMountPattern {
  public static function derive(
    source:CastInsert,
    component:MachineComponent,
    componentPath:String,
    componentFingerprint:String,
    componentToInsert:AssemblyFrame,
    connectorIds:Array<String>,
    thread:cadkit.threads.ThreadSpec,
    clearanceDiameter:Null<Float>,
    clearanceReference:Null<String>,
    permitted:Array<InsertHoleRoute>,
    allocation:MountHoleErrorAllocation
  ):CastInsert {
    var spec = source.insertSpec();
    var manufacturing = switch spec.origin {
      case Fabricated(plan):
        plan;
      case Catalogue(_):
        throw "A catalogue insert cannot acquire fabricated mounting holes";
    };
    var finished = switch materia.casting.InsertFinishing.assess(spec.body, spec.interfaces) {
      case Resolved(steel):
        steel;
      case Invalid(reason), Unsupported(reason):
        throw "Resolve mounting-hole steel: " + reason;
    };
    var stock = materia.casting.InsertFinishing.box(finished);
    if (stock == null) throw "Unsupported non-box mounting-hole steel: " + componentPath;
    var derived = MountHoleDerivation.derive(
      stock,
      source.insertMaterial().grade,
      component,
      componentPath,
      componentFingerprint,
      componentToInsert,
      connectorIds,
      thread,
      clearanceDiameter,
      clearanceReference,
      permitted,
      allocation
    );
    for (hole in derived.holes) {
      if (spec.holes.filter(existing -> existing.id
        == hole.id).length > 0) throw "Duplicate source mounting hole: " + componentPath + "/" + hole.id;
      spec.holes.push(hole);
    }
    for (requirement in derived.requirements) manufacturing.mountingRequirements.push(requirement);
    for (route in derived.routes) manufacturing.holeRoutes.push(route);
    return new CastInsert(source.values().copy().setToken("holes", InsertHoles.fromNeutral(spec.holes)).setToken(
      "origin",
      haxe.Json.stringify(materia.casting.InsertOrigin.InsertOriginCodec.record(spec.origin))
    ));
  }
}
