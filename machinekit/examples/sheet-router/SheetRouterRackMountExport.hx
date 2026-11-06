import machinekit.structural.fem.AssemblyFemExport;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentValue.*;
import machinekit.transmission.*;
import machinekit.structural.*;
import cadkit.modeling.AssemblyState;
import sys.FileSystem;
import sys.io.File;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssembly.MachineAssemblyComponent;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** The frozen structural model excludes independently floor-supported services.
 * Keep their declared masses in a separate ledger; do not invent their inertia. */
private class RouterFrameMassView extends MachineAssembly {
  final source:SheetRouter;
  final excluded:Array<String>;
  public function new(source:SheetRouter, excluded:Array<String>) {
    super();
    this.source = source;
    this.excluded = excluded;
  }
  override public function components():Array<MachineAssemblyComponent
    > return [for (row in source.components()) if (excluded.indexOf(row.id) < 0) row];
  override public function mateParents():Map<String, String> return source.mateParents();
  override public function solvedPoses(?state:AssemblyState):Map<String,
    AssemblyFrame> return source.solvedPoses(state);
}

/** Geometry/material ownership for the local mount solve. No STEP/mesh packet is
 * emitted here; native mass, inertia, poses and source drilling remain auditable. */
class SheetRouterRackMountExport {
  static function inputParameters(part:machinekit.component.MachineComponent):Dynamic {
    // Some machined hardware has no reconstructible catalogue recipe. Its
    // posed mass/bounds remain mandatory; explicit stock/bores are added below.
    try return parameters(part.values()) catch (error:Dynamic) {
      if (Std.string(error) != 'Component "' + part.designation + '" has no recipe values') throw error;
      return {recipeAvailable: false, designation: part.designation};
    }
  }
  static function parameters(values:ComponentValues):Dynamic {
    var result:Dynamic = {};
    for (name in values.names()) {
      var entry = values.get(name);
      if (entry == null) throw "Missing component parameter: " + name;
      var value:Dynamic = switch entry {
        case Number(v):
          v;
        case Integer(v):
          v;
        case Boolean(v):
          v;
        case Token(v):
          v;
        case Unset:
          null;
      };
      if (name == "holes" || name == "mountingRequirements") value = haxe.Json.parse(cast value);
      Reflect.setField(result, name, value);
    }
    return result;
  }
  public static function write(machine:SheetRouter, folder:String):Void {
    if (!FileSystem.exists(folder)) FileSystem.createDirectory(folder);
    var state = new AssemblyState(machine.definition());
    state.setJoint("x", 0);
    state.setJoint("y", machine.routerSpec.workY / 2);
    state.setJoint("z", machine.routerSpec.travelZ);
    state.forwardKinematics();
    var floorSeeds = [
      "vacuumBlower",
      "cabinet",
      "vfd",
      "brakeResistor",
      "mainBreaker",
      "mainContactor",
      "safetyRelay",
      "emergencyStop"
    ], parents = machine.mateParents(), floorIds:Array<String> = [], floorParts:Array<Dynamic> = [], floorMass = 0.0;
    for (member in machine.components()) {
      var cursor:Null<String> = member.id, seen:Map<String, Bool> = [], independent = false;
      while (cursor != null && !seen.exists(cursor)) {
        seen.set(cursor, true);
        if (floorSeeds.indexOf(cursor) >= 0) {
          independent = true;
          break;
        }
        cursor = parents.get(cursor);
      }
      if (!independent) continue;
      var mass = member.component.massProperties(),
        pose = state.worldPose(member.id), c = mass.centreOfMass, centre = F.transformPoint(
          pose,
          c.x,
          c.y,
          c.z
        );
      floorIds.push(member.id);
      floorMass += mass.mass;
      floorParts.push({
        id: member.id,
        parentId: parents.get(member.id),
        massKg: mass.mass,
        centreOfMassMm: centre,
        poseMm: pose,
        materialId: member.component.materialId,
        centroidalInertiaKgMm2World: mass.inertia == null ? null : mass.inertia.rotated(
          pose.qx,
          pose.qy,
          pose.qz,
          pose.qw
        ),
        inertiaSource : mass.inertia == null ? "not declared; excluded from frame model" : "component tensor",
        representation : "independent-floor-mass",
        massSource : Std.string(mass.source)
      });
    }
    var bodies = AssemblyFemExport.bodies(new RouterFrameMassView(machine,
      floorIds), state, folder, _ -> false), inputs:Array<Dynamic> = [];
    for (binding in machine.cableChainBindings) {
      var rows = [for (row in bodies.parts) if (row.id == binding.member) row];
      if (rows.length != 1) throw "Missing chain mass owner";
      var raw = binding.chain.geometryAt(state.joint(binding.axis) + binding.positionOffset);
      var posed = machinekit.assembly.PosedParts.moved(raw, state.worldPose(binding.member));
      raw.close();
      var physical = posed.massProperties(), row = rows[0];
      row.centreOfMassMm = {x: physical.centerOfMass.x, y: physical.centerOfMass.y, z: physical.centerOfMass.z};
      row.centroidalInertiaKgMm2World = physical.inertiaAtDensity(row.massKg / physical.volume);
      row.boundsMm = machinekit.assembly.PosedParts.boxOf(posed);
      row.volumeMm3 = physical.volume;
      row.inertiaSource = "routed chain at exported pose, scaled to owned component mass";
      posed.close();
    }
    for (member in machine.components()) {
      var part = member.component;
      if (!StringTools.startsWith(member.id,
        "rackMount") && !StringTools.startsWith(member.id, "pinion") && !Std.isOfType(
          part,
          RectTubeMember
        ) && !Std.isOfType(
          part,
          MountDrilledPlate
        ) && !Std.isOfType(
          part,
          GearRackRun
        )) continue;
      var row:Dynamic = {
        id: member.id,
        designation: part.designation,
        material: part.materialSpec(),
        geometryKey: part.geometryKey(Preview),
        parameters: inputParameters(part),
        pose: state.worldPose(member.id)
      };
      if (Std.isOfType(part, PinionPivotAnchor)) {
        var anchor:PinionPivotAnchor = cast part;
        Reflect.setField(
          row,
          "pivotAnchor",
          {hostZMm: anchor.hostZ, topMount: anchor.topMount, stockBoxBounds: anchor.stockRegions(), bores: anchor.bores}
        );
      }
      if (Std.isOfType(part, LinearMountCleat)) {
        var cleat:LinearMountCleat = cast part;
        Reflect.setField(
          row,
          "cleat",
          {
            stockBoxBounds: cleat.stockRegions(),
            cutBores: cleat.cutBores(),
            connectionEnvelopeN: {tangential: 1000, radial: 400, axial: 400},
            connection: cleat.connection(1000, 400, 400)
          }
        );
      }
      if (Std.isOfType(part, RadialPinionCarrier)) {
        var carrier:RadialPinionCarrier = cast part;
        Reflect.setField(row, "carrierMotorFlange", machinekit.motion.AxisMotor.of(carrier.motor).flange);
        Reflect.setField(
          row,
          "carrierStock",
          {stockBoxBounds: carrier.stockRegions(), bossRadiusMm: 14, boreRadiusMm: 7, thicknessMm: 15}
        );
      }
      inputs.push(row);
    }
    var result = {status: "local-model input; no SR5/SR6 acceptance", bedMm: machine.routerSpec.workY, poseMm: {
      x: 0.0,
      y: machine.routerSpec.workY / 2,
      z: machine.routerSpec.travelZ
    }, stockBoundsOrder: [
      "x0",
      "x1",
      "y0",
      "y1",
      "z0",
      "z1"
    ], parts: bodies.parts, totalMassKg: bodies.totalMassKg + floorMass,
      structuralBodyMassKg: bodies.totalMassKg, independentFloorMassKg: floorMass,
      independentFloorParts: floorParts, materials: bodies.materials, mountInputs: inputs, moving: [for (axis in [
        "x",
        "y",
        "z"
      ]) {
      axis:axis, mass:SheetRouterSizing.moving(machine, axis)
    }
    ], joints:machine.definition().joints, assumptions:[
      "Declared rack/motor masses retain their catalogue or assumed distribution basis",
      "Moving cable-chain mass is the whole-chain upper bound; routed inertia is evaluated at the exported pose",
      "Local solid sections, drilling, fastener grips and pivot geometry are inputs, not stiffness ratings",
      "Independent floor services follow the frozen model's exclusion; their declared masses remain in a separate ledger and missing inertia is not fabricated",
      "Preload/contact, bearing clearances, compound creep and actual material properties require explicit bounds",
      "Replay existing J6 histories with nominal stop only; final executed reduced-stop SR5/SR6 campaign remains held"
    ]};
    File.saveContent(folder + "/mount-input.json", haxe.Json.stringify(result, null, "  ") + "\n");
    Sys.println('Built rack mount inputs ' + machine.routerSpec.workY
      + ': ' + bodies.totalMassKg + ' kg carried-body ledger; ' + floorMass + ' kg independent floor equipment');
    Sys.stdout().flush();
  }
  public static function main():Void {
    var bed = Sys.args().length > 0 ? Std.parseFloat(Sys.args()[0]) : 2600.0;
    if (bed != 2600 && bed != 3150) throw "Choose bed 2600 or 3150";
    var machine = new SheetRouter(new SheetRouterSpec(bed));
    machine.check().throwIfErrors();
    write(machine, 'build/router-mount-implementation/model-' + Std.int(bed));
  }
}
