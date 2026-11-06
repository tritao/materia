import materia.casting.CastingInput;
import machinekit.structural.MountHoleDerivation;
import machinekit.structural.InsertMountPattern.MountHoleErrorAllocation;
import machinekit.motion.LinearRail;
import machinekit.transmission.GearRack;
import materia.assembly.AssemblyFrames as F;
import machinekit.assembly.AxisBuilder;

class LinearMountTests {
  static var count = 0;
  static function check(ok:Bool, message:String):Void {
    count++;
    if (!ok) throw message;
  }
  static function rejected(work:() -> Void, message:String):Void {
    var fails = false;
    try work() catch (_:Dynamic) fails = true;
    check(fails, message);
  }
  static function allocation():MountHoleErrorAllocation return {
    placementOffsetMm: Unresolved("placementOffsetMm"),
    placementAngleRad: Unresolved("placementAngleRad"),
    assemblyAdjustmentMm: Unresolved("assemblyAdjustmentMm"),
    cureBowMm: Unresolved("cureBowMm"),
    machiningErrorMm: Unresolved("machiningErrorMm"),
    pitchAccumulationMm: Unresolved("pitchAccumulationMm"),
    requiredMarginMm: Unresolved("requiredMarginMm")
  };
  public static function main():Void {
    var rail = LinearRail.metric("HGH25CA", 400, LinearRail.NOMINAL);
    var pose = AxisBuilder.orient(0, -200, 35, [0.0, 0, 1], [0.0, 1, 0]);
    var steel = {x: -20.0, y: -220.0, z: 0.0, width: 40.0, depth: 440.0, height: 20.0};
    var result = MountHoleDerivation.derive(
      steel,
      C45Bright,
      rail,
      "rail",
      haxe.crypto.Sha256.encode("fixture"),
      pose,
      rail.mountingConnectors().map(c -> c.name),
      rail.mountingThread(),
      rail.mountingHole().diameter,
      rail.mountingHole().reference,
      [TransferAfterCasting],
      allocation()
    );
    check(result.holes.length == rail.mountingConnectors().length, "rail holes follow actual connectors");
    for (i in 0...result.holes.length) {
      var h = result.holes[i], r = result.requirements[i];
      check(Math.abs(h.frame.z - 20) < 1e-8, "thread starts on receiver steel, not rail or compound");
      check(r.connectorId == rail.mountingConnectors()[i].name, "connector identity retained");
      check(switch r.budget.placementOffsetMm {
        case Unresolved(_):
          true;
        default:
          false;
      }, "unknown budget remains pending");
      check(result.routes[i].route == TransferAfterCasting, "pending budget selects permitted transfer proposal");
    }
    var rack = GearRack.catalogue("29-15-055");
    // Rack drills across its width, independently of the replicated back plane.
    var rackPose = F.fromRotationMatrix(0, -250, 45, [0, -1, 0, 0, 0, 1, -1, 0, 0]);
    var rackSteel = {x: -25.0, y: -270.0, z: 0.0, width: 50.0, depth: 540.0, height: 20.0};
    var holes = MountHoleDerivation.derive(
      rackSteel,
      C45Bright,
      rack,
      "rack",
      haxe.crypto.Sha256.encode("supplier"),
      rackPose,
      rack.mountingConnectors().map(c -> c.name),
      rack.mountingThread(),
      rack.mountingHole().diameter,
      rack.mountingHole().reference,
      [TransferAfterCasting],
      allocation()
    );
    check(holes.holes.length == 4, "catalogue rack supplies four holes, not assumed 100 mm pitch");
    check(Math.abs(holes.holes[1].frame.y - holes.holes[0].frame.y - 125) < 1e-8, "supplier rack pitch retained");
    rejected(() -> {MountHoleDerivation.derive(
      steel,
      C45Bright,
      rail,
      "rail",
      haxe.crypto.Sha256.encode("fixture"),
      F.identity(),
      ["mount1"],
      rail.mountingThread(),
      7,
      "fixture",
      [TransferAfterCasting],
      allocation()
    );
    }, "wrong normal fails");
    rejected(() -> {MountHoleDerivation.derive(
      steel,
      C45Bright,
      rail,
      "rail",
      haxe.crypto.Sha256.encode("fixture"),
      pose,
      ["mount1", "mount1"],
      rail.mountingThread(),
      7,
      "fixture",
      [TransferAfterCasting],
      allocation()
    );
    }, "duplicate connector fails");
    for (pair in [
      {element: machinekit.structural.MountedElement.of(rail), holes: result},
      {element: machinekit.structural.MountedElement.of(rack), holes: holes}
    ]) {
      var element = pair.element, placed = element.component == rail ? pose : rackPose;
      var h = pair.holes.holes[0];
      var facet = machinekit.structural.InsertHoles.facet(pair.holes.holes, 50, 600, 20);
      var localSeat = F.compose(element.connectors[0].frame, F.translation(0, -element.bore.counterboreDepth, 0));
      var seat = F.compose(placed, localSeat);
      var screw = machinekit.structural.MountServiceScrews.select(
        "fixture",
        element,
        seat,
        facet.holes[0],
        h.frame,
        [12, 16, 20, 25, 30, 35]
      );
      check(screw.length > 0, "shared fastener selector finds positive grip and policy engagement");
      if (element.component == rack) check(
        Math.abs(localSeat.x - 7) < 1e-9,
        "rack head seats at actual counterbore bottom x7"
      );
      check(element.fingerprint("fixture") == element.fingerprint("fixture"), "element identity stable");
    }
    rejected(() -> {machinekit.structural.MountedElement.of(new machinekit.motion.LinearScale(100, 200));
    }, "generic tape scale cannot invent mounting holes");
    var inputs:machinekit.structural.LinearMountDatum.ReplicationLayerInputs = {
      compoundReference: "fixture assumptions, not supplier limits",
      minimumMm: 2,
      maximumMm: 8,
      fixtureErrorMm: 0.5,
      cureDistortionMm: 0.1
    };
    switch machinekit.structural.LinearMountDatum.allowance(inputs) {
      case LayerInterval(lo, hi):
        check(
          Math.abs(lo - 2.6) < 1e-9 && Math.abs(hi - 7.4) < 1e-9,
          "one shared replication interval includes both error bounds"
        );
      default:
        throw "Known fixture layer interval expected";
    }
    inputs.cureDistortionMm = null;
    switch machinekit.structural.LinearMountDatum.allowance(inputs) {
      case AwaitLayerInputs(missing):
        check(missing.indexOf("cureDistortionMm") >= 0, "missing cure input remains pending");
      default:
        throw "Unknown layer must not resolve";
    }
    inputs.cureDistortionMm = 4;
    rejected(() -> {machinekit.structural.LinearMountDatum.allowance(inputs);
    }, "impossible layer envelope fails");
    inputs.cureDistortionMm = 0.1;
    var assembly = new machinekit.assembly.MachineAssembly();
    function plate(name:String,
      material:String):machinekit.structural.SeatedPlate return new machinekit.structural.SeatedPlate(
        "TEST-" + name.toUpperCase(),
        60,
        60,
        20,
        name,
        [],
        null,
        material
      );
    assembly.addComponent("pad", plate("welded pad", "steel"));
    assembly.addComponent("receiver", plate("receiver", "steel C45"));
    assembly.addComponent("layer", plate("replication fixture", "plastic"));
    assembly.addComponent(
      "push",
      machinekit.standard.SocketSetScrew.forThread(machinekit.standard.FastenerThreads.metric(4, 0.7), 16)
    );
    assembly.addComponent("lock", machinekit.standard.SocketHeadCapScrew.metric("M6", 20, Steel88));
    var datum = new machinekit.structural.LinearMountDatum(
      RelievedWeldedPad("pad", "fixture thermal relief"),
      F.identity(),
      "receiver",
      "receiver",
      "layer",
      ["push"],
      ["lock"],
      inputs
    );
    datum.validateMembers(assembly);
    check(datum.hostMember() == "pad", "welded host retains its separate identity");
    var expected = 0.0;
    for (m in assembly.components()) if (m.id != "pad") expected += m.component.massProperties().mass;
    check(Math.abs(datum.addedMassKg(assembly) - expected) < 1e-12, "shared bar/receiver contributes mass once");
    rejected(() -> {new machinekit.structural.LinearMountDatum(
      RetainedInsert("pad", "fixture"),
      F.identity(),
      "receiver",
      "receiver",
      "layer",
      ["push"],
      ["lock"],
      inputs
    ).validateMembers(assembly);
    }, "welded steel cannot masquerade as a cast insert");
    rejected(() -> {new machinekit.structural.LinearMountDatum(
      RelievedWeldedPad("pad", "fixture"),
      F.identity(),
      "layer",
      "receiver",
      "layer",
      ["push"],
      ["lock"],
      inputs
    ).validateMembers(assembly);
    }, "compound cannot receive lateral clamp loads");
    rejected(() -> {new machinekit.structural.LinearMountDatum(
      RelievedWeldedPad("pad", "fixture"),
      F.identity(),
      "receiver",
      "receiver",
      "layer",
      [],
      ["lock"],
      inputs
    );
    }, "missing side adjustment fails");
    rejected(() -> {new machinekit.structural.LinearMountDatum(
      RelievedWeldedPad("pad", "fixture"),
      F.identity(),
      "receiver",
      "receiver",
      "layer",
      ["missing"],
      ["lock"],
      inputs
    ).validateMembers(assembly);
    }, "missing support hardware fails");
    var values = machinekit.structural.CastInsert.boxValues(40, 440, 20).setToken(
      "role",
      materia.casting.InsertSpec.InsertRole.GenericMount
    );
    var source = new machinekit.structural.CastInsert(values);
    var castSeat = machinekit.structural.InsertMountPattern.derive(
      source,
      rail,
      "rail",
      haxe.crypto.Sha256.encode("fixture"),
      pose,
      rail.mountingConnectors().map(c -> c.name),
      rail.mountingThread(),
      7,
      "fixture",
      [TransferAfterCasting],
      allocation()
    );
    check(castSeat.insertSpec().holes.length == result.holes.length, "cast adapter uses identical shared derivation");
    check(source.insertSpec().holes.length == 0, "derivation leaves the source insert unchanged");
    rejected(() -> {machinekit.structural.InsertHoles.tappedBlind(
      machinekit.structural.InsertHoles.metricThread(rail.mountingThread()),
      8,
      C45Bright
    );
    }, "legacy 8 mm backing cannot silently acquire a blind M6 thread");
    rejected(() -> {MountHoleDerivation.derive(
      steel,
      C45Bright,
      rail,
      "rail",
      "not-a-fingerprint",
      pose,
      ["mount1"],
      rail.mountingThread(),
      7,
      "fixture",
      [TransferAfterCasting],
      allocation()
    );
    }, "invalid lineage fingerprint fails on welded route too");
    var badSteel = {x: -20.0, y: -220.0, z: 0.0, width: 40.0, depth: Math.NaN, height: 20.0};
    rejected(() -> {MountHoleDerivation.derive(
      badSteel,
      C45Bright,
      rail,
      "rail",
      haxe.crypto.Sha256.encode("fixture"),
      pose,
      ["mount1"],
      rail.mountingThread(),
      7,
      "fixture",
      [TransferAfterCasting],
      allocation()
    );
    }, "nonfinite receiver geometry fails");
    var sharedReceiver = machinekit.structural.LinearMountSteel.receiver(40, 440, 20, result);
    check(
      sharedReceiver.mountingRequirements().length == rail.mountingConnectors().length,
      "same physical receiver recipe retains rail lineage as well as rack lineage"
    );
    rejected(() -> {new machinekit.structural.LinearMountSteel(sharedReceiver.values().setNumber("thickness", 8));
    }, "a thinner receiver cannot retain deep service holes from the previous design");
    assembly.addComponent("washer", machinekit.standard.FlatWasher.metric("M6"));
    var ownedDatum = new machinekit.structural.LinearMountDatum(
      RelievedWeldedPad("pad", "fixture relief"),
      F.identity(),
      "receiver",
      "receiver",
      "layer",
      ["push"],
      ["lock"],
      inputs,
      ["washer", "receiver"]
    );
    check(
      Math.abs(ownedDatum.addedMassKg(assembly)
      - datum.addedMassKg(assembly) - machinekit.standard.FlatWasher.metric("M6").massProperties().mass) < 1e-12,
      "other hardware included once, overlapping receiver not duplicated"
    );
    rejected(() -> {new machinekit.structural.LinearMountDatum(
      RelievedWeldedPad("pad", "fixture"),
      F.identity(),
      "receiver",
      "receiver",
      "layer",
      ["push"],
      ["lock"],
      inputs,
      ["pad"]
    ).validateMembers(assembly);
    }, "shared host cannot become added mount mass");
    var axial = machinekit.structural.LinearMountCompliance.axialMPerN(4, 900, 4e9);
    check(Math.abs(axial - 1.111111111111111e-9) < 1e-20, "layer axial flexibility uses SI units");
    check(
      Math.abs(machinekit.structural.LinearMountCompliance.axialMPerN(4, 900, 4e9, 2) - axial / 2) < 1e-20,
      "parallel pads add stiffness"
    );
    var bar = machinekit.structural.LinearMountCompliance.barMPerN(100, 28, 20, 210e9);
    check(
      Math.abs(machinekit.structural.LinearMountCompliance.barMPerN(200, 28, 20, 210e9) / bar - 8) < 1e-12,
      "bar bending follows span cubed"
    );
    var record:machinekit.structural.LinearMountCompliance.MountComplianceRecord = {
      schema: 1,
      host: "pad",
      element: "rail",
      frame: F.identity(),
      terms: [
        {
          name: "compound",
          members: ["layer"],
          xMPerN: axial,
          yMPerN: axial,
          zMPerN: axial,
          reference: "fixture E=4 GPa assumed",
          assumed: true
        },
        {
          name: "receiver",
          members: ["receiver"],
          xMPerN: bar,
          yMPerN: bar,
          zMPerN: bar,
          reference: "simply supported fixture; assumed support",
          assumed: true
        }
      ]
    };
    var compliance = new machinekit.structural.LinearMountCompliance(record);
    check(
      Math.abs(compliance.stiffnessNPerM().x - 1 / (axial + bar)) < 1e-6,
      "series flexibility is summed before inversion"
    );
    record.terms[0].xMPerN = 0;
    rejected(() -> {new machinekit.structural.LinearMountCompliance(record);
    }, "unknown or rigid default direction fails");
    check(compliance.snapshot().terms[0].xMPerN == axial, "facet owns a snapshot of its inputs");
    rejected(() -> {machinekit.structural.LinearMountCompliance.axialMPerN(4, 900, Math.NaN);
    }, "nonfinite mechanics input fails");
    check(
      machinekit.structural.LinearMountCompliance.decode("null") == null,
      "legacy member has no invented compliance"
    );
    Sys.println('LinearMountTests: $count assertions passed');
  }
}
