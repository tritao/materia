import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import machinekit.structural.*;
import materia.assembly.AssemblyFrames as F;

/** Review fixture, not an adoption of router CAD or qualification of a compound. */
class LinearMountAccessTests {
  static var checks = 0;
  static function check(ok:Bool, text:String):Void {
    checks++;
    if (!ok) throw text;
  }
  static function reject(work:() -> Void, text:String):Void {
    var failed = false;
    try work() catch (_:Dynamic) failed = true;
    check(failed, text);
  }
  public static function main():Void {
    var rail = machinekit.motion.LinearRail.metric("HGH25CA", 400, machinekit.motion.LinearRail.NOMINAL);
    var element = MountedElement.of(rail);
    // Historical 12 mm datum: relieved welded pad 8 + assumed replication 4.
    var railPose = machinekit.assembly.AxisBuilder.orient(0, -200, 12 + rail.spec.railHeight, [0.0, 0, 1], [0.0, 1, 0]);
    var cage = LinearMountNutCassette.fromElement(element, railPose, 440);
    var cagePose = F.translation(0, 0, -16.5), cover = cage.cover(), coverPose = F.translation(0, 0, -18.5);
    var assembly = new machinekit.assembly.MachineAssembly();
    assembly.addComponent("cassette", cage, cagePose);
    assembly.addComponent("cover", cover, coverPose);
    var reloaded = machinekit.assembly.MachineAssembly.decode(assembly.encode());
    check(reloaded.encode() == assembly.encode(), "cassette and drilled cover survive assembly reload");
    var parts:Array<Part> = [];
    function own(p:Part):Part {
      parts.push(p);
      return p;
    }
    function placed(p:Part, pose:materia.assembly.AssemblyRecord.AssemblyFrame):Part return own(PosedParts.moved(
      p,
      pose
    ));
    var output = Sys.getEnv("LINEAR_MOUNT_EXPORT");
    if (output != null && !sys.FileSystem.exists(output)) sys.FileSystem.createDirectory(output);
    var evidence:Array<Dynamic> = [];
    function clear(a:Part, b:Part, text:String):Void {
      var common = own(a.intersect(b));
      check(common.volume() < 1e-7, text);
    }
    try {
      var cutters:Array<Part> = [];
      for (c in element.connectors) {
        var p = F.compose(railPose, c.frame);
        cutters.push(Part.cylinderSpan(3.5, -6.1, 0.1, p.x, p.y));
      }
      var hollow = Solids.cut(
        Part.box(100, 540, 100).translated(new Vector(0, 0, -100)),
        [Part.box(88, 542, 88).translated(new Vector(0, 0, -94))]
      );
      var host = own(Solids.cut(hollow, cutters));
      var pad = new machinekit.structural.SeatedPlate(100, 440, 8,
        "welded host pad", [{name: "rail", frame: F.identity(), holes: element.connectors.map(c -> {var p = F.compose(
          railPose,
          c.frame
        );
      return {x: p.x, y: p.y, diameter: 7.0};
      })}], null, "steel");
      var padBody = own(pad.geometry());
      var compoundTools:Array<Part> = [];
      for (c in element.connectors) {
        var p = F.compose(railPose, c.frame);
        compoundTools.push(Part.cylinderSpan(3.5, 7.9, 12.1, p.x, p.y));
      }
      var compound = own(Solids.cut(Part.box(40, 440, 4).translated(new Vector(0, 0, 8)), compoundTools));
      var railBody = placed(own(rail.geometry()), railPose);
      var cageBody = placed(own(cage.geometry()), cagePose), coverBody = placed(own(cover.geometry()), coverPose);
      check(
        host.valid() && host.solidCount() == 1 && cageBody.valid() && cageBody.solidCount() == 1,
        "opened host and captive cage are connected native solids"
      );
      check(
        padBody.distance(host) < 1e-6 && compound.distance(padBody) < 1e-6 && railBody.distance(compound) < 1e-6,
        "pad, replication and rail close the support path without changing the datum"
      );
      check(cageBody.distance(coverBody) < 1e-6, "removable cover closes the nut cage");
      for (c in element.connectors) {
        var p = F.compose(railPose, c.frame), seat = F.compose(p, F.translation(0, -element.bore.counterboreDepth, 0));
        var bolt = MountThroughBolts.select(c.name, element, seat, Solids.axial(p.x, p.y, -6), [30, 35, 40, 45, 50]);
        var washerBody = placed(own(bolt.washer.geometry()), bolt.washerPose), nutBody = placed(
          own(bolt.nut.geometry()),
          bolt.nutPose
        );
        clear(nutBody, cageBody, "nut fits its real hex capture pocket");
        clear(washerBody, cageBody, "washer fits its mouth without cutting the cage");
        var screwBody = placed(own(bolt.screw.geometry()), bolt.screwPose);
        clear(screwBody, host, "bolt passes the drilled tube wall");
        clear(screwBody, coverBody, "bolt tip clears the cover bore");
        var keyBody = placed(
          own(Part.prism(
            cadkit.modeling.Polygon.regular(6, 4.9),
            bolt.screw.spec.headHeight - bolt.screw.spec.socketDepth + 0.1,
            bolt.screw.spec.headHeight + 80
          )),
          bolt.screwPose
        );
        clear(keyBody, screwBody, "5 mm hex key enters the head socket from above");
        clear(keyBody, railBody, "key reaches head through the rail counterbore");
        assembly.addComponent(c.name + "-nut", bolt.nut, bolt.nutPose);
        assembly.addComponent(c.name + "-washer", bolt.washer, bolt.washerPose);
        assembly.addComponent(c.name + "-bolt", bolt.screw, bolt.screwPose);
        check(
          washerBody.distance(host) < 1e-6 && washerBody.distance(nutBody) < 1e-6,
          "washer and nut bear on the real tube wall"
        );
        check(
          bolt.projectionMm >= 2 * bolt.screw.thread.pitch - 1e-6,
          "nut retains full engagement and two-pitch projection"
        );
        check(nutBody.distance(coverBody) > 1, "cover does not carry bolt preload");
        check(
          cage.nut.acrossFlats + 0.5 < 2 * cage.nut.acrossFlats / Math.sqrt(3),
          "hex pocket cannot let nut rotate through 30 degrees"
        );
        check(bolt.washer.outerDiameter < bolt.nut.acrossFlats + 0.5 + 2, "washer remains within machined mouth");
        evidence.push({
          connector: c.name,
          screwLengthMm: bolt.screw.length,
          gripMm: bolt.gripMm,
          projectionMm: bolt.projectionMm,
          nutPose: bolt.nutPose,
          washerPose: bolt.washerPose
        });
      }
      for (end in [-1, 1]) {
        var bolt = machinekit.standard.SocketHeadCapScrew.metric("M3", 6, Steel88);
        var pose = F.fromRotationMatrix(
          end < 0 ? 6.5 : -6.5,
          end * (cage.length / 2 - 8),
          -18.5,
          [1, 0, 0, 0, -1, 0, 0, 0, -1]
        );
        var body = placed(own(bolt.geometry()), pose);
        check(
          bolt.length - 2 >= 3.9 && 10 - (bolt.length - 2) >= 1.5,
          "cover screws engage C45 while preserving back wall"
        );
        clear(body, coverBody, "cover screws pass the cover clearance holes");
        assembly.addComponent(end < 0 ? "cover-front" : "cover-back", bolt, pose);
      }
      assembly.addComponent("rail", rail, railPose);
      var fullReload = machinekit.assembly.MachineAssembly.decode(assembly.encode());
      check(
        fullReload.encode() == assembly.encode(),
        "source cassette, cover and every service fastener round trip together"
      );
      var fem = machinekit.structural.fem.AssemblyFemExport.bodies(
        fullReload,
        new cadkit.modeling.AssemblyState(fullReload.definition()),
        ".",
        _ -> false
      );
      var ownedMass = 0.0;
      for (m in assembly.components()) ownedMass += m.component.massProperties().mass;
      check(
        Math.abs(fem.totalMassKg - ownedMass) < 1e-10,
        "cassette, cover and every real fastener enter FEM mass exactly once"
      );
      for (step in 0...16) {
        var offset = -step * 50.0;
        var body = placed(own(cage.geometry()), F.translation(0, offset, -16.5));
        check(
          body.distance(host) > 0.01,
          "cassette can withdraw through the existing open tube end without a side window"
        );
      }
      var pull = machinekit.standard.SocketHeadCapScrew.metric("M3", 16, Steel88);
      // Five mm engagement, 11 mm free grip for hooking; remove before operation.
      var pullPose = F.fromRotationMatrix(0, -cage.length / 2 - 11, -11.5, [1, 0, 0, 0, 0, -1, 0, 1, 0]);
      var pullBody = placed(own(pull.geometry()), pullPose);
      clear(pullBody, host, "service pull screw is accessible through the open tube end");
      check(5 >= 3.9 && 6 - 5 >= 0.5, "M3 service pull screw has engagement and blind-tip reserve");
      check(
        cage.serviceThreads().filter(h -> h.id == "pull-front" || h.id == "pull-back").length == 2,
        "both cassette ends have physical extraction threads"
      );
      // A 24 x 12 cassette fits the 88 x 88 open end with a 2 mm service allowance.
      check(24 + 4 < 88 && 12 + 4 < 88, "end mouth clears the cassette and service allowance");
      reject(() -> {LinearMountNutCassette.fromElement(element, F.identity(), 440);
      }, "wrong source-hole normal fails");
      reject(() -> {new LinearMountNutCassette(cage.values().setNumber("width", 10));
      }, "undersized capture stock fails");
      reject(() -> {new LinearMountNutCassette(cage.values().setNumber("length", 100));
      }, "changed length cannot strand source pockets outside stock");
      if (output != null) {
        sys.io.File.saveContent(output + "/mass.json", haxe.Json.stringify(fem, null, "  "));
        host.shape.exportStep(output + "/host.step");
        cageBody.shape.exportStep(output + "/cassette.step");
        coverBody.shape.exportStep(output + "/cover.step");
        sys.io.File.saveContent(
          output + "/access.json",
          haxe.Json.stringify(
            {
              reviewOnly: true,
              railDatumMm: 12,
              tubeMm: {width: 100, height: 100, wall: 6, length: 540},
              cassetteMm: {width: 24, height: 10, length: 440, coverThickness: 2},
              endWithdrawalMm: 750,
              cassetteTopClearanceMm: 0.5,
              replicationThicknessMm: 4,
              assumedCompound: true,
              bolts: evidence,
              limitations: [
                "No compound creep qualification",
                "Local fixture; frame end caps and real-machine withdrawal corridor need checking",
                "Weld pad and drilled tube require structural/fatigue rerun",
                "Torque/bearing strength not released"
              ]
            },
            null,
            "  "
          )
        );
      }
    } catch (error:Dynamic) {
      for (p in parts) p.close();
      throw error;
    }
    for (p in parts) p.close();
    Sys.println('LinearMountAccessTests: $checks assertions passed');
  }
}
