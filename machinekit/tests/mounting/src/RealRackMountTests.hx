import machinekit.structural.*;
import machinekit.transmission.*;
import machinekit.component.*;
class RealRackMountTests {
  static var assertions = 0;
  static function check(v:Bool, s:String):Void {
    assertions++;
    if (!v) throw s;
  }
  static function reject(fn:Void -> Void, label:String):Void {
    var rejected = false;
    try fn() catch (_:Dynamic) rejected = true;
    check(rejected, label);
  }
  public static function main():Void {
    reject(() -> {new machinekit.assembly.RackMount(
      "host",
      materia.assembly.AssemblyFrames.identity(),
      "missing",
      [{first: "$rack", second: "missing"}]
    );
    }, "disconnected support rejected");
    reject(() -> {new machinekit.assembly.RackMount(
      "$rack",
      materia.assembly.AssemblyFrames.identity(),
      "floating",
      [{first: "$rack", second: "floating"}]
    );
    }, "rack placeholder cannot serve as host");
    var run = new GearRackRun(new ComponentValues().setInteger("sections", 4));
    reject(() -> {new GearRackRun(new ComponentValues().setInteger("sections", 17));
    }, "run cannot exceed assembly connector capacity");
    reject(() -> {new GearRackRun(new ComponentValues().setToken("designation",
      "29-15-105").setInteger("sections", 10));
    }, "long source sections respect their larger hole count");
    reject(() -> {new GearRackRun(new ComponentValues().setNumber("finishPinDiameter", 5.7));
    }, "supplier pilot is not a finished locating fit");
    check(run.length == 2000, "whole rack sections");
    check(
      run.bom.partNumber == run.section.bom.partNumber && run.bom.quantity == 4,
      "run orders four actual catalogue sections"
    );
    var purchases = new Bom();
    purchases.addComponent(run);
    purchases.addComponent(run.section);
    purchases.addComponent(new GearRackRun(new ComponentValues().setInteger("sections", 2).setNumber(
      "finishPinDiameter",
      6
    )));
    check(
      purchases.lines().length == 1 && purchases.quantity(run.section.bom.partNumber) == 7,
      "finished and plain runs aggregate by supplier purchase"
    );
    check(run.section.bom.quantity == 1, "run does not mutate catalogue BOM quantity");
    var ownedMass = new machinekit.assembly.MachineAssembly();
    ownedMass.place("rack", run, materia.assembly.AssemblyFrames.identity());
    check(
      Math.abs(ownedMass.massProperties().mass - 4 * run.section.massProperties().mass) < 1e-9,
      "section purchase count does not multiply owned run mass twice"
    );

    for (i in 1...5) for (pin in 1...3) run.connector('section$i-pinPilot$pin');
    check(MountedElement.of(run).connectors.length == 16, "all source connectors");
    var cleat = new LinearMountCleat(), body = cleat.geometry();
    check(body.valid() && body.solidCount() == 1, "cleat is a connected drilled solid");
    body.close();
    var result = cleat.connection(1000, 400, 400);
    check(result.pass, "cleat connection envelope");
    Sys.println(haxe.Json.stringify(result));
    var rack = run.section, mesh = RackGeometry.rack(run);
    check(mesh.length == 2000 && mesh.teeth == 400, "run mesh fields");
    var b = new machinekit.assembly.AxisBuilder();
    b.place(
      "host",
      new RectTubeMember(100, 100, 6, 700),
      machinekit.assembly.AxisBuilder.orient(0, -350, -38, [0.0, 0, 1], [0.0, 1, 0])
    );
    var wheel = new HelicalPinion(new ComponentValues().setToken("hand",
      "left")), pose = machinekit.assembly.AxisBuilder.orient(
        -70,
        0,
        58,
        [0.0, 0, 1],
        [1.0, 0, 0]
      );
    var one = new GearRackRun(new ComponentValues().setNumber("finishPinDiameter", 6));
    check(
      one.section.spec.pinPilotMm == 5.7 && one.finishPinDiameter == 6,
      "shop finish retains the supplier pilot dimension"
    );
    reject(() -> {b.rackPlacement({id: "y", lower: 0.0, upper: 100.0, initial: 0.0}, wheel, pose, null, one);
    }, "rack placement has no unmounted path");
    var built = LinearRackMount.build(
      b,
      "rackMount",
      {id: "y", lower: 0.0, upper: 100.0, initial: 0.0},
      [0.0, 1, 0],
      wheel,
      pose,
      one,
      "rack",
      "host",
      "host",
      machinekit.assembly.AxisBuilder.orient(0, 0, 12, [0.0, 0, 1], [0.0, 1, 0]),
      machinekit.assembly.AxisBuilder.orient(50, 0, -38, [0.0, 1, 0], [-1.0, 0, 0])
    );
    check(built.members.length > 0, "physical members");
    var originalCount = built.mount.contacts.length, copy = built.mount.contacts;
    copy.resize(0);
    check(built.mount.contacts.length == originalCount, "contact graph is immutable through its public view");
    var changedDatum = built.mount.datum, originalZ = changedDatum.z;
    changedDatum.z += 100;
    check(built.mount.datum.z == originalZ, "mount datum is immutable through its public view");
    for (id in built.members) if (StringTools.endsWith(id, "Receiver")) {
      var receiver:LinearMountSteel = cast b.component(id);
      check(receiver.length == 498, "receiver joints retain a 2 mm clearance on the 500 mm rack pitch");
      for (requirement in receiver.mountingRequirements()) {
        check(requirement.componentPath == "rack", "lineage names the installed rack occurrence");
        check(StringTools.startsWith(requirement.connectorId, "section1-"), "lineage names the run connector");
        one.connector(requirement.connectorId);
      }
    }
    var cassettes = [for (id in built.members) if (StringTools.endsWith(id, "NutCassette")) id];
    check(cassettes.length == 5, "each cleat has a real captive-nut cassette");
    for (id in cassettes) {
      var cassette:LinearMountNutCassette = cast b.component(id);
      check(
        cassette.compact && cassette.pocketStations().length == 2,
        "compact cassette derives both cleat bolt stations"
      );
      var shape = cassette.geometry();
      check(shape.valid() && shape.solidCount() == 1, "cassette is a connected serviceable solid");
      shape.close();
    }
    var location = machinekit.assembly.RackMesh.layout(
      {id: "y", lower: 0.0, upper: 100.0, initial: 0.0},
      [0.0, 1, 0],
      wheel,
      pose,
      materia.assembly.AssemblyFrames.compose(b.zeroPose("host"), built.mount.datum),
      one
    );
    built.mount.validate(b, one, location.pose);
    var webId = "rackMountSection1Cleat1WebBolt-14", receiverId = "rackMountSection1Receiver";
    var fit = RackMountInterfaces.threadFit(b, receiverId, webId);
    check(fit != null && fit.fullThreadTipGapMm >= 1, "assembled web screw retains free usable thread");
    machinekit.assembly.PosedParts.scope(cache -> RackMountInterfaces.validateThreadSolid(b, cache, fit));
    var originalWeb = b.component(webId);
    b.replaceComponent(
      webId,
      machinekit.standard.SocketHeadCapScrew.metric("M6", 25, machinekit.standard.PropertyClass.Steel109)
    );
    reject(() -> {RackMountInterfaces.threadFit(b, receiverId, webId);
    }, "a free drill bottom does not excuse missing full-thread tip clearance");
    b.replaceComponent(webId, originalWeb);
    var wrong = materia.assembly.AssemblyFrames.compose(b.zeroPose("host"), built.mount.datum);
    wrong.z += 1;
    reject(() -> {machinekit.assembly.RackMesh.layout(
      {id: "y", lower: 0.0, upper: 100.0, initial: 0.0},
      [0.0, 1, 0],
      wheel,
      pose,
      wrong,
      one
    );
    }, "mesh/back-plane mismatch rejected");
    b.place("floating", new LinearMountSteel(), materia.assembly.AssemblyFrames.translation(5000, 0, 0));
    var broken = new machinekit.assembly.RackMount(
      "host",
      built.mount.datum,
      "floating",
      [{first: "$rack", second: "floating"}, {first: "floating", second: "host"}]
    );
    reject(() -> {broken.validate(b, one, location.pose);
    }, "geometric air gap rejected despite declared graph");
    Sys.println("Physical rack contact graph passes");
    var masses:Array<Dynamic> = [], total = 0.0;
    for (id in built.members) {
      var part = b.component(id), mass = part.massProperties(), f = b.zeroPose(id);
      masses.push({id: id, massKg: mass.mass, pose: f});
      total += mass.mass;
    }
    Sys.println("Local mount added mass kg=" + total);
    sys.io.File.saveContent(
      "build/router-mount-implementation/local-mass.json",
      haxe.Json.stringify({massKg: total, members: masses}, null, "  ")
    );

    Sys.println("RealRackMountTests: " + assertions + " local source/contact assertions passed");
    if (Sys.args().indexOf("--fixture-only") >= 0) return;
    for (bed in [2600.0, 3150.0]) {
      Sys.println("Building router " + bed);
      Sys.stdout().flush();
      var router = new SheetRouter(new SheetRouterSpec(bed));
      router.check().throwIfErrors();
      Sys.println("Built router " + bed + " components=" + router.components().length);
      Sys.stdout().flush();
    }
  }
}
