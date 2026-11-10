import hobber.*;
import machinekit.structural.MineralCasting;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import cadkit.modeling.Part;
import materia.casting.CastingCavities;
import materia.casting.CastingPatterns;
import materia.assembly.AssemblyFrames as F;

/** Independent frozen pre-migration CAD plus exact neutral/CAD intersection checks. */
class FoundationDesignTests {
  static var assertions = 0;

  static function same(a:Part, b:Part, id:String):Void {
    var added = a.subtract(b), removed = b.subtract(a), difference = added.volume() + removed.volume();
    added.close();
    removed.close();
    assertions++;
    if (difference > 1e-3) throw id + " symmetric difference: " + difference;
    Sys.println(id + " symmetric difference: " + difference);
  }

  /** Certify that the approved radius adds only the two intended fillets,
	 * leaving the base and every column service-hole frame unchanged. */
  static function columnCornerCorrection(frame:BenchMillFrame):Void {
    var details = new MillCastingDetails();
    details.columnFootCornerRadiusMm = 3.2;
    var previous = BenchMillFrame.create(frame.layout, frame.zAxis, frame.xyAxis, details);
    var scope = new cadkit.modeling.Scope();
    try {
      var radius = frame.casting.columnFootCornerRadiusMm;
      assertions++;
      if (radius != 12) throw "Approved structural column corner radius must be 12 mm";
      var before = scope.own(previous.column.body.geometry());
      var after = scope.own(frame.column.body.geometry());
      var added = scope.own(after.subtract(before));
      var removed = scope.own(before.subtract(after));
      var regions:Array<Part> = [];
      for (side in [-1, 1]) {
        var local = scope.own(Part.box(radius, frame.layout.columnDepth, radius));
        regions.push(scope.own(local.translated(new cadkit.modeling.Vector(
          side * (frame.layout.columnWidth / 2 + radius / 2),
          0,
          machinekit.milling.MillFrameLayout.COLUMN_FOOT_HEIGHT
        ))));
      }
      var outside = scope.own(added.subtractAll(regions));
      var expected = 2 * frame.layout.columnDepth * (1 - Math.PI / 4) * (radius * radius - 3.2 * 3.2);
      assertions++;
      if (removed.volume() > 1e-3 || outside.volume() > 1e-3 || Math.abs(added.volume() - expected)
        > 1e-3) throw "Column corner correction changes geometry outside the approved fillets";
      Sys.println("Approved column corner correction adds "
        + added.volume() + " mm3; removed " + removed.volume() + " mm3");
      var oldRows:Array<machinekit.structural.MineralFoundation.PostCureInsertPlacement
        > = [for (row in previous.column.insertLocations()) {
        id:row.id, purpose:row.purpose, part:row.part, pose:row.pose
      }
      ];
      oldRows = oldRows.concat(previous.column.postCureLocations());
      var rows:Array<machinekit.structural.MineralFoundation.PostCureInsertPlacement
        > = [for (row in frame.column.insertLocations()) {
        id:row.id, purpose:row.purpose, part:row.part, pose:row.pose
      }
      ];
      rows = rows.concat(frame.column.postCureLocations());
      for (row in rows) {
        var old = oldRows.filter(entry -> entry.id == row.id);
        assertions++;
        if (old.length != 1
          || materia.tooling.ToolingCodec.canonical(old[0].pose) != materia.tooling.ToolingCodec.canonical(row.pose)
          || materia.tooling.ToolingCodec.canonical(old[0].part.insertSpec().holes)
          != materia.tooling.ToolingCodec.canonical(row
          .part.insertSpec().holes)) throw "Column corner correction moved an insert or service hole: " + row.id;
      }
      same(
        scope.own(previous.base.body.geometry()),
        scope.own(frame.base.body.geometry()),
        "mill-base unchanged by column corner correction"
      );
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }

  /** Source pockets meet at declared openings; they never share interior.
	 * Bounds only discard separated pairs. Profiles use the exact CAD cavity. */
  static function disjointPockets(source:MineralCasting, id:String):Void {
    var cavities = source.finishedCavities(), envelope = source.castingEnvelope();
    var entries = [for (feature in source.castingFeatures()) if (feature.kind != InsertRecess) {
      id:feature.id, bounds:materia.casting.CastingRegionBounds.box(CastingCavities.cutRegion(envelope,
        feature)), cavity:cavities.filter(cavity -> cavity.featureId == feature.id)[0]
    }
    ];
    for (index in 0...entries.length) for (other in index + 1...entries.length) {
      var a = entries[index], b = entries[other];
      if (a.bounds != null && b.bounds != null && materia.casting.BoxAlgebra.intersection(a.bounds,
        b.bounds) == null) continue;
      var common = Solids.building([machinekit.structural.CastingCavityCad.part(a.cavity)],
        parts -> {parts.push(machinekit.structural.CastingCavityCad.part(b.cavity));
      parts.push(parts[0].intersect(parts[1]));
      parts[0].close();
      parts[1].close();
      return parts[2];
      });
      var volume:Float;
      try volume = common.volume() catch (error:Dynamic) {
        common.close();
        throw error;
      }
      common.close();
      if (volume > 1e-7) throw "Source pockets overlap: " + id + ": " + a.id + ", " + b.id;
    }
    assertions++;
  }

  /** Catalogue sockets remain single source inserts, without duplicate box seats. */
  static function socketCount(foundation:machinekit.structural.MineralFoundation):Void {
    var sockets = foundation.insertLocations().filter(row -> switch row.part.insertSpec().origin {
      case Catalogue(product):
        product.reference == machinekit.structural.FixingSockets.PFEIFER_REFERENCE;
      default:
        false;
    }
    );
    assertions++;
    if (sockets.length != 12) throw "Hobber tray needs twelve catalogue sockets without duplicate box seats";
  }

  static function sourceGeometrySupport():Void {
    for (rounded in [false, true]) {
      var feature = CastingPatterns.box(
        rounded ? "rounded-source" : "drafted-source",
        materia.casting.CastingFeature.CastingFeatureKind.Lightening,
        [CastingPatterns.face("body.z-min")],
        20,
        20,
        20
      );
      if (rounded) feature.cornerRadius = 3;
      else feature.draft = {degrees: 3, axis: {x: 0, y: 0, z: -1}};
      var source = new MineralCasting(new ComponentValues().setNumber("width", 100).setNumber("depth", 80).setNumber(
        "height",
        120
      ).setToken(
        "features",
        CastingPatterns.encode([feature])
      ));
      assertions++;
      if (source.collisionHullQualification.status
        != machinekit.structural.CastingCollisionQualification.CastingCollisionStatus
        .Unsupported) throw "A box-only hull must not approximate rounded or drafted source intent";
      if (rounded) {
        var body = source.geometry(), volume = body.volume();
        body.close();
        var expected = 100 * 80 * 120 - (400 - (4 - Math.PI) * 9) * 20;
        assertions++;
        if (Math.abs(volume - expected) > 1e-5) throw "Rounded source CAD differs from the analytic cavity volume";
      } else {
        var body = source.geometry(), volume = body.volume();
        body.close();
        var offset = 20 * Math.tan(3 * Math.PI / 180);
        var expected = 100 * 80 * 120 - 20 * (400 - 40 * offset + 4 * offset * offset / 3);
        assertions++;
        if (Math.abs(volume - expected) > 1e-5) throw "Drafted source CAD differs from the analytic cavity volume";
      }
    }
  }

  static function buriedThreadExit():Void {
    var choice =
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating());
    var dimensions = new ComponentValues().setNumber("width", 100).setNumber("depth", 100).setNumber("height", 60);
    var insertValues = machinekit.structural.CastInsert.boxValues(40, 40, 25).setToken("role", "threaded-anchor");
    var failed = false;
    try new machinekit.structural.MineralFoundation(
      dimensions,
      [{
        id: "anchor",
        purpose:materia.casting.InsertSpec.InsertRole.ThreadedAnchor,
        pose: F.translation(0, 0, 35),
        openings: [materia.casting.CastingPatterns.face("body.z-max")],
        part: new machinekit.structural.CastInsert(insertValues.setToken(
          "holes",
          machinekit.structural.InsertHoles.tappedThrough(choice, 25)
        ))
      }]
    ) catch (error:Dynamic) failed =
      Std.string(error).indexOf("Through insert thread exits into casting material: anchor/fixing") >= 0;
    assertions++;
    if (!failed) throw "A buried tapped through-hole must be rejected by the source design";
    var foundation = new machinekit.structural.MineralFoundation(
      dimensions,
      [{
        id: "anchor",
        purpose:materia.casting.InsertSpec.InsertRole.ThreadedAnchor,
        pose: F.translation(0, 0, 35),
        openings: [materia.casting.CastingPatterns.face("body.z-max")],
        part: new machinekit.structural.CastInsert(insertValues.setToken(
          "holes",
          machinekit.structural.InsertHoles.tappedBlind(choice, 25)
        ))
      }]
    );
    var hole =
      machinekit.structural.InsertHoles.records(foundation.insertLocations()[0].part.parameterToken("holes"))[0];
    assertions++;
    if (hole.extent != Blind || hole.depth != 22 || hole.clearanceDiameter
      != 13.5) throw "Blind C45 anchor must preserve its policy back wall and ISO 273 medium clearance";
    for (width in [40.0, 4.0]) {
      var opening = machinekit.structural.CastingFeaturePatterns.box(
        "service",
        ServiceOpening,
        [materia.casting.CastingPatterns.face("body.z-min")],
        width,
        40,
        35,
        0,
        0,
        0
      );
      var accepted = true;
      try new machinekit.structural.MineralFoundation(
        dimensions.setToken("features", materia.casting.CastingPatterns.encode([opening])),
        [{
          id: "anchor",
          purpose:materia.casting.InsertSpec.InsertRole.ThreadedAnchor,
          pose: F.translation(0, 0, 35),
          openings: [
            materia.casting.CastingPatterns.face("body.z-max"),
            materia.casting.CastingPatterns.feature("service")
          ],
          part: new machinekit.structural.CastInsert(insertValues.setToken(
            "holes",
            machinekit.structural.InsertHoles.tappedThrough(choice, 25)
          ))
        }]
      ) catch (error:Dynamic) {
        if (Std.string(error).indexOf("Through insert thread exits into casting material: anchor/fixing")
          < 0) throw error;
        accepted = false;
      }
      assertions++;
      if (accepted
        != (width == 40)) throw "The whole thread exit disk must open into the declared feature, not just its centre";
    }

  }

  public static function main():Void {
    sourceGeometrySupport();
    buriedThreadExit();
    var flat = new machinekit.assembly.Diagnostics();
    machinekit.design.DesignChecks.blindFastener(flat, "flat-bottom anchor", 40, 20, 23.5, 27, 0, 30, 15.6, 1.75, 3);
    flat.throwIfErrors();
    assertions++;
    var negativePoint = false;
    try machinekit.design.DesignChecks.blindFastener(flat, "invalid point", 40, 20, 23.5, 27, -1, 30,
      15.6, 1.75, 3) catch (error:Dynamic) negativePoint = Std.string(error).indexOf("Invalid blind drill point") >= 0;
    assertions++;
    if (!negativePoint) throw "A flat bottom must not admit an invalid negative drill point";


    var spec = machinekit.standard.FastenerThreads.coarse(12).mating();
    var limits = machinekit.design.ThreadedHolePolicy.resolve(spec, C45Bright);
    assertions++;
    if (Math.abs(limits.minEngagement - 15.6) > 1e-9
      || limits.minTipGap != spec.pitch || limits.minBack != 3 || limits.minWall != 12
      || Math.abs(limits.minClearance - 1.2) > 1e-9) throw "Steel hole policy must resolve dimensional design minima";
    var custom = machinekit.design.ThreadedHolePolicy.resolve(spec, C45Bright, {minEngagement: 18});
    assertions++;
    if (custom.minEngagement != 18
      || custom.overrides.minEngagement != 18 || custom.minWall != limits.minWall || limits.overrides.minEngagement
      != null) throw "Hole limit overrides must be recorded without changing policy defaults";
    var rejected = false;
    try machinekit.design.ThreadedHolePolicy.resolve(spec,
      C45Bright, {minWall: 0}) catch (e:Dynamic) rejected = Std.string(e).indexOf("finite and positive: minWall") >= 0;
    assertions++;
    if (!rejected) throw "Hole policy accepted a placeholder wall limit";
    var pattern = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6, 20));
    pattern[0].id = "left";
    pattern[0].frame.x = -10;
    var right = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6, 20))[0];
    right.id = "right";
    right.frame.x = 10;
    pattern.push(right);
    var encoded = machinekit.structural.InsertHoles.fromNeutral(pattern);
    var restored = machinekit.structural.InsertHoles.records(encoded);
    var interfaces = machinekit.structural.InsertHoles.facet(restored, 50, 40, 20);
    var copied = machinekit.structural.InsertHoles.copy(interfaces);
    assertions++;
    if (copied.length != 2
      || copied[0].frame.x != -10 || copied[1].frame.x != 10 || machinekit.structural.InsertHoles.fromNeutral(copied)
      != encoded) throw "Insert hole pattern must retain frames and resolved policy through the facet";
    copied[1].counterbore = {diameter: 12, depth: 3};
    var analytic = materia.casting.InsertHoleGeometry.massProperties(50, 40, 20, copied);
    var insert = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 40, 20).setToken(
      "role",
      "generic-mount"
    ).setToken(
      "holes",
      machinekit.structural.InsertHoles.fromNeutral(copied)
    ));
    var solid = insert.geometry(), physical = solid.massProperties();
    solid.close();
    assertions++;
    if (Math.abs(analytic.volume - physical.volume) > 1e-6 || Math.abs(analytic.center.x - physical.centerOfMass.x)
      > 1e-7 || Math.abs(analytic.center.z - physical.centerOfMass.z)
      > 1e-7) throw "Multi-hole insert mass and centroid must agree with actual drilled steel CAD";
    var blind = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.through(6, 20))[0];
    blind.extent = materia.casting.InsertHole.HoleExtent.Blind;
    blind.depth = 8;
    blind.counterbore = {diameter: 12, depth: 3};
    var blindInsert = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50,
      40, 20).setToken(
        "role",
        "generic-mount"
      ).setToken(
        "holes",
        machinekit.structural.InsertHoles.fromNeutral([blind])
      ));
    var drilled = blindInsert.geometry(), actualBlind = drilled.massProperties();
    var pin = machinekit.tooling.ToolingPin.boreMask(blind).geometry(), mask = PosedParts.moved(pin, blind.frame);
    pin.close();
    var common = mask.intersect(drilled), overlapVolume = common.volume();
    common.close();
    mask.close();
    drilled.close();
    var expectedBlind = materia.casting.InsertHoleGeometry.massProperties(50, 40, 20, [blind]);
    assertions++;
    if (overlapVolume > 1e-7 || Math.abs(expectedBlind.volume - actualBlind.volume)
      > 1e-6) throw "Blind counterbore depth and stepped protection must agree with actual CAD";
    var tapped = machinekit.structural.InsertHoles.records(machinekit.structural.InsertHoles.tappedThrough(
      machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating()),
      20
    ));
    tapped[0].limits = machinekit.design.ThreadedHolePolicy.neutral(custom);
    var overridden = machinekit.structural.InsertHoles.fromNeutral(tapped);
    var recovered = machinekit.structural.InsertHoles.copy(machinekit.structural.InsertHoles.facet(
      machinekit.structural.InsertHoles.records(overridden),
      50,
      40,
      20
    ));
    assertions++;
    if (recovered[0].limits.policyId != custom.policyId
      || recovered[0].limits.overrides[0].value != 18 || machinekit.structural.InsertHoles.fromNeutral(recovered)
      != overridden) throw "Insert facet lost resolved limits or their recorded overrides";
    var target = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 40, 20).setToken(
      "role",
      "generic-mount"
    ).setToken(
      "holes",
      machinekit.structural.InsertHoles.tappedThrough(
        machinekit.structural.InsertHoles.metricThread(machinekit.standard.FastenerThreads.coarse(12).mating()),
        20
      )
    ));
    var clamp = new machinekit.structural.CastInsert(machinekit.structural.CastInsert.boxValues(50, 50, 5).setToken(
      "role",
      "generic-mount"
    ).setToken(
      "holes",
      machinekit.structural.InsertHoles.through(14, 5)
    ));
    var screw = machinekit.standard.SocketHeadCapScrew.metric("M12", 30, machinekit.standard.PropertyClass.Steel88);
    var head = materia.assembly.AssemblyFrames.fromRotationMatrix(0, 0, 25, [1, 0, 0, 0, -1, 0, 0, 0, -1]);
    var joint = new machinekit.assembly.MachineAssembly();
    joint.addComponent("target", target);
    joint.addComponent("clamp", clamp, materia.assembly.AssemblyFrames.translation(0, 0, 20));
    joint.addComponent(
      "screw",
      screw,
      materia.assembly.AssemblyFrames.compose(head, materia.assembly.AssemblyFrames.inverse(screw.bearingFrame()))
    );
    joint.addDesignRequirement(machinekit.design.DesignRequirement.FastenerJoint(
      "through-fixing",
      "screw",
      {member: "target", feature: "fixing"},
      [{member: "clamp", feature: "fixing"}]
    ));
    var throughAudit = machinekit.design.AssemblyDesignChecks.check(joint);
    throughAudit.throwIfErrors();
    assertions++;
    joint.replaceComponent(
      "screw",
      machinekit.standard.SocketHeadCapScrew.metric("M12", 10, machinekit.standard.PropertyClass.Steel88)
    );
    var shortAudit = machinekit.design.AssemblyDesignChecks.check(joint);
    assertions++;
    if (shortAudit.items.filter(item -> item.subject == "through-fixing thread engagement").length
      != 1) throw "Through-hole fasteners must still enforce numeric engagement limits";
    rejected = false;
    try machinekit.structural.InsertHoles.records('{"version":2,"holes":[]}') catch (e:Dynamic) rejected =
      Std.string(e).indexOf("Unsupported insert hole recipe") >= 0;
    assertions++;
    if (!rejected) throw "Legacy insert hole encoding must be rejected";
    var cwd = Sys.getCwd(), root = (sys.FileSystem.exists(cwd + "/machinekit")
      ? cwd + "/machinekit/tests/tooling" : cwd) + "/fixtures/foundation-design";
    var frame = MillProject.smallFrame();
    columnCornerCorrection(frame);
    if (Sys.args().indexOf("--export-approved-mill") >= 0) {
      for (row in [{id: "mill-base", body: frame.base.body}, {id: "mill-column", body: frame.column.body}]) {
        var part = row.body.geometry();
        try part.shape.exportStep(root + "/" + row.id + "-simplified.step") catch (error:Dynamic) {
          part.close();
          throw error;
        }
        part.close();
      }
      return;
    }
    if (Sys.args().indexOf("--export-approved-hobber") >= 0) {
      var sources:Array<{id:String, part:machinekit.structural.MineralFoundation}> = [
        {id: "hobber-bed", part: new hobber.MineralCastBed()},
        {id: "hobber-column", part: new hobber.MineralCastColumn()}
      ];
      for (row in sources) {
        var part = row.part.body.geometry();
        try part.shape.exportStep(root + "/" + row.id + "-simplified.step") catch (error:Dynamic) {
          part.close();
          throw error;
        }
        part.close();
      }
      return;
    }
    var columnBolts = frame.components().filter(row -> StringTools.startsWith(row.id, "columnBolt"));
    assertions++;
    if (columnBolts.length != 4) throw "Mill needs its four column service screws";
    for (row in columnBolts) {
      var bolt:machinekit.standard.SocketHeadCapScrew = cast row.component;
      assertions++;
      if (bolt.length != 40 || bolt.propertyClass != machinekit.standard
        .PropertyClass.Steel88) throw "Mill column service screws must derive the 40 mm stack length and class 8.8";
    }
    var mold = machinekit.tooling.CastingMoldCad.plan(MillMoldSetup.document(frame), MillMoldSetup.options(frame));
    assertions++;
    if (mold.coreUnits.length != 10 || mold.coreUnits.filter(unit -> unit.coreIds.length == 1).length
      != 10) throw "Six directly authored underside pockets and four foot-access pockets need ten individual core units";
    var reference:Dynamic = haxe.Json.parse(sys.io.File.getContent(root + "/reference.json"));
    if (Sys.args().indexOf("--audit-openings") >= 0) for (source in [frame.base.body,
      frame.column.body]) for (feature in source.castingFeatures()) Sys.println(feature.id
      + " " + haxe.Json.stringify(feature.openings));
    var millOnly = Sys.args().indexOf("--mill-only") >= 0;
    if (millOnly && Sys.args().indexOf("--write-baseline-current")
      >= 0) throw "A focused mill check cannot overwrite the four-foundation baseline";
    var foundations:Array<{id:String, part:machinekit.structural.MineralFoundation}> = [
      {id: "mill-base", part: frame.base},
      {id: "mill-column", part: frame.column}
    ];
    if (!millOnly) {
      foundations.push({id: "hobber-bed", part: new hobber.MineralCastBed()});
      foundations.push({id: "hobber-column", part: new hobber.MineralCastColumn()});
    }
    var baseline:Dynamic = haxe.Json.parse(sys.io.File.getContent(root
      + "/../../../../../machines/flexspline-hobber/checks/evidence/BASELINE_FOUNDATIONS.json"));
    if (baseline.format != "machinekit.foundation-baseline"
      || baseline.version != 3) throw "Unsupported foundation migration baseline";
    var frozenRows:Array<Dynamic> = cast baseline.currentBeforeMigration;
    var currentRows:Array<Dynamic> = [];
    for (row in foundations) {
      disjointPockets(row.part.body, row.id);
      assertions++;
      if (row.part.body.castingFeatures().filter(feature -> feature.id.indexOf("-fragment-")
        >= 0).length != 0) throw "Foundation pockets must be directly authored at named design stations: " + row.id;
      var process = sys.io.Process.run("gzip", ["-dc", root + "/" + row.id + "-simplified.step.gz"]);
      var text = process.readStdout(), status = process.exitCode();
      process.close();
      if (status != 0) throw "Cannot decode frozen migration CAD fixture";
      var records:Array<Dynamic> = cast(StringTools.startsWith(row.id,
        "mill-") ? reference.approvedSimplification.files : reference.approvedHobberSimplification.files);
      var matches = records.filter(record -> record.id == row.id);
      assertions++;
      if (matches.length != 1
        || haxe.crypto.Sha256.encode(text) != matches[0].stepSha256 || haxe.io.Bytes.ofString(text).length
        != matches[0].stepBytes) throw "Frozen migration CAD fixture fingerprint mismatch: " + row.id;
      var previous = new Part(cadkit.Shape.importStepText(text)), actual = row.part.body.geometry();
      same(previous, actual, row.id + " approved source simplification");
      var sourceDatums = row.part.body.foundationDatumScheme();
      var sourceInput = machinekit.tooling.FoundationToolingAdapter.snapshot(
        row.part,
        sourceDatums == null ? row.id : sourceDatums.foundationId,
        1
      );
      var unsupported = row.part.body.finishedCavities().filter(cavity -> !CastingCavities.supportsBoxes(cavity));
      var boxSupported = row.part.body.collisionHullQualification.status
        == machinekit.structural.CastingCollisionQualification.CastingCollisionStatus.Supported;
      var boxes = boxSupported ? row.part.body.materialBoxes() : null;
      var half = row.id == "hobber-bed"
        ? hobber.HobberLayout.get().xNominal().footSpan / 2 : row.part.body.parameterNumber("width") / 2;
      currentRows.push({
        id: row.id,
        sourceFingerprint: materia.tooling.ToolingCodec.inputFingerprint(sourceInput),
        sourceCodecVersion: materia.tooling.ToolingCodec.VERSION,
        volumeMm3: actual.volume(),
        massKg: row.part.body.massProperties().mass,
        boxStatus: boxSupported ? "pass" : "unsupported",
        boxReason : row.part.body.collisionHullQualification.reason,
        unsupportedFeatures : [for (cavity in unsupported) cavity.featureId],
        minimumIyMm4 : boxes == null ? null : materia.casting.BoxAlgebra.minimumIy(boxes, -half, half)
      });
      previous.close();
      actual.close();
      if (StringTools.startsWith(row.id, "hobber-")) {
        var kind = row.id.substr(7), frozen = frozenRows.filter(before -> before.id == kind)[0];
        var boxes:Array<Dynamic> = cast frozen.cells;
        var previousCells:Array<materia.casting.Box3> = [for (box in boxes) {
          x:materia.casting.CastingRecordCodec.number(box,
            "x"), y:materia.casting.CastingRecordCodec.number(box, "y"), z:materia.casting.CastingRecordCodec.number(
              box,
              "z"
            ), width:materia.casting.CastingRecordCodec.number(
              box,
              "width"
            ), depth:materia.casting.CastingRecordCodec.number(
              box,
              "depth"
            ), height:materia.casting.CastingRecordCodec.number(
              box,
              "height"
            )
        }
        ];
        var half:Float = frozen.halfSpan;
        var historicalIy = materia.casting.BoxAlgebra.minimumIy(previousCells,
          -half, half), frozenIy:Float = frozen.minimumIy;
        assertions++;
        if (Math.abs(historicalIy - frozenIy)
          > frozenIy * 1e-10) throw "Shared algebra changes historical frozen section Iy: " + row.id;
        var boxFeatures = row.part.body.castingFeatures()
          .filter(feature -> materia.casting.CastingFeature.CastingGeometrySolids.isBox(feature.geometry.solid));
        var cells = materia.casting.BoxAlgebra.material(row.part.body.castingEnvelope(), boxFeatures);
        // The historic cells still prove the shared section algebra above.
        // Approved geometry intentionally differs from them. Prove current
        // box material against native booleans; the full source is separately
        // compared with its approved frozen STEP, including non-box hardware.
        var boxValues = row.part.body.values().copy().setToken("features",
          CastingPatterns.encode(boxFeatures)).setToken(
            "embeddedAnchors",
            "[]"
          );
        var nativeBoxes = new MineralCasting(boxValues).geometry();
        var neutralBoxes:Part;
        try neutralBoxes = Solids.building([],
          owned -> {for (cell in cells) owned.push(machinekit.tooling.ToolingBlock.placedBox(cell));
        return Solids.union(owned);
        }) catch (error:Dynamic) {
          nativeBoxes.close();
          throw error;
        }
        try same(nativeBoxes,
          neutralBoxes, row.id + " shared box material versus native booleans") catch (error:Dynamic) {
          nativeBoxes.close();
          neutralBoxes.close();
          throw error;
        }
        nativeBoxes.close();
        neutralBoxes.close();
        if (kind == "bed") socketCount(row.part);
        if (kind == "bed") {
          var message = "";
          try row.part.body.minimumSectionIy(-half, half) catch (error:Dynamic) message = Std.string(error);
          assertions++;
          if (message.indexOf("Unsupported non-box anchor material for inserts: ") != 0
            || row.part.body.embeddedAnchors().filter(anchor -> message.indexOf(anchor.owner) < 0).length
            > 0) throw "Final casting box-only stiffness must name its unsupported anchor owners: " + message;
        }
        var features = row.part.body.castingFeatures();
        if (kind == "bed") {
          assertions++;
          if (features.filter(feature -> feature.kind == InterfaceRecess
            && feature.owner != null && StringTools.startsWith(
              feature.owner,
              "footSeat"
            )).length != 0) throw "Compression feet must not leave cast plate registers";
        }
        assertions++;
        if (row.part.insertLocations().filter(insert -> insert.id == "driveSeat")[0].part.parameterToken("role")
          != "drive-mount") throw "Hobber drive seats require their explicit source role";
      }
    }
    if (Sys.args().indexOf("--write-baseline-current") >= 0) {
      Reflect.setField(baseline, "currentDesign", currentRows);
      Reflect.setField(
        baseline,
        "currentDesignBasis",
        "Exact native volumes; box-only sections are Unsupported for non-box socket recesses and embedded anchor hardware, never approximated."
      );
      sys.io.File.saveContent(
        root + "/../../../../../machines/flexspline-hobber/checks/evidence/BASELINE_FOUNDATIONS.json",
        haxe.Json.stringify(baseline, null, "  ") + "\n"
      );
    }
    Sys.println("FoundationDesignTests: " + assertions + " assertions passed");
  }
}
