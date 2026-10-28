import machinekit.assembly.MachineAssembly;
import machinekit.assembly.PosedParts;
import machinekit.component.MachineKitComponents;
import machinekit.milling.MillFrameLayout;
import machinekit.structural.FoundationIntegrity;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Check real solid partitioning, solved mating datums and recipe persistence. */
class MillFrameChecks {
  static var assertions = 0;
  static function check(value:Bool, label:String):Void {
    assertions++;
    if (!value) throw label;
  }
  static function near(a:Float, b:Float, label:String):Void check(Math.abs(a - b) < 1e-6, '$label ($a vs $b)');
  static function rejects(action:Void -> Void, label:String):Void {
    var rejected = false;
    try action() catch (_:Dynamic) rejected = true;
    check(rejected, label);
  }
  public static function main():Void {
    var reference = new BenchMill(false, false), layout = MillFramePreview.layout(reference);
    var z = MillFramePreview.zLayout(reference, layout), xy = new machinekit.milling.MillXYStageLayout(
      layout,
      z.spindleY,
      BenchMill.RAIL
    );
    // Envelope regression: shorten the front without moving the spindle or column.
    near(layout.yRailStart, -270, "compact front rail datum");
    near(layout.yRailLength, 410, "rails sized from stroke and support envelopes");
    near(layout.baseDepth, 805, "base follows compact rail start");
    near(xy.yHalfTravel * 2, layout.yTravel.stroke, "shared Y stroke");
    check(xy.yOvertravel >= layout.yTravel.overtravel, "reserved Y overtravel survives shortening");
    for (stroke in [120.0, 160]) {
      var travel = new machinekit.milling.MillYTravel(stroke);
      var sized = MillFramePreview.layout(reference, travel);
      var stage = new machinekit.milling.MillXYStageLayout(sized, z.spindleY, BenchMill.RAIL);
      near(stage.tableDepth, stroke + 50, "table grows with working travel");
      near(stage.yHalfTravel * 2, stroke, "stage uses requested working travel");
      check(stage.yOvertravel >= travel.overtravel, "configured travel retains overrun");
      check(sized.baseDepth <= layout.baseDepth, "smaller stroke does not grow the base");
    }
    rejects(() -> new machinekit.milling.MillYTravel(Math.NaN), "reject nonfinite Y stroke");
    rejects(() -> new machinekit.milling.MillYTravel(0), "reject zero Y stroke");
    rejects(
      () -> MillFramePreview.layout(reference, new machinekit.milling.MillYTravel(240)),
      "larger travel requires more spindle reach, not just longer rails"
    );
    // A longer-reach variant can use the same sizing rule for a larger stroke.
    var largerTravel = new machinekit.milling.MillYTravel(240), largerCentre = z.spindleY - 80;
    var guide = machinekit.motion.LinearRailBlock.metric(BenchMill.RAIL);
    var largerStart = largerTravel.railStart(largerCentre, guide);
    var largerFrame = new MillFrameLayout(
      {
        yRailSpacing: layout.yRailSpacing,
        yRailStart: largerStart,
        yRailLength: largerTravel.railEnd(
          largerCentre,
          guide
        ) - largerStart,
        columnFront: layout.columnFront,
        columnHeight: layout.columnHeight,
        zRailSpacing: layout.zRailSpacing,
        zRailStart: layout.zRailStart,
        zRailLength: layout.zRailLength
      },
      {
        yTravel: largerTravel,
        frontExtension: largerTravel.frontExtension
      }
    );
    var largerStage = new machinekit.milling.MillXYStageLayout(largerFrame, largerCentre, BenchMill.RAIL);
    near(largerStage.yHalfTravel * 2, 240, "larger travel fits with deliberately increased spindle reach");
    check(largerStage.yOvertravel >= largerTravel.overtravel, "larger travel retains overrun");
    check(largerFrame.baseDepth > layout.baseDepth, "larger working envelope grows the structure");
    var frame = BenchMillFrame.create(layout, z, xy);
    frame.validate();
    var sourceDesign = machinekit.design.AssemblyDesignChecks.check(frame);
    check(
      !sourceDesign.hasErrors(),
      "Source joint and threaded steel design: "
      + sourceDesign.items.map(item -> item.code + ": " + item.message).join("; ")
    );
    var poses = frame.solvedPoses();
    for (foundation in [frame.base, frame.column]) {
      check(!FoundationIntegrity.check(foundation).hasErrors(), "nominal insert planes");
      var lifting = foundation.insertLocations().filter(row -> row.part.parameterToken("role")
        == materia.casting.InsertSpec.InsertRole.LiftingMount);
      check(lifting.length == 4, "four declared source lifting connections");
      for (row in lifting) Sys.println("LIFT-SOURCE " + (foundation
        == frame.base ? "base/" : "column/") + row.id + " XYZ=" + row.pose.x + "," + row.pose.y + "," + row.pose.z);
      var designs = [for (row in lifting) materia
        .tooling.ToolingCodec.canonical(materia.casting.InsertSpec.InsertSpecCodec.record(row.part.insertSpec()))];
      check(
        designs.filter(design -> design != designs[0]).length == 0,
        "one lifting part per casting, without mirrored drilling patterns"
      );
      var diameters:Array<Float> = [];
      for (row in foundation.insertLocations()) {
        for (hole in row.part.insertSpec().holes) if (row.part.parameterToken("role")
          == materia.casting.InsertSpec.InsertRole.LiftingMount && hole.id == "fixing") {
          near(hole.frame.x, 0, "lifting service thread is centred in its blank X");
          near(hole.frame.y, 0, "lifting service thread is centred in its blank Y");
        }
        switch row.part.insertSpec().anchorage {
          case Bolted(_, anchors):
            for (anchor in anchors) diameters.push(anchor.screw.thread.majorDiameter);
          default:
        }
      }
      check(
        diameters.length > 0 && diameters.filter(diameter -> diameter != diameters[0]).length == 0,
        "one shop-declared anchor diameter per casting"
      );

      var body = foundation.body.geometry();
      check(body.valid() && body.solidCount() == 1, "casting remains one connected valid solid");
      var locations = foundation.solvedPoses();
      for (row in foundation.insertLocations()) {
        var local = row.part.geometry(), steel = PosedParts.moved(local, locations.get(row.id));
        var common = body.intersect(steel);
        near(common.volume(), 0, 'no double-counted steel ${row.id}');
        common.close();
        steel.close();
        local.close();
      }
      check(foundation.castingByWeight().kg > 0, "casting mass available");
      body.close();
    }
    for (side in ["Left", "Right"]) {
      var railY = F.compose(
        poses.get('base/railY$side'),
        frame.base.components().filter(m -> m.id == 'railY$side')[0].component.connector("top").frame
      );
      near(railY.z, layout.railPlane, "Y steel mounting height");
      var railZ = F.compose(
        poses.get('column/railZ$side'),
        frame.column.components().filter(m -> m.id == 'railZ$side')[0].component.connector("top").frame
      );
      near(railZ.y, layout.columnFront, "Z steel mounting face");
      var normal = F.transformVector(railZ, 0, 1, 0);
      near(normal.y, -1, "Z mounting normal");
      for (end in ["Front", "Rear"]) {
        var id = side + end;
        var lowerSteel = frame.base.components().filter(m -> m.id == 'column$id')[0].component;
        var upperSteel = frame.column.components().filter(m -> m.id == 'foot$id')[0].component;
        var lower = F.compose(poses.get('base/column$id'), lowerSteel.connector("top").frame);
        var upper = F.compose(poses.get('column/foot$id'), upperSteel.connector("top").frame);
        var lowerSpec = frame.base.insertLocations().filter(row -> row.id == 'column$id')[0].part.insertSpec();
        var upperSpec = frame.column.insertLocations().filter(row -> row.id == 'foot$id')[0].part.insertSpec();
        var lowerHole = lowerSpec.holes.filter(hole -> hole.id == "fixing")[0];
        var upperHole = upperSpec.holes.filter(hole -> hole.id == "fixing")[0];
        var lowerAxis = F.compose(poses.get('base/column$id'), lowerHole.frame);
        var upperAxis = F.compose(poses.get('column/foot$id'), upperHole.frame);
        near(lowerAxis.x, upperAxis.x, "column fixing axes X");
        near(lowerAxis.y, upperAxis.y, "column fixing axes Y");
        var a = F.transformVector(lowerAxis, 0, 0, 1), b = F.transformVector(upperAxis, 0, 0, 1);
        near(a.x * b.x + a.y * b.y + a.z * b.z, -1, "opposed fixing-hole entry axes");
        var clampPose:AssemblyFrame = poses.get('columnBolt$id');
        var farFace = F.transformPoint(upperAxis, 0, 0, upperHole.depth);
        near(clampPose.x, farFace.x, "clamp head uses fixing-hole exit X");
        near(clampPose.y, farFace.y, "clamp head uses fixing-hole exit Y");
        near(clampPose.z, farFace.z, "clamp head uses fixing-hole exit Z");
        near(upper.z - lower.z, layout.columnJointGapMm, "column steel pair declared grout gap");
        var plate = frame.base.components().filter(member -> member.id == 'foot$id')[0].component;
        var foot = F.compose(poses.get('base/foot$id'), plate.connector("top").frame);
        near(foot.z, layout.footPockets ? frame.casting.footRoof : 0, "foot steel plane");
      }
    }
    var insertState = new cadkit.modeling.AssemblyState(frame.definition());
    check(
      frame.column.insertLocations().filter(row -> StringTools.startsWith(row.id, "drive")).length == 0,
      "Z bearing regions are not extra physical inserts"
    );
    for (side in ["Left", "Right"]) {
      var id = 'railZ$side', member = frame.column.insertLocations().filter(row -> row.id == id)[0].part;
      var stock = materia.casting.InsertSpec.InsertBodyCodec.box(member.insertSpec().body, id);
      near(
        stock.depth,
        layout.zRailStart + layout.zRailLength + frame.casting.zStripBearingExtension,
        "Continuous strip spans lower anchor through upper anchor"
      );
      for (end in ["Lower", "Upper"]) {
        var seat = insertState.worldConnector('column/$id', 'bearing${end}Face');
        near(seat.y, layout.columnFront, "Bearing mounting face preserved");
        near(
          seat.z,
          layout.columnMountPlane + (end == "Lower" ? 10 : layout.zRailStart + layout.zRailLength + 10),
          "Bearing anchor height preserved"
        );
      }
    }
    check(
      frame.components().filter(m -> StringTools.startsWith(m.id, "levelingFoot")).length == 4,
      "four leveling feet installed"
    );
    for (side in ["Left", "Right"]) for (end in ["Front", "Rear"]) {
      var id = side + end;
      var steelLocal = frame.base.components().filter(m -> m.id == 'foot$id')[0].component.geometry();
      var footLocal = frame.components().filter(m -> m.id == 'levelingFoot$id')[0].component.geometry();
      var steel = PosedParts.moved(steelLocal, poses.get('base/foot$id')), foot = PosedParts.moved(
        footLocal,
        poses.get('levelingFoot$id')
      );
      var nutLocal = frame.components().filter(member -> member.id == 'footSupportNut$id')[0].component.geometry();
      var nutPose = poses.get('footSupportNut$id');
      var nut = PosedParts.moved(nutLocal, nutPose);
      near(steel.distance(nut), 0, "support nut bears on the plate underside");
      var pressed = PosedParts.moved(nutLocal, F.compose(F.translation(0, 0, 1e-3), nutPose));
      var bearing = steel.intersect(pressed);
      check(bearing.volume() > 1e-6, "upward foot reaction meets the steel plate");
      bearing.close();
      pressed.close();
      var lowered = PosedParts.moved(nutLocal, F.compose(F.translation(0, 0, -1e-3), nutPose));
      bearing = steel.intersect(lowered);
      near(bearing.volume(), 0, "support separates when the foot reaction reverses");
      bearing.close();
      lowered.close();
      var common = steel.intersect(foot);
      near(common.volume(), 0, "stem clears bored steel seat");
      common.close();
      var casting = frame.base.body.geometry(), body = PosedParts.moved(casting, poses.get("base/casting"));
      common = body.intersect(foot);
      near(common.volume(), 0, "foot clears access pocket and stem opening");
      common.close();
      near(body.distance(steel), 0, "bearing plate contacts the casting ceiling");
      var plateUp = PosedParts.moved(steelLocal, F.compose(F.translation(0, 0, 1e-3), poses.get('base/foot$id')));
      var ceilingContact = body.intersect(plateUp);
      check(ceilingContact.volume() > 1e-6, "upward plate reaction meets the casting ceiling");
      ceilingContact.close();
      plateUp.close();
      check(
        frame.casting.footPlateSide * Math.sqrt(2) < frame.casting.footAccessDepth,
        "floating bearing plate stays in the pocket at every rotation around the rod"
      );
      common = body.intersect(nut);
      near(common.volume(), 0, "support nut clears the dry access pocket");
      common.close();
      body.close();
      casting.close();
      steel.close();
      foot.close();
      nut.close();
      nutLocal.close();
      steelLocal.close();
      footLocal.close();
    }
    check(layout.columnFootWidth > layout.columnWidth, "column has a wider separate foot");
    for (side in ["Left", "Right"]) for (end in ["Front", "Rear"]) {
      var id = side + end;
      var bolt:AssemblyFrame = poses.get('columnBolt$id');
      var blank = frame.column.body.geometry(), casting = PosedParts.moved(blank, poses.get("column/casting"));
      blank.close();
      var side = bolt.x < 0 ? -1 : 1;
      var bit = cadkit.modeling.Part.cylinderSpan(6, bolt.z + 6, layout.columnMountPlane + 68, bolt.x, bolt.y);
      var handle = cadkit.modeling.Part.cylinderAlong(
        6,
        new cadkit.modeling.Vector(
          bolt.x,
          bolt.y,
          layout.columnMountPlane + 68
        ),
        new cadkit.modeling.Vector(
          side,
          0,
          0
        ),
        layout.columnFootWidth / 2 - Math.abs(bolt.x) + 60
      );
      for (access in [bit, handle]) {
        var common = casting.intersect(access);
        near(common.volume(), 0, "column bolt has side-entry L-key access");
        common.close();
        access.close();
      }
      var blocked = cadkit.modeling.Part.cylinderSpan(6, bolt.z + 12, layout.columnMountPlane + 180, bolt.x, bolt.y);
      var common = casting.intersect(blocked);
      check(common.volume() > 1, "column taper stays closed above the bolt");
      common.close();
      blocked.close();
      casting.close();
      near(bolt.z, layout.columnMountPlane + layout.insertThickness, "clamp head bears on steel sole");
      var screw:machinekit.standard.SocketHeadCapScrew = cast frame.components().filter(m -> m.id
        == 'columnBolt$id')[0].component;
      var tip:Float = bolt.z - screw.shankLength();
      var anchor:AssemblyFrame = poses.get('base/column$id');
      check(tip < layout.railPlane && tip > anchor.z, "actual clamp screw engages the declared embedded anchor");
    }
    var restored:MachineAssembly = MachineAssembly.decode(frame.encode(), MachineKitComponents.standard());
    restored.validate();
    check(restored.components().length == frame.components().length, "frame recipe roundtrip preserves parts");
    var restoredPoses:Map<String, AssemblyFrame> = restored.solvedPoses();
    var originalColumn = frame.column.body.geometry(),
      rebuiltColumn = restored.components().filter(m -> m.id == "column/casting")[0].component.geometry();
    near(originalColumn.volume(), rebuiltColumn.volume(), "tapered foot and access wells survive recipe roundtrip");
    originalColumn.close();
    rebuiltColumn.close();
    for (member in frame.components()) {
      var original:AssemblyFrame = poses.get(member.id);
      var rebuilt:AssemblyFrame = restoredPoses.get(member.id);
      near(rebuilt.x, original.x, "roundtrip X");
      near(rebuilt.y, original.y, "roundtrip Y");
      near(rebuilt.z, original.z, "roundtrip Z");
    }
    var study = MillFramePreview.concept(reference, frame);
    study.validate();
    var studyPoses:Map<String, AssemblyFrame> = study.solvedPoses();
    var referencePoses:Map<String, AssemblyFrame> = reference.solvedPoses();
    check(
      study.components().filter(m -> m.id == "reference/base" || m.id == "reference/column").length == 0,
      "old structural bodies omitted"
    );
    for (id in ["railYLeft", "saddle", "table", "screwX", "screwY"]) check(
      studyPoses.exists('xyStage/$id'),
      'mechanical XY part $id included'
    );
    var table:AssemblyFrame = studyPoses.get("xyStage/table");
    near(table.z, xy.tableBottom, "table follows new stack datum");
    near(table.y, z.spindleY, "table centred beneath spindle");
    var interfaces:MillFrameInterfaces = {
      yRailSpacing: layout.yRailSpacing,
      yRailStart: layout.yRailStart,
      yRailLength: layout.yRailLength,
      columnFront: layout.columnFront,
      columnHeight: layout.columnHeight,
      zRailSpacing: layout.zRailSpacing,
      zRailStart: layout.zRailStart,
      zRailLength: layout.zRailLength
    };
    var variant = BenchMillFrame.create(new MillFrameLayout(
      interfaces,
      {
        baseWidth: 620,
        baseHeight: 260,
        columnWidth: 360,
        columnDepth: 300
      }
    ));
    variant.validate();
    var variantPoses:Map<String, AssemblyFrame> = variant.solvedPoses();
    var variantColumn:AssemblyFrame = variantPoses.get("column/casting");
    near(variantColumn.z, 260, "changed base height moves the column interface");
    near(variantColumn.y - 150, layout.columnFront, "deeper column preserves the guide datum");
    for (foundation in [variant.base, variant.column]) {
      var shape = foundation.body.geometry();
      check(shape.valid() && shape.solidCount() == 1, "resized frame stays connected");
      shape.close();
    }
    rejects(() -> new MillFrameLayout(interfaces, {baseWidth: 100}), "reject unsupported guide/column layout");
    rejects(() -> new MillFrameLayout(interfaces, {columnJointGapMm: 20}), "reject excessive column joint gap");
    rejects(() -> new MillFrameLayout(interfaces, {wall: Math.NaN}), "reject nonfinite geometry");
    rejects(() -> new MillFrameLayout(interfaces, {roof: 220}), "reject absent underside cavities");
    rejects(() -> new MillFrameLayout(interfaces, {columnDepth: 120}), "reject overlapping column sole pads");
    Sys.println('Mill frame: $assertions assertions passed');
  }
}
