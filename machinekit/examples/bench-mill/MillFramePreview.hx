import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;
import machinekit.design.DesignRequirement;
import machinekit.milling.MillFrameLayout;
import machinekit.milling.MillZAxis;
import machinekit.milling.MillZAxisLayout;
import machinekit.milling.MillZAxisIntegrity;
import machinekit.milling.MillXYStage;
import machinekit.milling.MillGuidedXYStage;
import machinekit.milling.MillXYStageIntegrity;
import machinekit.milling.MillXYStageLayout;
import machinekit.motion.LinearRail;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

/** Mechanical XYZ assembly on the casting.
 * The legacy BenchMill keeps its independent, working motion/control model.
 */
class MillFramePreview {
  public static function layout(reference:BenchMill, ?yTravel:machinekit.milling.MillYTravel):MillFrameLayout {
    var state:Map<String, AssemblyFrame> = reference.solvedPoses();
    var z:AssemblyFrame = state.get("railZLeft");
    var column:AssemblyFrame = state.get("column");
    var zRail:LinearRail = cast reference.components().filter(row -> row.id == "railZLeft")[0].component;
    var rise = stageRise(reference);
    if (yTravel == null) yTravel = new machinekit.milling.MillYTravel();
    var guide = machinekit.motion.LinearRailBlock.metric(BenchMill.RAIL);
    var spindle:AssemblyFrame = state.get("spindle");
    var centreY = spindle.y;
    if (centreY + yTravel.halfTravel + yTravel.tableDepth / 2 + yTravel.clearance
      > column.y - reference.column.depth / 2 - guide.spec
      .railHeight) throw "Y travel/table depth exceeds spindle reach: increase column-to-spindle clearance before increasing Y stroke";
    var railStart = yTravel.railStart(centreY, guide), railEnd = yTravel.railEnd(centreY, guide);
    return new MillFrameLayout(
      {
        yRailSpacing: 220,
        yRailStart: railStart,
        yRailLength: railEnd - railStart,
        columnFront: column.y - reference.column.depth / 2,
        columnHeight: reference.column.height + rise + 40 + spindleStackRise(reference),
        zRailSpacing: Math.max(180, reference.column.railSpacing),
        zRailStart: z.z - reference.base.height + 100,
        zRailLength: zRail.length + rise - 60 + spindleStackRise(reference)
      },
      {
        baseWidth: 520,
        columnFootWidth: 440,
        columnJointGapMm: MillCastingDetails.COLUMN_JOINT_GAP_MM,
        frontExtension: yTravel.frontExtension,
        footPockets: true,
        yTravel: yTravel
      }
    );
  }

  public static function spindleStackRise(reference:BenchMill):Float {
    var poses:Map<String, AssemblyFrame> = reference.solvedPoses();
    var head:AssemblyFrame = poses.get("head"), tool:AssemblyFrame = poses.get("tool");
    return machinekit.milling.Bt30Cartridge.SEAT + 35 + reference.tool.stickout - (head.z - tool.z);
  }

  public static function stageRise(reference:BenchMill):Float return MillXYStageLayout.stackHeight(BenchMill.RAIL)
    - (reference.tableTop - reference.base.height);

  public static function zLayout(
    reference:BenchMill,
    layout:MillFrameLayout,
    guideBlock:Null<machinekit.motion.LinearRailBlock> = null
  ):MillZAxisLayout {
    var poses:Map<String, AssemblyFrame> = reference.solvedPoses();
    var head:AssemblyFrame = poses.get("head"), spindle:AssemblyFrame = poses.get("spindle");
    return new MillZAxisLayout(
      layout,
      head.z + layout.railPlane - reference.base.height + stageRise(reference) + spindleStackRise(reference),
      reference.head.height,
      spindle.y,
      reference.specs[2].lower,
      BenchMill.RAIL,
      guideBlock,
      layout.railPlane + machinekit.motion.LinearRailBlock.metric(BenchMill.RAIL).spec.blockHeight
    );
  }

  /** Both the axis-only study and full concept use this same mechanical assembly. */
  public static function structure(frame:BenchMillFrame):MachineAssembly {
    if (frame.zAxis == null) throw "Z study needs a frame with drive interfaces";
    var axis = new MillZAxis(frame.zAxis), result = new MachineAssembly();
    result.include("frame", frame);
    result.include("zAxis", axis);
    var framePoses:Map<String, AssemblyFrame> = frame.solvedPoses();
    var plate = axis.zeroPose("fixedMount");
    result.addMemberConnector(
      "frame/column/railZLeft",
      "axisSeat",
      F.compose(F.inverse(framePoses.get("column/railZLeft")), plate)
    );
    result.addMemberConnector("zAxis/fixedMount", "frameSeat", F.identity());
    result.addMate("z-on-column", "fixed", "frame/column/railZLeft", "axisSeat", "zAxis/fixedMount", "frameSeat");
    machinekit.milling.MillRailFasteners.install(
      result,
      frame.column,
      "frame/column",
      frame.casting.anchorRule.screwLengthsMm
    );
    return result;
  }

  /** `spindle` defaults to the one the Z layout was sized for. */
  public static function concept(
    reference:BenchMill,
    frame:BenchMillFrame,
    ?spindle:machinekit.milling.MillSpindlePackage
  ):MachineAssembly {
    if (frame.xyAxis == null) throw "Full mill concept needs XY steel drive seats";
    return assemble(reference, frame, new MillXYStage(frame.xyAxis), spindle);
  }

  /** Both sizes use the same frame/Z/spindle and guided cross-slide assembly path. */
  public static function assemble(
    reference:BenchMill,
    frame:BenchMillFrame,
    xy:MillGuidedXYStage,
    ?spindle:machinekit.milling.MillSpindlePackage
  ):MachineAssembly {
    var result = structure(frame);
    attachSpindle(reference, result, frame.zAxis, spindle);
    var driven = Std.downcast(xy, machinekit.milling.MillXYStage);
    result.include("xyStage", driven == null ? xy : MillFrameCovers.stage(driven));
    var poses:Map<String, AssemblyFrame> = frame.solvedPoses();
    result.addMemberConnector(
      "frame/base/casting",
      "stageSeat",
      F.compose(F.inverse(poses.get("base/casting")), xy.zeroPose("railYLeft"))
    );
    result.addMemberConnector("xyStage/railYLeft", "frameSeat", F.identity());
    result.addMate("xy-on-base", "fixed", "frame/base/casting", "stageSeat", "xyStage/railYLeft", "frameSeat");
    machinekit.milling.MillRailFasteners.install(
      result,
      frame.base,
      "frame/base",
      frame.casting.anchorRule.screwLengthsMm
    );
    if (driven != null) MillFrameCovers.finish(result);
    return result;
  }

  /**
   * The spindle's service removals, swept with the whole mill by `AssemblyIntegrity.service`:
   * `motorWithdrawal`: belt, pulley, guard, and the motor's screws and nuts off, then the motor and its shaft up off the head;
   * `cartridgeWithdrawal`: the tool, belt, motor and its carrier, the draw-bar cylinder and its frame off, then the cartridge with its rotor,
   * draw-bar and belt pulley down out of the head until the pulley is below the head's underside (at the upper Z stop the table is 266 mm below).
   */
  static function declareSpindleServices(parts:MachineAssembly):Void {
    var ids = [for (m in parts.components()) m.id];
    function named(prefix:String):Array<String> return [for (id in ids) if (StringTools.startsWith(id, prefix)) id];
    // The motor's shaft and body length; a spindle drive that is not a servo motor (a study part) is taken out the same 220 mm.
    var motor = Std.downcast(parts.components().filter(m -> m.id == "motor")[0].component, machinekit.motion.ServoMotor);
    var lift = motor == null ? 220.0 : motor.rating.shaftLength + motor.rating.bodyLength;
    var drive = ["belt", "beltGuard", "motorPulley"].concat(named("guardBolt")).concat(named("motorBolt")).concat(named("motorNut"));
    parts.addDesignRequirement(ServiceRemoval("motorWithdrawal", ["motor", "motorShaft"],
      [{direction: [0.0, 0.0, 1.0], distance: lift}], drive, []));
    // The pulley plane is 252.5 mm above the gauge line (`MillSpindleHead`), the head's underside `Bt30Cartridge.SEAT` above it.
    parts.addDesignRequirement(ServiceRemoval("cartridgeWithdrawal",
      ["spindle", "rotor", "springs", "pullStud", "holder", "drawbar", "gripper", "spindlePulley"],
      [{direction: [0.0, 0.0, -1.0], distance: 252.5 + 15 + 3 - machinekit.milling.Bt30Cartridge.SEAT}],
      ["tool", "belt", "motorCarrier", "motor", "motorShaft", "motorPulley", "beltGuard", "reactionFrame", "cylinder", "pistonRod", "piston"]
        .concat(named("motorNut")).concat(named("motorBolt")).concat(named("guardBolt")).concat(named("carrierBolt")), []));
  }

  /** Complete shared spindle payload for both machine configurations. */
  public static function attachSpindle(
    reference:BenchMill,
    result:MachineAssembly,
    spindleLayout:MillZAxisLayout,
    ?selected:machinekit.milling.MillSpindlePackage
  ):Void {
    var spindle:machinekit.milling.MillSpindlePackage = selected ?? spindleLayout.spindle;
    var spindleParts = spindle.assembly(spindleLayout);
    var spindleOrigin = F.translation(0, spindleLayout.spindleY, spindleLayout.headBottom - spindle.seat());
    var tool = new machinekit.milling.FlatEndMill(new machinekit.component.ComponentValues().setNumber(
      "diameter",
      reference.tool.diameter
    ).setNumber(
      "fluteLength",
      reference.tool.fluteLength
    ).setNumber(
      "stickout",
      reference.tool.stickout
    ));
    spindleParts.addComponent(
      "tool",
      tool,
      F.translation(0, spindleLayout.spindleY, spindleOrigin.z - 35 - reference.tool.stickout)
    );
    spindleParts.addMemberConnector("holder", "toolSeat", F.translation(0, 0, -35 - reference.tool.stickout));
    spindleParts.addMemberConnector("tool", "seat", F.identity());
    spindleParts.addMate("tool-in-holder", "fixed", "holder", "toolSeat", "tool", "seat");
    declareSpindleServices(spindleParts);
    result.include("spindle", spindleParts);
    var axis = new MillZAxis(spindleLayout);
    result.addMemberConnector(
      "zAxis/head",
      "spindlePayload",
      F.compose(F.inverse(axis.zeroPose("head")), spindleOrigin)
    );
    result.addMemberConnector("spindle/spindle", "headSeat", F.identity());
    result.addMate("spindle-on-head", "fixed", "zAxis/head", "spindlePayload", "spindle/spindle", "headSeat");
  }

  static function writeServiceStudy(assembly:MachineAssembly, layout:MillZAxisLayout, directory:String):Void {

    machinekit.milling.MillSpindleIntegrity.checkPayload(assembly, layout).throwIfErrors();
    var poses:Map<String, AssemblyFrame> = assembly.solvedPoses(), cache = new machinekit.assembly.PosedParts();
    var parts:Array<materia.project.SceneArtifact.SceneArtifactPart> = [];
    try for (member in assembly.components()) if (member.id == "zAxis/head"
      || member.id == "zAxis/carriage" || member.id == "frame/column/casting" || StringTools.startsWith(
        member.id,
        "spindle/"
      )) parts.push(AssemblyPreview.part(
        member.id,
        member.component.description,
        cache.posed(member.component, poses.get(member.id)),
        member.component.materialId
      ));
    catch (e:Dynamic) {
      cache.close();
      throw e;
    }
    cache.close();
    // This static inspection scene carries no physics. CAD clearance is checked above and by the axis audits.
    var model = new cadkit.modeling.AssemblyModel("mm");
    for (part in parts) model.add(part.id);
    var definition = model.definition("spindle-service-review");
    for (component in definition.definitions) component.collisionHulls = [];
    sys.io.File.saveBytes(
      directory + "/mill-spindle-service-study.mtrg",
      materia.project.SceneArtifact.encode({
        lengthUnit: "mm",
        metresPerUnit: 0.001,
        parts: parts,
        assemblyDefinition: definition,
        assemblyState: model.initialState("spindle-service-review").record()
      })
    );
  }


  /** Same concept report schema for previews and same-build FEM reference packets. */
  public static function writeReport(
    assembly:MachineAssembly,
    frame:BenchMillFrame,
    directory:String,
    clearancesVerified:Bool,
    ?extraPose: {x: Float, y: Float, z: Float}
  ):Dynamic {
    var l = frame.layout, z:MillZAxisLayout = frame.zAxis, xy:MillXYStageLayout = frame.xyAxis;
    var report = {
      status: "mechanical XYZ concept; not qualified for manufacture",
      units: "mm",
      analysisReference: MillFemReference.capture(assembly, frame, extraPose),
      travel: [
        {axis: "x", travel: 2 * xy.xHalfTravel},
        {axis: "y", travel: 2 * xy.yHalfTravel},
        {axis: "z", travel: -z.lower}
      ],
      base: {width: l.baseWidth, depth: l.baseDepth, height: l.baseHeight, railPlane: l.railPlane},
      column: {
        width: l.columnWidth,
        depth: l.columnDepth,
        height: l.columnHeight,
        railFaceY: l.columnFront,
        footProfile: "stepped",
        footHeightMm: MillFrameLayout.COLUMN_FOOT_HEIGHT,
        clampAccess: "vertical side face"
      },
      xyFullStrokeStructuralClearance: clearancesVerified
      ? "passed conservative Cartesian-product CAD swept-volume audit" : "not rerun by the physical-reference writer",
      zFullStrokeStructuralClearance : clearancesVerified
      ? "passed conservative CAD swept-volume audit" : "not rerun by the physical-reference writer",
      xy : {
        tableTop : xy.tableTop,
        saddleHeight : MillXYStageLayout.SADDLE_HEIGHT,
        tableRibDepth : MillXYStageLayout.RISER_HEIGHT,
        xOvertravel : xy.xOvertravel,
        yOvertravel : xy.yOvertravel,
        yScrewZ : xy.yScrewZ,
        yMotorFace : xy.yMotorFace,
        yBayRoof : l.roof,
        yBayLocation : "lower column",
        yServiceRear : xy.yServiceRear
      },
      z : {screwX : 0, screwY : z.screwY, guideSpacing : l.zRailSpacing, lower : z.lower, upper : 0},
      castingMassKg : {base : frame.base.castingByWeight().kg, column : frame.column.castingByWeight().kg},
      pending : [
        "BT30 study axial interfaces, release force/stroke and supplier selection",
        "guide/block drilling, vendor mounting patterns and installed fasteners",
        "insert anchorage and strength analysis",
        "electrical brake and motor controls",
        "chip drainage and enclosure",
        "lifting and mold design",
        "material qualification and static/modal analysis"
      ]
    };
    sys.io.File.saveContent(directory + "/mill-frame-concept.json", haxe.Json.stringify(report, null, "  "));
    return report;
  }

  public static function main():Void {
    var reference = new BenchMill(false, false), l = layout(reference);
    var z = zLayout(reference,
      l), xy = new MillXYStageLayout(l, z.spindleY, BenchMill.RAIL), frame = BenchMillFrame.create(
        l,
        z,
        xy
      );
    MillZAxisIntegrity.check(new MillZAxis(z), frame).throwIfErrors();
    MillXYStageIntegrity.check(new MillXYStage(xy), frame).throwIfErrors();
    var root = Sys.getCwd(), directory = sys.FileSystem.exists(root + "/machinekit")
      ? root + "/machinekit/examples/bench-mill/build" : root + "/build";
    if (!sys.FileSystem.exists(directory)) sys.FileSystem.createDirectory(directory);
    for (entry in [
      {name: "mill-frame", assembly: cast(frame, MachineAssembly)},
      {name: "mill-z-axis", assembly: structure(frame)},
      {name: "mill-xy-stage", assembly: cast(new MillXYStage(xy), MachineAssembly)},
      {name: "mill-frame-concept", assembly: concept(reference, frame)}
    ]) {
      var path = directory + "/" + entry.name + ".mtrg";
      sys.io.File.saveBytes(
        path,
        materia.project.SceneArtifact.encode(AssemblyPreview.scene(entry.assembly, entry.name))
      );
      Sys.println('Mill frame preview: $path');
    }
    writeServiceStudy(concept(reference, frame), z, directory);

    MillDesignStudy.write(concept(reference, frame), frame, directory);
    writeReport(concept(reference, frame), frame, directory, true);
    Sys.println('Mill frame concept report: $directory/mill-frame-concept.json');
  }
}
