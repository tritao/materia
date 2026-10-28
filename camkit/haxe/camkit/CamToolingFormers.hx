package camkit;

import materia.tooling.casting.FormerConstruction;
import materia.casting.CastingSection.CastingSectionContour;
import materia.casting.CastingSection.CastingSectionBand;
import materia.casting.CastingProfiles;
import materia.casting.CastingProfileOffsets;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelPlan.PanelOperationKind;
import toolpathkit.geometry.Point3;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.tool.Tool;
import toolpathkit.tool.CutterProfile;
import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;

typedef FormerCamProgram = {
  var id:String;
  var program:ToolpathProgram;
}

/** Board former machining through the same contour conversion, compensated
 * profiles and island clearing as panels. There is no separate CAM engine.
 * Coordinates in the neutral record are mm; CamJob receives metres.
 */
class CamToolingFormers {
  public static function build(
    plan:FormerMachiningPlan,
    settings:CamToolingPanels.PanelCamSettings,
    ?measuredTools:Map<
      String,
      Tool
  >
  ):Array<FormerCamProgram> {
    var blankHeight = plan.stock.measuredThickness * 0.001, finishedHeight = plan.heightMm * 0.001;
    var declaredPlane = settings.spoilboardPlane;
    if (declaredPlane == null) throw "Former CAM needs a declared spoilboard plane: " + plan.id;
    var base:Float = declaredPlane;
    var top = base + blankHeight, height = base + finishedHeight;
    var breakthrough = settings.breakthroughAllowance;
    if (blankHeight < finishedHeight - 1e-10 || finishedHeight <= 0
      || plan.bands.length == 0 || plan.toleranceMm <= 0) throw "Invalid former machining stock/bands: " + plan.id;
    var rough = tool(plan.roughingCutter, 1, settings, measuredTools);
    var finish = tool(plan.cutter, plan.draftDegrees == 0 ? 1 : 2, settings, measuredTools);
    if (!settings.singleToolPrograms && (rough.length <= 0 || finish.length
      <= 0)) throw "Former programs need measured tool lengths or declared per-tool touch-off: " + plan.id;
    if (plan.roughingCutter.taperDegrees != 0 || Math.abs(plan.cutter.taperDegrees - plan.draftDegrees)
      > 1e-7) throw "Former cutter does not match the declared source draft: " + plan.id;
    for (cutter in [rough, finish]) {
      var flutes = cutter.fluteLength, stickOut = cutter.stickOut;
      if (flutes == null || stickOut == null || flutes < finishedHeight + breakthrough - 1e-10
        || stickOut < blankHeight + settings.approachClearance) throw "Former cutter reach is insufficient: " + plan.id;
    }
    var safe = top + settings.safeZ, margin = settings.blankMargin;
    var width = plan.widthMm * 0.001 + 2 * margin, depth = plan.depthMm * 0.001 + 2 * margin;
    var fits = (plan.stock.allowedRotations.indexOf(0) >= 0 && width * 1000 <= plan.stock.width
      && depth * 1000 <= plan.stock.height) || (plan.stock.allowedRotations.indexOf(90) >= 0
      && depth * 1000 <= plan.stock.width && width * 1000 <= plan.stock.height);
    if (!fits) throw "Former raw blank and machining margin exceed the declared sheet: " + plan.id;
    // Vertical error on a shallow drafted wall amplifies radial chord error.
    var contourTolerance = plan.draftDegrees == 0 ? settings.chordTolerance : Math.min(
      settings.chordTolerance,
      plan.toleranceMm * 0.001 * Math.tan(plan.draftDegrees * Math.PI / 180) / 20
    );
    var setup = new Setup(
      "1",
      settings.workOrigin,
      new SetupStock(
        0,
        width,
        0,
        depth,
        top,
        base,
        safe,
        settings.fixtures,
        base,
        breakthrough
      )
    );
    function job():CamJob return new CamJob(
      safe,
      settings.spindleRpm,
      new Point3(0, 0, safe),
      Math.min(
        settings.blendTolerance,
        contourTolerance
      ),
      Math.min(
        settings.arcTolerance,
        contourTolerance
      ),
      settings.spindleSpinUpSeconds,
      true,
      settings.cutDirection,
      settings.approachClearance
    );
    var roughing = job();
    if (top > height + 1e-10) roughing.surface(
      new Point3(0, 0, top),
      new Point3(width, depth, top),
      rough,
      height,
      settings.feed,
      rough.diameter * settings.stepOverFraction,
      settings.stepDown,
      settings.plungeFeed,
      materia.tooling.ToolingIds.derive(
        "",
        plan.id,
        "-facing"
      )
    );
    for (band in plan.bands) {
      if (band.topMm <= band.bottomMm
        || band.bottomMm < 0 || band.topMm > plan.heightMm + 1e-7) throw "Invalid former band depth: " + plan.id;
      for (row in band.contours) {
        var operation = PanelJoints.operation(
          "former-profile",
          row.depth % 2 == 0 ? OutsideProfile : InsideProfile,
          row.contour,
          band.topMm - band.bottomMm,
          tipDiameter(finish) * 1000
        );
        try materia.tooling.panel.PanelCodec.cutterFit(operation,
          []) catch (error:Dynamic) throw "Former " + plan.id + ": " + Std.string(error);
      }
      clear(roughing, plan, band, rough, settings, plan.finishingStockMm, width, depth, margin, base);
      if (plan.draftDegrees == 0) clear(roughing, plan, band, rough, settings, 0, width, depth, margin, base);
    }
    var result:Array<FormerCamProgram> = [{
      id: materia.tooling.ToolingIds.derive("", plan.id, "-rough"),
      program: roughing.finish(setup)
    }];
    if (plan.draftDegrees > 0) {
      if (plan.bands.length != 1) throw "Drafted former needs one source wall contour: " + plan.id;
      var finishing = job();
      for (row in plan.bands[0].contours) {
        var contour = CamCastingContours.fromProfile(
          row.contour,
          contourTolerance,
          false,
          0,
          0,
          0,
          margin,
          height,
          "Former " + plan.id
        );
        finishing.profile(
          contour,
          finish,
          base - breakthrough,
          settings.feed,
          row.depth % 2 == 0 ? "outside" : "inside",
          finishedHeight + breakthrough,
          settings.plungeFeed,
          materia.tooling.ToolingIds.derive(
            "",
            plan.id,
            "-draft-finish"
          ),
          height,
          breakthrough
        );
      }
      result.push({id: materia.tooling.ToolingIds.derive("", plan.id, "-finish"), program: finishing.finish(setup)});
    }
    return result;
  }

  static function tipDiameter(tool:Tool):Float return switch tool.profile().segments[0] {
    case Line(_, z0, r, z1, zone) if (z0 == 0 && z1 == 0 && zone == Cutting):
      2 * r;
    default:
      throw "Former contour finishing needs a flat cutting tip";
  };

  static function tool(
    spec:FormerCutter,
    number:Int,
    settings:CamToolingPanels.PanelCamSettings,
    measuredTools:Null<Map<
      String,
      Tool
  >>
  ):Tool {
    var radius = spec.tipDiameterMm * 0.0005, flute = spec.fluteLengthMm * 0.001;
    var outer = radius + flute * Math.tan(spec.taperDegrees * Math.PI / 180);
    if (measuredTools != null) {
      var measured = measuredTools.get(spec.id);
      if (measured == null) throw "Missing measured former cutter: " + spec.id;
      if (Math.abs(measured.nominalDiameter - 2 * outer) > settings.diameterMatchTolerance
        || Math.abs(tipDiameter(measured) - 2 * radius)
        > settings.diameterMatchTolerance) throw "Measured former cutter differs from recipe dimensions: " + spec.id;
      var angle = switch measured.profile().segments[1] {
        case Line(r0, z0, r1, z1, zone) if (z1 > z0 && zone == Cutting):
          Math.atan2(r1 - r0, z1 - z0) * 180 / Math.PI;
        default:
          throw "Measured former cutter is not a straight tapered edge: " + spec.id;
      };
      if (Math.abs(angle - spec.taperDegrees)
        > 1e-7) throw "Measured former cutter taper differs from recipe: " + spec.id;
      return measured;
    }
    var profile = spec.taperDegrees == 0 ? CutterProfile.flat(2 * radius, flute) : new CutterProfile([
      Line(
        0,
        0,
        radius,
        0,
        Cutting
      ),
      Line(
        radius,
        0,
        outer,
        flute,
        Cutting
      )
    ]);
    return new Tool(number, 0, 2 * outer, profile, flute, spec.stickOutMm * 0.001, 2 * outer);
  }

  static function clear(
    job:CamJob,
    plan:FormerMachiningPlan,
    band:CastingSectionBand,
    tool:Tool,
    settings:CamToolingPanels.PanelCamSettings,
    allowance:Float,
    width:Float,
    depth:Float,
    margin:Float,
    base:Float
  ):Void {
    var z = base + band.topMm * 0.001;
    var bottom = base + band.bottomMm * 0.001 - (band.bottomMm == 0 ? settings.breakthroughAllowance : 0);
    function contour(row:CastingSectionContour):CamContour {
      var profile = allowance == 0 ? row.contour : row.depth % 2 == 0 ? CastingProfileOffsets.outward(
        row.contour,
        allowance
      ) : CastingProfileOffsets.inward(
        row.contour,
        allowance
      );
      return CamCastingContours.fromProfile(
        profile,
        settings.chordTolerance,
        false,
        0,
        0,
        0,
        margin,
        z,
        "Former " + plan.id
      );
    }
    // Clearing reaches beyond the raw edges, so it leaves no fictitious
    // corner stock from treating the blank perimeter as a pocket wall.
    var padding = 2 * tool.diameter;
    var outer = CamCastingContours.fromProfile(
      CastingProfiles.rectangle(
        -padding * 1000,
        -padding * 1000,
        (width + 2 * padding) * 1000,
        (depth + 2 * padding) * 1000
      ),
      settings.chordTolerance,
      false,
      0,
      0,
      0,
      0,
      z
    );
    var islands = [for (row in band.contours) if (row.depth == 0) contour(row)];
    job.pocketContours(
      outer,
      islands,
      tool,
      bottom,
      settings.feed,
      tool.diameter * settings.stepOverFraction,
      settings.stepDown,
      settings.plungeFeed,
      materia.tooling.ToolingIds.derive(
        "",
        plan.id,
        "-outside"
      )
    );
    for (hole in band.contours) if (hole.depth % 2 == 1) {
      var children = [for (row in band.contours) if (row.depth == hole.depth + 1
        && materia.casting.CastingProfileGeometry.inside(
          hole.contour,
          row.contour[0].start.x,
          row.contour[0].start.y
        )) contour(row)];
      job.pocketContours(
        contour(hole),
        children,
        tool,
        bottom,
        settings.feed,
        tool.diameter * settings.stepOverFraction,
        settings.stepDown,
        settings.plungeFeed,
        materia.tooling.ToolingIds.derive(
          "",
          plan.id,
          "-inside"
        )
      );
    }
  }
}
