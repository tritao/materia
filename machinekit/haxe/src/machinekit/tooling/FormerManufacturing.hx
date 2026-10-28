package machinekit.tooling;

import cadkit.modeling.Part;
import materia.casting.CastingSection.CastingSectionBand;
import cadkit.modeling.Scope;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.ToolingRecipe;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.FormerConstruction;
import materia.tooling.casting.MoldRegions;

/** Declared board construction; the finished former remains the authoritative
 * cavity-shaping envelope used by the existing release/removal checks.
 */
class FormerManufacturing {
  /** A work frame is selected from the two board faces by exact accessibility,
	 * not by sampled tool poses. Every zero-draft band is proved to be a prism.
	 */
  public static function machining(
    id:String,
    region:CastingRegion,
    pose:materia.assembly.AssemblyRecord.AssemblyFrame,
    stock:materia.tooling.panel.PanelStockSpec,
    draftDegrees:Float,
    recipe:ToolingRecipe
  ):FormerMachiningPlan {
    var kernel = new NativeCastingMoldKernel();
    var bounds = kernel.bounds(MoldRegions.moved(region, F.inverse(pose)), id);
    pose = F.compose(pose, F.translation(bounds.x, bounds.y, bounds.z));
    var cutter = FormerConstruction.cutter(recipe.former, draftDegrees, id);
    var rough = FormerConstruction.cutter(recipe.former, 0, id);
    if (cutter == null
      || rough == null) throw "No declared former cutter for " + id + " / " + draftDegrees + " degrees";
    var work = MoldRegions.moved(region, F.inverse(pose));
    var bands:Array<CastingSectionBand>;
    if (draftDegrees > 0) {
      bands = [{topMm: bounds.height, bottomMm: 0, contours: kernel.section(work, F.identity(), id)}];
    } else {
      try bands = machiningBands(work, bounds.width, bounds.depth, bounds.height, id) catch (first:Dynamic) {
        var flip = F.fromRotationMatrix(bounds.width, 0, bounds.height, [-1, 0, 0, 0, 1, 0, 0, 0, -1]);
        pose = F.compose(pose, flip);
        work = MoldRegions.moved(region, F.inverse(pose));
        try bands = machiningBands(work,
          bounds.width, bounds.depth, bounds.height, id) catch (second:Dynamic) throw "Former "
          + id + " has no accessible single-face 2.5D setup: " + Std.string(first) + "; " + Std.string(second);
      }
    }
    return {
      id: id,
      stockToMold: pose,
      widthMm: bounds.width,
      depthMm: bounds.depth,
      heightMm: bounds.height,
      stock: stock,
      bands: bands,
      draftDegrees: draftDegrees,
      cutter: cutter,
      roughingCutter: rough,
      finishingStockMm: recipe.former.finishingStockMm,
      toleranceMm: recipe.former.toleranceMm
    };
  }

  static function machiningBands(region:CastingRegion,
    width:Float, depth:Float, height:Float, id:String):Array<CastingSectionBand> {
    var kernel = new NativeCastingMoldKernel(), bands = kernel.prismaticBands(region, width, depth, height, id);
    var previous:Null<CastingRegion> = null;
    for (band in bands) {
      var footprint = MoldRegions.section(band.contours, F.identity(), 1);
      if (previous != null && kernel.moments(Difference(previous,
        [footprint])).volumeMm3 > 1e-6) throw "Upper material blocks the cutter shaft at " + band.topMm + " mm";
      previous = footprint;
    }
    return bands;
  }

  public static function boardRegion(former:MoldFormer,
    member:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement):CastingRegion return Intersection([
      MoldRegions.tooling(former.volume),
      panelRegion(member)
    ]);

  public static function plan(former:MoldFormer,
    recipe:ToolingRecipe, ?resin:Part, ?castingEnvelope:CastingRegion):FormerConstructionPlan {
    var direction = former.withdrawal;
    // The closed end is machined from above; the open withdrawal end is Z0.
    var frame = CastingSectionContours.normalFrame(
      {x: -direction.x, y: -direction.y, z: -direction.z},
      former.volume.id
    );
    var region = MoldRegions.tooling(former.volume);
    var bounds = MoldRegions.exactBounds(MoldRegions.moved(region, F.inverse(frame)), former.volume.id);
    frame = F.compose(frame, F.translation(bounds.x, bounds.y, bounds.z));
    var result:FormerConstructionPlan = {
      stockToMold: frame,
      widthMm: bounds.width,
      depthMm: bounds.depth,
      heightMm: bounds.height,
      attachments: [],
      airRegions: [],
      sealingRegions: [],
      build: Unsupported("Construction has not been resolved")
    };
    var stock = FormerConstruction.solidStock(recipe.former.stocks, bounds.height, bounds.width, bounds.depth);
    if (stock != null) {
      var cutter = FormerConstruction.cutter(recipe.former, former.draftDegrees, former.volume.id);
      result.build = cutter == null ? Unsupported("No declared cutter matches source draft "
        + former.draftDegrees + " degrees") : cutter.fluteLengthMm + 1e-7
        < bounds.height ? Unsupported("Declared cutter cannot reach the former height") : Solid(
          stock,
          Math.max(
            0,
            stock.measuredThickness - bounds.height
          ),
          cutter
        );
      return result;
    }
    var thickest = 0.0;
    for (sheet in recipe.former.stocks) thickest = Math.max(thickest, sheet.measuredThickness);
    if (bounds.height <= thickest + 1e-7) {
      result.build = Unsupported("Former footprint does not fit any declared sufficient-thickness sheet");
      return result;
    }
    if (former.draftDegrees != 0) {
      result.build =
        Unsupported("Drafted built-up former requires leaning wall panels; that construction is not implemented");
      return result;
    }
    var bodyRegion = castingEnvelope != null && former.volume.profiles.length > 0 ? Intersection([
      region,
      castingEnvelope
    ]) : region;
    var bodyBounds = MoldRegions.exactBounds(MoldRegions.moved(bodyRegion, F.inverse(frame)), former.volume.id);
    var body:FormerBoxExtent = {
      pose: F.compose(frame, F.translation(bodyBounds.x, bodyBounds.y, bodyBounds.z)),
      widthMm: bodyBounds.width,
      depthMm: bodyBounds.depth,
      heightMm: bodyBounds.height
    };
    var stocks = recipe.former.stocks.copy();
    stocks.sort((a, b) -> Reflect.compare(a.measuredThickness, b.measuredThickness));
    var reasons:Array<String> = [];
    for (sheet in stocks) {
      try {
        var selected = materia.tooling.ToolingCodec.copyRecipe(recipe);
        selected.panel = sheet;
        var panels = materia.tooling.casting.MoldPanels.formerBox(
          former.volume.id,
          body.widthMm,
          body.depthMm,
          body.heightMm,
          selected
        );
        for (member in panels.members) {
          member.pose = F.compose(body.pose, member.pose);
          member.withdrawal = direction;
        }
        result.build = PanelBox(sheet, panels, body);
        result.attachments = [];
        result.airRegions = [];
        result.sealingRegions = [];
        if (castingEnvelope != null && former.volume.profiles.length > 0) attachments(
          former,
          result,
          bodyRegion,
          bodyBounds,
          recipe
        );
        boxEvidence(former, result, recipe.minimumLigament, resin);
        return result;
      } catch (error:Dynamic) reasons.push(Std.string(error));
    }
    result.build = Unsupported("No declared sheet can construct the panel box: " + reasons.join("; "));
    return result;
  }

  /** Neck and flange are removable sheet parts, not a laminated blank. */
  static function attachments(
    former:MoldFormer,
    construction:FormerConstructionPlan,
    bodyRegion:CastingRegion,
    body:materia.casting.Box3,
    recipe:ToolingRecipe
  ):Void {
    var source = MoldRegions.tooling(former.volume), frame = construction.stockToMold;
    var sheet = switch construction.build {
      case PanelBox(stock, _, _):
        stock;
      default:
        throw "Expected panel former";
    };
    var t = sheet.measuredThickness;
    var inner = materia.tooling.panel.PanelContours.roundedRectangle(
      body.x + t,
      body.y + t,
      body.width - 2 * t,
      body.depth - 2 * t,
      recipe.routerDiameter / 2
    );
    var air:CastingRegion = Placed({solid: Prism(inner, body.z), pose: frame});
    construction.airRegions.push(air);
    function add(id:String, region:CastingRegion):Void {
      var bounds = MoldRegions.exactBounds(MoldRegions.moved(region, F.inverse(frame)), id);
      var stock = FormerConstruction.solidStock(recipe.former.stocks, bounds.height, bounds.width, bounds.depth);
      if (stock == null) throw "No declared sheet fits former attachment " + id;
      construction.attachments.push({
        id: id,
        stock: stock,
        region: region,
        pose: F.compose(
          frame,
          F.translation(
            bounds.x,
            bounds.y,
            bounds.z
          )
        )
      });
    }
    var profiles:Array<CastingRegion> = [];
    for (index in 0...former.volume.profiles.length) {
      var profile = former.volume.profiles[index];
      var normal = F.transformVector(F.compose(F.inverse(frame), profile.pose), 0, 0, 1);
      if (Math.abs(Math.abs(normal.z) - 1)
        > 1e-7) throw "Former sheet attachment is transverse to withdrawal: " + former.volume.id;
      var region:CastingRegion = Intersection([source, MoldRegions.profile(profile)]);
      profiles.push(region);
      var bounds = MoldRegions.exactBounds(MoldRegions.moved(region, F.inverse(frame)), former.volume.id);
      var neckBox = MoldRegions.moved(
        MoldRegions.box({
          x: body.x,
          y: body.y,
          z: bounds.z,
          width: body.width,
          depth: body.depth,
          height: bounds.height
        }),
        frame
      );
      var board:CastingRegion = Difference(Intersection([region, neckBox]), [air]);
      add(materia.tooling.ToolingIds.derive("", former.volume.id, "-neck-" + index), board);
      construction.sealingRegions.push(Difference(region, [board, air]));
    }
    var flange:CastingRegion = Difference(source, [bodyRegion, Union(profiles), air]);
    add(materia.tooling.ToolingIds.derive("", former.volume.id, "-flange"), flange);
  }

  static function panelRegion(member:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement):CastingRegion {
    var t = member.panel.stock.measuredThickness;
    var outer = materia.tooling.panel.PanelContours.outside(member.panel);
    var body:CastingRegion = Placed({solid: Prism(outer, t), pose: member.pose});
    var cuts:Array<CastingRegion> = [];
    for (operation in member.panel.operations) if (operation.kind != OutsideProfile) {
      var z = operation.through
        || operation.side == materia.tooling.ToolingEnums.MachiningSide.NegativeZ ? 0 : t - operation.depth;
      cuts.push(Placed({
        solid: Prism(operation.contour, operation.depth),
        pose: F.compose(
          member.pose,
          F.translation(
            0,
            0,
            z
          )
        )
      }));
    }
    return Difference(body, cuts);
  }

  /** Exact reconstruction includes the open air volume and only the declared
	 * rabbet gaps as seal material. It cannot excuse an arbitrary missing board.
	 */
  public static function boxEvidence(
    former:MoldFormer,
    construction:FormerConstructionPlan,
    minimumWall:Float,
    ?resin:Part
  ): {
    differenceMm3: Float,
    minimumWallMm: Float,
    interior: CastingRegion,
    seals: CastingRegion
  } {
    var selected = switch construction.build {
      case PanelBox(stock, panels, body):
        {stock: stock, panels: panels, body: body};
      default:
        throw "Former is not a panel box";
    };
    var t = selected.stock.measuredThickness, frame = selected.body.pose;
    var envelope = MoldRegions.tooling(former.volume);
    var airBox = MoldRegions.moved(
      MoldRegions.box({
        x: t,
        y: t,
        z: 0,
        width: selected.body.widthMm - 2 * t,
        depth: selected.body.depthMm - 2 * t,
        height: selected.body.heightMm - t
      }),
      frame
    );
    airBox = Union([airBox].concat(construction.airRegions));
    var interior:CastingRegion = Intersection([envelope, airBox]);
    var boardUnion:CastingRegion = Union([for (member in selected
      .panels.members) panelRegion(member)].concat([for (attachment in construction.attachments) attachment.region]));
    var boards:CastingRegion = Intersection([envelope, boardUnion]);
    var rebateCuts:Array<CastingRegion> = [];
    for (member in selected.panels.members) for (operation in member.panel.operations) if (operation.kind
      == materia.tooling.panel.PanelPlan.PanelOperationKind.Rebate) {
      var z = operation.side == materia.tooling.ToolingEnums.MachiningSide.NegativeZ
        ? 0 : member.panel.stock.measuredThickness - operation.depth;
      rebateCuts.push(Placed({
        solid: Prism(operation.contour, operation.depth),
        pose: F.compose(
          member.pose,
          F.translation(
            0,
            0,
            z
          )
        )
      }));
    }
    var seals:CastingRegion = Intersection([
      envelope,
      Difference(
        Union(rebateCuts.concat(construction.sealingRegions)),
        [
          boardUnion,
          interior
        ]
      )
    ]);
    var scope = new Scope();
    try {
      var original = scope.own(ToolingVolume.geometryOf(former.volume));
      var air = scope.own(machinekit.structural.CastingRegionCad.part(airBox));
      var escaped = scope.own(air.subtract(original));
      if (resin == null && escaped.volume() > 1e-6) throw "Former interior breaks through its authored envelope";
      var local = scope.own(machinekit.assembly.PosedParts.moved(original, F.inverse(construction.stockToMold)));
      var localAir = scope.own(machinekit.assembly.PosedParts.moved(air, F.inverse(construction.stockToMold)));
      var minimum = resin == null ? Math.POSITIVE_INFINITY : air.shape.distance(resin.shape);
      for (index in 0...(resin == null ? local.shape.subshapeCount(CadKit.ShapeKind.Face) : 0)) {
        var face = scope.own(new cadkit.modeling.Model(local.shape.subshape(CadKit.ShapeKind.Face, index)));
        var bounds = face.shape.bounds();
        var opening = face.shape.surfaceKind() == CadKit.SurfaceKind.Plane && Math.abs(face.shape.faceNormal().get_z())
          > 1 - 1e-7 && Math.abs(bounds.get_min().get_z()) < 1e-6 && Math.abs(bounds.get_max().get_z()) < 1e-6;
        if (!opening) minimum = Math.min(minimum, face.shape.distance(localAir.shape));
      }
      if (minimum < minimumWall - 1e-7) throw "Former interior leaves only "
        + minimum + " mm to a closed face; requires " + minimumWall + " mm";
      var rebuilt = scope.own(machinekit.structural.CastingRegionCad.part(Union([boards, interior, seals])));
      var missing = scope.own(original.subtract(rebuilt)), excess = scope.own(rebuilt.subtract(original));
      var difference = missing.volume() + excess.volume();
      if (difference > 1e-6) throw "Former panel reconstruction differs by " + difference + " mm3";
      scope.close();
      return {differenceMm3: difference, minimumWallMm: minimum, interior: interior, seals: seals};
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }

  /** Actual board pieces are clipped to the finished envelope, preserving its
 * sealing neck, flanges and insert clearances. The remainder must be explained
 * by the declared interior and the existing rabbet sealing gaps.
 */
  public static function panelTargets(former:MoldFormer, construction:FormerConstructionPlan):Array<{
    id:String,
    pose:materia.assembly.AssemblyRecord.AssemblyFrame,
    part:Part
  }> {
    var panels = switch construction.build {
      case PanelBox(_, panels, _):
        panels;
      default:
        throw "Former is not a panel box";
    };
    var result:Array<{id:String, pose:materia.assembly.AssemblyRecord.AssemblyFrame, part:Part}> = [];
    var scope = new Scope();
    try {
      var envelope = scope.own(ToolingVolume.geometryOf(former.volume));
      for (member in panels.members) {
        var local = scope.own(ToolingPanel.fromPlan(member.panel).geometry());
        var placed = scope.own(machinekit.assembly.PosedParts.moved(local, member.pose));
        var clipped = scope.own(placed.intersect(envelope));
        if (clipped.volume() <= 1e-7) throw "Empty former board: " + member.panel.id;
        var target = machinekit.assembly.PosedParts.moved(clipped, F.inverse(member.pose));
        result.push({id: member.panel.id, pose: member.pose, part: target});
      }
      scope.close();
      return result;
    } catch (error:Dynamic) {
      for (item in result) item.part.close();
      scope.close();
      throw error;
    }
  }
}
