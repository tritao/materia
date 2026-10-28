package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.tooling.ToolingRecipe;
import materia.tooling.ToolingEnums.MachiningSide;
import materia.tooling.panel.PanelJoints;
import materia.tooling.panel.PanelContours;
import materia.tooling.panel.PanelCodec;
import materia.tooling.panel.PanelPlan;
import materia.tooling.panel.PanelAssemblyPlan.PanelPlacement;
import materia.tooling.casting.CoreSupport;
import materia.tooling.casting.CoreSupport.CoreSupportKind;
import materia.tooling.casting.CastingMoldPlan.CastingMoldCore;
import materia.tooling.casting.CastingMoldPlan.MoldBox;
import materia.tooling.casting.CastingMoldPlan.PenetrationPlan;

private typedef BridgeRegistration = {
  var receiver:PanelPlan;
  var bridge:PanelPlan;
  var receiverOrigin: {x: Float, y: Float};
  var bridgeOrigin: {x: Float, y: Float};
  var pin: {x: Float, y: Float};
  var bolt: {x: Float, y: Float};
  var secondary:Bool;
  var angle:Float;
  var receiverPinId:String;
  var bridgePinId:String;
  var receiverBoltId:String;
  var bridgeBoltId:String;
}

/** Two explicit support templates; neither establishes adhesive or structural qualification. */
class CoreSupports {
  public static function specification(kind:CoreSupportKind, r:ToolingRecipe):CoreSupport return {
    kind: kind,
    assemblyByLayer: false,
    breakoutAfterLift: false,
    adhesiveSpec: r.adhesiveSpec,
    adhesiveThickness: r.adhesiveLineThickness,
    fitClearance: r.jointClearance,
    embedDepth: 0,
    bondAreaMm2: 0,
    bridgeBondAreaMm2: 0,
    clampBearingAreaMm2: 0,
    socketOperationId: null,
    registrationProfiles: [],
    foot: null,
    panelIds: [],
    bondedPanelIds: [],
    hardware: [],
    padClosure: kind == Bridge ? materia.tooling.panel.PanelJoints.PanelClosure.ExternalClamps : null
  };

  public static function cutting(b:MoldBox,
    axis:materia.tooling.ToolingRecipe.CoreWireAxis):materia.tooling.casting.CoreSupport.CoreCutting {
    var v = switch axis {
      case X:
        {x: 1.0, y: 0.0, z: 0.0};
      case Y:
        {x: 0.0, y: 1.0, z: 0.0};
      case Z:
        {x: 0.0, y: 0.0, z: 1.0};
      default:
        throw "Invalid core wire axis";
    };
    return {
      solid: MoldRegions.box({x: 0, y: 0, z: 0, width: b.width, depth: b.depth, height: b.height}),
      block: {x: 0, y: 0, z: 0, width: b.width, depth: b.depth, height: b.height},
      stockToMold: F.translation(b.x, b.y, b.z),
      wireDirection: v,
      entryFace: {origin: {x: 0, y: 0, z: 0}, normal: {x: -v.x, y: -v.y, z: -v.z}},
      exitFace: {origin: {x: v.x * b.width, y: v.y * b.depth, z: v.z * b.height}, normal: {x: v.x, y: v.y, z: v.z}}
    };
  }

  /** A line/arc prism is ruled exactly along its extrusion. Orient that axis
	 * onto the declared stock wire axis; bounds describe only the blank. The
	 * caller selects stock and layers, without applying kerf compensation.
	 */
  public static function prismCutting(
    profile:Array<materia.casting.CastingProfile.CastingSegment>,
    depth:Float,
    coreToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
    axis:materia.tooling.ToolingRecipe.CoreWireAxis
  ):materia.tooling.casting.CoreSupport.CoreCutting {
    if (!Math.isFinite(depth) || depth <= 0) throw "Invalid core cutting depth";
    profile = materia.casting.CastingProfile.CastingProfileCodec.snapshot(profile);
    coreToMold = materia.casting.CastingRecordCodec.frame(coreToMold);
    var bounds = materia.casting.CastingProfileGeometry.bounds(profile);
    var dimensions:Array<Float>, rotation:Array<Float>;
    switch axis {
      case X:
        dimensions = [depth, bounds.width, bounds.height];
        rotation = [0, 1, 0, 0, 0, 1, 1, 0, 0];
      case Y:
        dimensions = [bounds.height, depth, bounds.width];
        rotation = [0, 0, 1, 1, 0, 0, 0, 1, 0];
      case Z:
        dimensions = [bounds.width, bounds.height, depth];
        rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1];
      default:
        throw "Invalid core wire axis";
    }
    var stockToCore = F.fromRotationMatrix(bounds.x, bounds.y, 0, rotation);
    var result = cutting({x: 0, y: 0, z: 0, width: dimensions[0], depth: dimensions[1], height: dimensions[2]}, axis);
    result.stockToMold = F.compose(coreToMold, stockToCore);
    result.solid = materia.casting.CastingRegion.Placed({
      solid: materia.casting.CastingFeature.CastingGeometrySolid.Prism(profile, depth),
      pose: F.inverse(stockToCore)
    });
    return result;
  }

  public static function validateCutting(
    actual:materia.tooling.casting.CoreSupport.CoreCutting,
    b:MoldBox,
    r:ToolingRecipe
  ):Void {
    if (materia.tooling.ToolingCodec.canonical(actual) != materia.tooling.ToolingCodec.canonical(cutting(
      b,
      r.coreWireAxis
    ))) throw "Invalid ruled core cutting definition";
  }

  public static function kind(cb:MoldBox, b:MoldBox):CoreSupportKind {
    var bottom = Math.abs(cb.z - b.z) < 1e-7, top = Math.abs(cb.z + cb.height - b.z - b.height) < 1e-7;
    if (bottom) return RoutedSocket;
    if (!top) throw "Bridge requires a cavity opening onto the fill face";
    return Bridge;
  }

  /** The existing routed-foot construction in the receiving panel's frame.
	 * +Z points into the cavity; the panel material is below Z0 in this frame.
	 */
  public static function socketFrame(panel:PanelPlacement):materia.assembly.AssemblyRecord.AssemblyFrame {
    return switch panel.panel.contactFace {
      case materia.tooling.ToolingEnums.ContactFace.PositiveZ:
        F.compose(panel.pose, F.translation(0, 0, panel.panel.stock.measuredThickness));
      case materia.tooling.ToolingEnums.ContactFace.NegativeZ:
        F.compose(panel.pose, F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]));
      default:
        throw "Routed core socket needs a panel contact face: " + panel.panel.id;
    };
  }

  static function movedBox(value:MoldBox,
    pose:materia.assembly.AssemblyRecord.AssemblyFrame):MoldBox return CastingMoldPlanner.box(
      value.width,
      value.depth,
      value.height,
      F.compose(pose, F.translation(value.x + value.width / 2, value.y + value.depth / 2, value.z))
    );

  public static function routedSocket(cavity:MoldBox, featureId:String, r:ToolingRecipe, panel:PanelPlacement): {
    core: CastingMoldCore,
    penetration: PenetrationPlan
  } {
    var frame = socketFrame(panel), local = movedBox(cavity, F.inverse(frame));
    if (Math.abs(local.z) > 1e-7) throw "EPS pocket does not open on receiver panel: " + featureId;
    var bounds = materia.casting.CastingProfileGeometry.bounds(PanelContours.outside(panel.panel));
    var positive = panel.panel.contactFace == materia.tooling.ToolingEnums.ContactFace.PositiveZ;
    var virtualPanel = PanelJoints.blank(
      panel.panel.id,
      bounds.width,
      bounds.height,
      r.panel,
      r.routerDiameter,
      r.minimumLigament
    );
    var virtualBed:PanelPlacement = {
      panel: virtualPanel,
      pose: F.translation(bounds.x, positive ? bounds.y : -bounds.y - bounds.height, -r.panel.measuredThickness),
      withdrawal : {x : 0, y : 0, z : -1}
    };
    var core = MoldCoreLayers.box(local, featureId, r, RoutedSocket), records:Array<PenetrationPlan> = [];
    build(core, local, r, virtualBed, [], records);
    if (records.length != 1) throw "Routed socket needs one sealed receiver: " + featureId;
    var transform = F.compose(F.inverse(panel.pose), F.compose(frame, virtualBed.pose));
    var ux = F.transformVector(transform, 1, 0, 0), uy = F.transformVector(transform, 0, 1, 0);
    var sign = ux.x * uy.y - ux.y * uy.x;
    for (operation in virtualPanel.operations) if (operation.id == core.support.socketOperationId) {
      var contour = [for (segment in operation.contour) {
        var a = F.transformPoint(transform, segment.start.x, segment.start.y, 0);
        var b = F.transformPoint(transform, segment.end.x, segment.end.y, 0), center = segment.center;
        var c = center == null ? null : F.transformPoint(transform, center.x, center.y, 0);
        {
          kind:segment.kind, start: {x: a.x, y: a.y}, end: {x: b.x, y: b.y}, center:c == null ? null : {
            x : c.x,
            y : c.y
          }, sweep : sign * segment.sweep
        };
      }
      ];
      if (sign < 0) {
        contour.reverse();
        contour = [for (segment in contour) {
          kind:segment.kind, start:segment.end, end:segment.start, center:segment.center, sweep:-segment.sweep
        }
        ];
      }
      panel.panel.operations.push(PanelJoints.operation(
        operation.id,
        operation.kind,
        contour,
        operation.depth,
        operation.toolDiameter,
        positive ? MachiningSide.PositiveZ : MachiningSide.NegativeZ,
        operation.through,
        operation.reliefToolDiameter
      ));
      panel.panel.contactOpenings.push(operation.id);
    }
    panel.panel = PanelCodec.copy(panel.panel);
    core.cavity = MoldRegions.moved(core.cavity, frame);
    core.foamEnvelope = MoldRegions.moved(core.foamEnvelope, frame);
    for (layer in core.layers) layer.stockToMold = F.compose(frame, layer.stockToMold);
    var foot = core.support.foot;
    if (foot == null) throw "Routed socket is missing its EPS foot: " + featureId;
    foot.foam = movedBox(foot.foam, frame);
    foot.cutting.stockToMold = F.compose(frame, foot.cutting.stockToMold);
    core.support.panelIds = [panel.panel.id];
    var record = records[0], seal = record.sealingFace;
    if (seal == null) throw "Routed socket is missing its sealing face: " + featureId;
    record.sealingFace = F.compose(frame, seal);
    return {core: core, penetration: record};
  }

  /** The receiver is sized from a certified contact rectangle, independently of
	 * the rest of a branched EPS core. Neither rectangle is a material surrogate.
	 */
  public static function socketFoot(
    c:CastingMoldCore,
    foam: {x: Float, y: Float, width: Float, depth: Float},
    cavityContact: {x: Float, y: Float, width: Float, depth: Float},
    baseZ:Float,
    contactZ:Float,
    r:ToolingRecipe,
    bed:PanelPlacement,
    penetrations:Array<PenetrationPlan>
  ):Void {
    var s = c.support, t = r.panel.measuredThickness,
      l = r.minimumLigament, tool = r.routerDiameter, g = r.adhesiveLineThickness, hw = r.hardware;
    if (s.kind != RoutedSocket) throw "Locating foot requires a routed-socket support: " + c.id;
    if (contactZ < baseZ - 1e-7) throw "EPS contact lies below its socket receiver: " + c.id;
    PanelJoints.validateClearance(
      r.jointClearance,
      r.panel.thicknessVariation,
      r.jointLiningAllowance,
      r.jointMachiningAllowance
    );
    s.panelIds = [bed.panel.id];
    if ((r.panel.measuredThickness / 3) > t - l
      || (r.panel.measuredThickness / 3) <= 2 * g) throw "Core socket leaves insufficient bed stock or glue space";
    var inset = tool + l, foot:MoldBox = {
      x: foam.x + inset,
      y: foam.y + inset,
      z: baseZ - (r.panel.measuredThickness / 3) + g,
      width: foam.width - 2 * inset,
      depth: foam.depth - 2 * inset,
      height: contactZ - baseZ + (r.panel.measuredThickness / 3) - 2 * g
    };
    if (foot.width <= tool || foot.depth <= tool) throw "Core too small for locating foot";
    s.foot = {foam: foot, cutting: cutting(foot, r.coreWireAxis)};
    s.embedDepth = (r.panel.measuredThickness / 3);
    s.bondAreaMm2 = foot.width * foot.depth;
    s.socketOperationId = materia.tooling.ToolingIds.derive("", c.id, "-socket");
    var op = PanelJoints.operation(
      s.socketOperationId,
      Pocket,
      PanelContours.relievedRectangle(
        foot.x + foot.width / 2 - bed.pose.x,
        foot.y + foot.depth / 2 - bed.pose.y,
        foot.width + r.jointClearance,
        foot.depth + r.jointClearance,
        tool / 2
      ),
      (r.panel.measuredThickness / 3),
      tool,
      MachiningSide.PositiveZ,
      false,
      tool
    );
    var bounds = materia.casting.CastingProfileGeometry.bounds(op.contour), cb = cavityContact;
    if (bounds.x + bed.pose.x < cb.x + l
      || bounds.y + bed.pose.y < cb.y + l || bounds.x + bounds.width + bed.pose.x > cb.x + cb.width - l
      || bounds.y + bounds.height + bed.pose.y > cb.y + cb.depth - l) throw "Core socket escapes sealing footprint";
    bed.panel.operations.push(op);
    bed.panel.contactOpenings.push(op.id);
    penetrations.push({
      panelId: bed.panel.id,
      operationId: op.id,
      kind: CoreSealed,
      insertId: null,
      coreId: c.id,
      sealMargin: l,
      sealMaterial: hw.sealMaterial,
      sealingFace: F.translation(cb.x + cb.width / 2, cb.y + cb.depth / 2, baseZ),
      plug: null
    });

  }

  public static function build(
    c:CastingMoldCore,
    b:MoldBox,
    r:ToolingRecipe,
    bed:PanelPlacement,
    panels:Array<PanelPlacement>,
    penetrations:Array<PenetrationPlan>
  ):Void {
    var foam = MoldCores.singleBox(c.foamEnvelope, c.id), cavity = MoldCores.singleBox(c.cavity, c.id);
    var s = c.support, t = r.panel.measuredThickness,
      l = r.minimumLigament, tool = r.routerDiameter, g = r.adhesiveLineThickness, hw = r.hardware;
    PanelJoints.validateClearance(
      r.jointClearance,
      r.panel.thicknessVariation,
      r.jointLiningAllowance,
      r.jointMachiningAllowance
    );
    if (s.kind == RoutedSocket) {
      socketFoot(
        c,
        {x: foam.x, y: foam.y, width: foam.width, depth: foam.depth},
        {x: cavity.x, y: cavity.y, width: cavity.width, depth: cavity.depth},
        b.z,
        b.z,
        r,
        bed,
        penetrations
      );
      return;
    }
    // The carrier is bonded to the foam; the bridge is bonded to the carrier.
    // Removable top pads register over the entire wall top in blind sockets.
    // Their locating pins and separate bolts sit outside the casting boundary.
    var edge = 2 * l + tool, padDepth = hw.padDepth, padWidth = b.width + 2 * t + 2 * edge, px = b.x - t - edge;
    var cx = foam.x + foam.width / 2, bw = Math.max(foam.width, hw.minimumBridgeWidth), bridgeX = cx - bw / 2;
    var southY = b.y - padDepth - edge,
      northY = b.y + b.depth, bridgeY = southY, bridgeDepth = b.depth + 2 * (padDepth + edge);
    var carrier = PanelJoints.blank(
      materia.tooling.ToolingIds.derive("", c.id, "-carrier"),
      foam.width,
      foam.depth,
      r.panel,
      tool,
      l
    );
    var bridge = PanelJoints.blank(
      materia.tooling.ToolingIds.derive("", c.id, "-bridge"),
      bw,
      bridgeDepth,
      r.panel,
      tool,
      l
    );
    function placed(p:PanelPlan, x:Float, y:Float, z:Float):Void {
      panels.push({panel: PanelCodec.copy(p), pose: F.translation(x, y, z), withdrawal: {x: 0, y: 0, z: 1}});
      s.panelIds.push(p.id);
    }
    for (north in [false, true]) {
      var id = materia.tooling.ToolingIds.derive("",
        c.id, north ? "-north-pad" : "-south-pad"), py = north ? northY : southY;
      var pad = PanelJoints.blank(id,
        padWidth, padDepth + edge, r.panel, tool, l), slotY = north ? t / 2 : padDepth + edge - t / 2;
      pad.labelY = north ? padDepth + edge - hw.labelOffset : hw.labelOffset;
      pad.operations.push(PanelJoints.operation(
        "wall-top-socket",
        Rebate,
        PanelContours.relievedRectangle(
          padWidth / 2,
          slotY,
          b.width + 2 * t + r.jointClearance,
          t + r.jointClearance,
          tool / 2
        ),
        t / 2,
        tool,
        MachiningSide.NegativeZ,
        false,
        tool
      ));
      var y = north ? b.y + b.depth + 2 * t : b.y - 2 * t, pinx = cx - bw / 4, boltx = cx + bw / 4;
      registration(
        {
          receiver: pad,
          bridge: bridge,
          receiverOrigin: {x: px, y: py},
          bridgeOrigin: {x: bridgeX, y: bridgeY},
          pin: {x: pinx, y: y},
          bolt: {x: boltx, y: y},
          secondary: north,
          angle: Math.PI / 2,
          receiverPinId: "locating-pin",
          bridgePinId: north ? "secondary-slot" : "primary-pin",
          receiverBoltId : "clamp-bolt",
          bridgeBoltId : north ? "north-bolt" : "south-bolt"
        },
        r,
        r.jointClearance
      );
      var prefix = materia.tooling.ToolingIds.derive("", c.id, north ? "-north" : "-south");
      supportHardware(
        s.hardware,
        materia.tooling.ToolingIds.derive("", prefix, "-pin"),
        materia.tooling.ToolingIds.derive("", prefix, "-bolt"),
        {x: pinx, y: y},
        {x: boltx, y: y},
        b.z + b.height,
        r,
        [pad.id, bridge.id]
      );
      placed(pad, px, py, b.z + b.height);
    }
    placed(carrier, foam.x, foam.y, b.z + b.height);
    placed(bridge, bridgeX, bridgeY, b.z + b.height + t);
    s.bondedPanelIds = [carrier.id, bridge.id];
    s.bondAreaMm2 = foam.width * foam.depth;
    s.bridgeBondAreaMm2 = s.bondAreaMm2;
  }

  public static function validateFoot(c:CastingMoldCore, r:ToolingRecipe, bed:MoldBox):Void {
    var foam = MoldCores.singleBox(c.foamEnvelope, c.id);
    var s = c.support, foot = s.foot;
    if (Math.abs(s.fitClearance - r.jointClearance) > 1e-7
      || Math.abs(s.adhesiveThickness - r.adhesiveLineThickness) > 1e-7) throw "Core support differs from recipe";
    if (s.kind != CoreSupportKind.RoutedSocket) return;
    if (foot == null) throw "Missing locating foot";
    if (Math.abs(s.embedDepth - r.panel.measuredThickness / 3) > 1e-7) throw "Locating foot does not match support";
    var f = foot.foam, b = foot.cutting.block, p = foot.cutting.stockToMold, v = foot.cutting.wireDirection;
    for (n in [
      f.x,
      f.y,
      f.z,
      f.width,
      f.depth,
      f.height,
      b.x,
      b.y,
      b.z,
      b.width,
      b.depth,
      b.height,
      p.x,
      p.y,
      p.z,
      p.qx,
      p.qy,
      p.qz,
      p.qw
    ]) if (!Math.isFinite(n)) throw "Invalid foot geometry";
    validateCutting(foot.cutting, f, r);
    var inset = r.routerDiameter + r.minimumLigament;
    if (f.width <= 0 || f.depth <= 0 || f.height <= 0 || f.width > r.core.width || f.depth > r.core.depth
      || f.height > r.core.height || Math.abs(f.x - foam.x - inset) + Math.abs(f.y - foam.y - inset)
      + Math.abs(f.width - foam.width + 2 * inset) + Math.abs(f.depth - foam.depth + 2 * inset) + Math.abs(f.z - bed.z
      + (r.panel.measuredThickness / 3) - r.adhesiveLineThickness) + Math.abs(f.height - (r.panel.measuredThickness
      / 3) + 2 * r.adhesiveLineThickness) > 1e-7) throw "Locating foot does not match support";
    if (b.x != 0 || b.y != 0
      || b.z != 0 || Math.abs(b.width - f.width) + Math.abs(b.depth - f.depth) + Math.abs(b.height - f.height) > 1e-7
      || Math.abs(p.x - f.x) + Math.abs(p.y - f.y) + Math.abs(p.z - f.z) + Math.abs(p.qx) + Math.abs(p.qy)
      + Math.abs(p.qz) > 1e-7 || Math.abs(Math.abs(p.qw) - 1) > 1e-8) throw "Invalid foot cutting definition";
  }

  /** Dry cross-carrier, supported on faced board pads over the wall tops.
	 * Pads use the existing former stock/machining route; external end clamps
	 * and board capacity retain the existing closure qualification requirement.
	 */
  public static function insertCarrier(
    p:CastingMoldPlan,
    insertId:String,
    face:materia.assembly.AssemblyRecord.AssemblyFrame,
    footprint:MoldBox
  ):PanelPlacement {
    var b = p.cavity, r = p.document.recipe, t = r.panel.measuredThickness, l = r.minimumLigament;
    if (face.z - p.document.setup.fillHeightMm
      < l - 1e-7) throw "Insert carrier has insufficient dry clearance above fill: " + insertId;
    var width = Math.max(r.hardware.minimumBridgeWidth, footprint.depth + 2 * l);
    if (width > b.depth) throw "Insert carrier exceeds the mold span: " + insertId;
    var y = Math.max(b.y, Math.min(b.y + b.depth - width, face.y - width / 2));
    if (footprint.y < y - 1e-7 || footprint.y + footprint.depth
      > y + width + 1e-7) throw "Insert carrier cannot cover the mounting face: " + insertId;
    var id = materia.tooling.ToolingIds.derive("fixture-carrier-", insertId);
    var left = b.x - t, right = b.x + b.width + t, center = b.x + b.width / 2;
    if (footprint.x > center + l) left = footprint.x - l;
    else if (footprint.x + footprint.width < center - l) right = footprint.x + footprint.width + l;
    var panel = PanelJoints.blank(id, right - left, width, r.panel, r.routerDiameter, l);
    panel.contactFace = materia.tooling.ToolingEnums.ContactFace.NegativeZ;
    panel.labelX = left > b.x ? right - left - 2 * r.hardware.labelOffset : 2 * r.hardware.labelOffset;
    panel.labelY = width / 2;
    var placed:PanelPlacement = {panel: panel, pose: F.translation(left, y, face.z), withdrawal: {x: 0, y: 0, z: 1}};
    var bridgeIds:Array<String> = [];
    for (core in p.cores) if (core.support.kind
      == Bridge) for (member in core.support.bondedPanelIds) if (bridgeIds.indexOf(member) < 0) bridgeIds.push(member);
    for (other in p.supportPanels) if (bridgeIds.indexOf(other.panel.id) >= 0) {
      var relative = F.compose(F.inverse(placed.pose), other.pose), normal = F.transformVector(relative, 0, 0, 1);
      if (Math.abs(normal.z - 1) > 1e-8) throw "Dry carrier requires a parallel core support: " + other.panel.id;
      if (relative.z >= t - 1e-7 || relative.z + other.panel.stock.measuredThickness <= 1e-7) continue;
      var outline = PanelContours.outside(other.panel), u = F.transformVector(relative, 1, 0, 0);
      var bounds = materia.casting.CastingProfileGeometry.bounds(PanelContours.placed(
        outline,
        Math.atan2(u.y, u.x),
        relative.x,
        relative.y
      ));
      var x0 = Math.max(0, bounds.x), x1 = Math.min(right - left, bounds.x + bounds.width);
      var y0 = Math.max(0, bounds.y), y1 = Math.min(width, bounds.y + bounds.height);
      if (x1 - x0 <= 1e-7 || y1 - y0 <= 1e-7) continue;
      if (outline.length != 4 || outline.filter(edge -> edge.center != null
        || Math.abs(edge.start.x - edge.end.x) > 1e-7 && Math.abs(edge.start.y - edge.end.y)
        > 1e-7).length > 0) throw "Dry carrier needs the exact non-rectangular support intersection: " + other.panel.id;
      if (relative.z <= r.jointClearance) throw "Core support crosses the source insert contact plane: "
        + insertId + "/" + other.panel.id;
      var clearance = r.jointClearance, cutWidth = x1 - x0 + clearance, cutHeight = y1 - y0 + clearance;
      var open = x0 <= 1e-7 || y0 <= 1e-7 || x1 >= right - left - 1e-7 || y1 >= width - 1e-7;
      var contour = PanelContours.relievedRectangle(
        (x0 + x1) / 2,
        (y0 + y1) / 2,
        cutWidth,
        cutHeight,
        r.routerDiameter / 2
      );
      panel.operations.push(PanelJoints.operation(
        materia.tooling.ToolingIds.derive("clear-", other.panel.id),
        open ? Rebate : Pocket,
        contour,
        t - relative.z + clearance,
        r.routerDiameter,
        MachiningSide.PositiveZ,
        false,
        r.routerDiameter
      ));
    }
    for (wall in p.panels.members.filter(member -> member.panel.id == "east" || member.panel.id == "west")) {
      if (wall.panel.id == "east"
        && right < b.x + b.width + t - 1e-7 || wall.panel.id == "west" && left > b.x - t + 1e-7) continue;
      var outline = materia.casting.CastingProfileGeometry.bounds(PanelContours.outside(wall.panel));
      var top = F.transformPoint(wall.pose, 0, outline.y + outline.height, 0).z, gap = face.z - top;
      if (gap < -1e-7) throw "Insert carrier intersects wall top: " + insertId + "/" + wall.panel.id;
      if (gap > 1e-7) {
        var padId = materia.tooling.ToolingIds.derive("", id, "-" + wall.panel.id + "-pad");
        p.formers.push({
          volume: {
            id: padId,
            material: FormerStock,
            cuts: [],
            regions: [],
            profiles: [],
            pieces: [{
              x: wall.panel.id == "east" ? b.x + b.width : b.x - t,
              y : y,
              z : top,
              width : t,
              depth : width,
              height : gap
            }]
          },
          featureIds : [],
          withdrawal : {x : 0, y : 0, z : 1},
          draftDegrees : 0
        });
      }
    }
    placed.panel = PanelCodec.copy(placed.panel);
    p.supportPanels.push(placed);
    return placed;
  }

  public static function sharedBridges(
    b:MoldBox,
    r:ToolingRecipe,
    cores:Array<materia.tooling.casting.CastingMoldPlan.CastingMoldCore>,
    panels:Array<PanelPlacement>,
    hardware:Array<materia.tooling.casting.CoreSupport.SupportHardware>,
    raised:MoldPanels.RaisedWalls = MoldPanels.RaisedWalls.EastWest
  ):Void {
    cores = cores.filter(core -> core.support.kind == Bridge);
    function seatBounds(core:CastingMoldCore):MoldBox {
      var rows = MoldCores.section(core.foamEnvelope, b.z + b.height - core.support.adhesiveThickness, core.id);
      var outers = rows.filter(row -> row.depth == 0);
      if (outers.length != 1) throw "Bridge needs one connected EPS contact: " + core.id;
      var footprint = materia.casting.CastingProfileGeometry.bounds(outers[0].contour), cover = core.coveringThickness;
      return {
        x: footprint.x - cover,
        y: footprint.y - cover,
        z: b.z + b.height,
        width: footprint.width + 2 * cover,
        depth: footprint.height + 2 * cover,
        height: r.panel.measuredThickness
      };
    }
    if (cores.length == 0) return;
    if (raised == MoldPanels.RaisedWalls.NorthSouth) {
      // Reuse one bridge construction in a rotated frame: the rails
      // always seat on the raised pair of walls, including their ends.
      var frame = F.fromRotationMatrix(0, 0, 0, [0, -1, 0, 1, 0, 0, 0, 0, 1]), inverse = F.inverse(frame);
      var localBox:CastingMoldPlan.MoldBox = {
        x: b.y,
        y: -b.x - b.width,
        z: b.z,
        width: b.depth,
        depth: b.width,
        height: b.height
      };
      var localCores:Array<CastingMoldCore> = [for (core in cores) {
        id:core.id,
          featureId:core.featureId, cavity:MoldRegions.moved(core.cavity, inverse), foamEnvelope:MoldRegions.moved(
            core.foamEnvelope,
            inverse
          ), layers:[for (layer in core.layers) {
          solid:layer.solid, block:layer.block, stockToMold:F.compose(inverse,
            layer.stockToMold), wireDirection:layer.wireDirection, entryFace:layer.entryFace, exitFace:layer.exitFace
        }
        ], coveringThickness:core.coveringThickness, support:core.support
      }
      ];
      var localPanels:Array<PanelPlacement> = [], localHardware:Array<SupportHardware> = [];
      sharedBridges(localBox, r, localCores, localPanels, localHardware, MoldPanels.RaisedWalls.EastWest);
      for (panel in localPanels) {
        for (operation in panel.panel.operations) if (operation.id == "wall-top-socket") {
          // North/south walls include both corner laps, unlike the
          // east/west walls. Their actual support span includes 2t.
          var edge = 2 * r.minimumLigament + r.routerDiameter, pad = r.hardware.padDepth;
          var right = panel.panel.id == "bridge-right-rail";
          operation.contour = PanelContours.relievedRectangle(
            right ? r.panel.measuredThickness / 2 : pad + edge - r.panel.measuredThickness / 2,
            (localBox.depth + 2 * r.panel.measuredThickness + 2 * edge) / 2,
            r.panel.measuredThickness + r.jointClearance,
            localBox.depth + 2 * r.panel.measuredThickness + r.jointClearance,
            r.routerDiameter / 2
          );
        }
        panel.panel = PanelCodec.copy(panel.panel);
        panel.pose = F.compose(frame, panel.pose);
        panel.withdrawal = F.transformVector(frame, panel.withdrawal.x, panel.withdrawal.y, panel.withdrawal.z);
        panels.push(panel);
      }
      for (item in localHardware) {
        item.pose = F.compose(frame, item.pose);
        hardware.push(item);
      }
      return;
    }
    var t = r.panel.measuredThickness,
      l = r.minimumLigament, edge = 2 * l + r.routerDiameter, pad = r.hardware.padDepth;
    // Overlapping beam bands are one physical row. A branched core is
    // supported at its actual pour-face contact, not its full-height bounds.
    var rows:Array<{bottom:Float, top:Float, memberIds:Array<String>}> = [];
    for (core in cores) {
      var bounds = seatBounds(core), y = bounds.y + bounds.depth / 2;
      var width = Math.max(r.hardware.minimumBridgeWidth, bounds.depth / 2);
      if (!Math.isFinite(width)
        || width < r.hardware.minimumBridgeWidth) throw "Core row cannot provide the minimum bridge width: " + core.id;
      rows.push({bottom: y - width / 2, top: y + width / 2, memberIds: [core.id]});
    }
    rows.sort((a, b) -> a.bottom != b.bottom ? Reflect.compare(a.bottom, b.bottom) : Reflect.compare(a.top, b.top));
    var merged:Array<{bottom:Float, top:Float, memberIds:Array<String>}> = [];
    for (row in rows) {
      if (merged.length > 0 && row.bottom <= merged[merged.length - 1].top + 1e-7) {
        var previous = merged[merged.length - 1];
        previous.top = Math.max(previous.top, row.top);
        previous.memberIds = previous.memberIds.concat(row.memberIds);
      } else merged.push(row);
    }
    rows = merged;
    function place(panel:materia.tooling.panel.PanelPlan, x:Float, y:Float, z:Float):Void panels.push({
      panel: PanelCodec.copy(panel),
      pose: F.translation(x, y, z),
      withdrawal: {x: 0, y: 0, z: 1}
    });
    var rails = [];
    for (right in [false, true]) {
      var x = right ? b.x + b.width : b.x - pad - edge, y = b.y - t - edge;
      var rail = PanelJoints.blank(
        right ? "bridge-right-rail" : "bridge-left-rail",
        pad + edge,
        b.depth + 2 * t + 2 * edge,
        r.panel,
        r.routerDiameter,
        l
      );
      rail.labelX = right ? pad + edge - 2 * r.hardware.labelOffset : 2 * r.hardware.labelOffset;
      rail.operations.push(PanelJoints.operation(
        "wall-top-socket",
        Rebate,
        PanelContours.relievedRectangle(
          right ? t / 2 : pad + edge - t / 2,
          (b.depth + 2 * t + 2 * edge) / 2,
          t + r.jointClearance,
          b.depth + r.jointClearance,
          r.routerDiameter / 2
        ),
        t / 2,
        r.routerDiameter,
        MachiningSide.NegativeZ,
        false,
        r.routerDiameter
      ));
      rails.push({panel: rail, x: x, y: y, right: right});
    }
    for (index in 0...rows.length) {
      var row = rows[index], y = (row.bottom + row.top) / 2, width = row.top - row.bottom;
      if (width < r.hardware.minimumBridgeWidth - 1e-7) throw "Core row is below the minimum bridge width";
      var id = "core-bridge-" + index, beam = PanelJoints.blank(
        id,
        b.width + 2 * (pad + edge),
        width,
        r.panel,
        r.routerDiameter,
        l
      ), bx = b.x - pad - edge, by = y - width / 2;
      beam.labelY = width / 2;
      for (rail in rails) {
        var x = rail.right ? b.x + b.width + 2 * t : b.x - 2 * t,
          py = y - width / 4, byy = y + width / 4, d = r.hardware.locatingPinDiameter + r.jointClearance;
        registration(
          {
            receiver: rail.panel,
            bridge: beam,
            receiverOrigin: {x: rail.x, y: rail.y},
            bridgeOrigin: {x: bx, y: by},
            pin: {x: x, y: py},
            bolt: {x: x, y: byy},
            secondary: rail.right,
            angle: 0,
            receiverPinId: "pin-" + index,
            bridgePinId: rail.right ? "secondary-slot" : "primary-pin",
            receiverBoltId : "bolt-" + index,
            bridgeBoltId : rail.right ? "right-bolt" : "left-bolt"
          },
          r,
          r.jointClearance
        );
        supportHardware(
          hardware,
          id + (rail.right ? "-right-pin" : "-left-pin"),
          id + (rail.right ? "-right-bolt" : "-left-bolt"),
          {x : x, y : py},
          {x : x, y : byy},
          b.z + b.height,
          r,
          [rail.panel.id, beam.id]
        );
      }
      place(beam, bx, by, b.z + b.height + t);
      for (c in cores) {
        if (row.memberIds.indexOf(c.id) >= 0) {
          c.support.bondedPanelIds = [MoldCores.carrierId(c), id];
          var foam = MoldCores.foam(c), f = MoldCores.stockBounds(c.foamEnvelope, c.id);
          var contactZ = b.z + b.height - c.support.adhesiveThickness;
          c.support.bondAreaMm2 = MoldCores.area(MoldCores.section(foam, contactZ, c.id));
          // Intersect the actual foam with the beam band, then measure its
          // exact contact section; a curved pad never gets its blank's area.
          var band = MoldRegions.box({x: f.x, y: by, z: f.z, width: f.width, depth: width, height: f.height});
          c.support.bridgeBondAreaMm2 = MoldCores.area(MoldCores.section(
            materia.casting.CastingRegion.Intersection([foam, band]),
            contactZ,
            c.id
          ));
          panels.push(MoldCores.carrier(c, r, b.z + b.height));
        }
      }
    }
    for (rail in rails) place(rail.panel, rail.x, rail.y, b.z + b.height);
  }

  static function registration(s:BridgeRegistration, r:ToolingRecipe, clearance:Float):Void {
    var h = r.hardware, d = h.locatingPinDiameter + clearance, t = r.panel.measuredThickness;
    var pinX = s.pin.x - s.receiverOrigin.x, pinY = s.pin.y - s.receiverOrigin.y;
    s.receiver.operations.push(PanelJoints.operation(
      s.receiverPinId,
      Drill,
      PanelContours.circle(pinX, pinY, d / 2),
      t,
      d
    ));
    s.receiver.operations.push(PanelJoints.operation(
      s.receiverBoltId,
      Drill,
      PanelContours.circle(s.bolt.x - s.receiverOrigin.x,
      s.bolt.y - s.receiverOrigin.y, h.supportClearanceDiameter / 2),
      t,
      h.supportClearanceDiameter
    ));
    var x = s.pin.x - s.bridgeOrigin.x, y = s.pin.y - s.bridgeOrigin.y;
    var contour = s.secondary ? PanelContours.placed(
      PanelContours.obround(0, 0, d + h.secondaryPinTravel, d),
      s.angle,
      x,
      y
    ) : PanelContours.circle(
      x,
      y,
      d / 2
    );
    s.bridge.operations.push(PanelJoints.operation(
      s.bridgePinId,
      s.secondary ? InsideProfile : Drill,
      contour,
      t,
      s.secondary ? r.routerDiameter : d
    ));
    s.bridge.operations.push(PanelJoints.operation(
      s.bridgeBoltId,
      Drill,
      PanelContours.circle(s.bolt.x - s.bridgeOrigin.x, s.bolt.y - s.bridgeOrigin.y, h.supportClearanceDiameter / 2),
      t,
      h.supportClearanceDiameter
    ));
  }

  static function supportHardware(
    out:Array<SupportHardware>,
    pinId:String,
    boltId:String,
    pin: {x: Float, y: Float},
    bolt: {x: Float, y: Float},
    z:Float,
    r:ToolingRecipe,
    attachments:Array<String>
  ):Void {
    var t = r.panel.measuredThickness;
    out.push({
      id: pinId,
      kind: LocatingPin,
      attachmentIds: attachments.copy(),
      pose: F.translation(pin.x, pin.y, z),
      diameter: r.hardware.locatingPinDiameter,
      length: 2 * t
    });
    out.push({
      id: boltId,
      kind: ClampBolt,
      attachmentIds: attachments.copy(),
      pose: F.translation(bolt.x, bolt.y, z + 2 * t),
      diameter: r.hardware.supportScrewDiameter,
      length: 2 * t
    });
  }
}
