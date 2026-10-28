package materia.tooling.casting;

import materia.assembly.AssemblyFrames as F;
import materia.casting.Box3;
import materia.casting.BoxAlgebra;
import materia.casting.FixtureConstraints;
import materia.casting.FixtureConstraints.FixtureConstraint;
import materia.casting.FixtureConstraints.FixtureConstraintReport;
import materia.casting.FixtureConstraints.FixtureVector;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.CastingToolingInput.CastingToolingInsert;
import materia.tooling.ToolingValidation;

typedef DatumLocationPlan = {
  var contacts:Array<FixtureConstraint>;
  var origin:FixtureVector;
  var referenceLengthMm:Float;
  var report:FixtureConstraintReport;
  var unsupportedReasons:Array<String>;
}

typedef DatumLocationInput = {
  var mountingFace:materia.assembly.AssemblyRecord.AssemblyFrame;
  var datum:MoldVolume;
  var fixings:Array<InsertFixtureFixing>;
}

private enum ContactFootprint {
  Rectangle(bounds:Box3);
  Disk(x:Float, y:Float, radius:Float);
  Unsupported(reason:String);
}

private typedef ContactBasis = {
  var frame:materia.assembly.AssemblyRecord.AssemblyFrame;
  var planeZ:Float;
  var footprint:ContactFootprint;
  var referenceLengthMm:Float;
}

/** Checks insert-to-datum location. Bed pins locate the datum, never the insert.
 * Rectangular subsets outside conservative hole/cut bounds certify contact witnesses;
 * they do not approximate a drilled solid or infer a press fit from a clearance screw.
 */
class DatumFixtureLocation {
  static inline var EPS = 1e-7;
  static inline var RANK_TOLERANCE = 1e-9;

  public static function plan(
    source:CastingToolingInsert,
    sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
    fixture:DatumLocationInput
  ):DatumLocationPlan {
    return contactPlan(
      source,
      sourceToMold,
      fixture.mountingFace,
      fixture.datum.id,
      fixture.datum.pieces,
      fixture.datum.cuts,
      fixture.datum.profiles.length > 0,
      fixture.fixings
    );
  }

  /** Contact witnesses stay inside a rectangle proved in the finished panel
	 * outline. Cut bounds exclude any witness that a real operation might remove.
	 * The support bounding prism is only a witness-search domain, not panel CAD.
	 */
  public static function onPanel(
    source:CastingToolingInsert,
    sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
    face:materia.assembly.AssemblyRecord.AssemblyFrame,
    panel:materia.tooling.panel.PanelAssemblyPlan.PanelPlacement,
    fixings:Array<InsertFixtureFixing>
  ):DatumLocationPlan {
    var thickness = panel.panel.stock.measuredThickness;
    var surface = DatumFixtures.contactSurface(panel, face);
    if (surface == null) throw "Insert face is not on its panel contact plane: " + source.id;
    var basis = contactBasis(source, sourceToMold, face), inverse = F.inverse(panel.pose);
    var outline = materia.tooling.panel.PanelContours.outside(panel.panel);
    var footprint = footprintBounds(basis.footprint);
    if (footprint != null) {
      var xs:Array<Float> = [], ys:Array<Float> = [];
      for (x in [footprint.x, footprint.x + footprint.width]) for (y in [footprint.y, footprint.y + footprint.depth]) {
        var world = F.transformPoint(basis.frame, x, y, basis.planeZ);
        var local = F.transformPoint(inverse, world.x, world.y, world.z);
        xs.push(local.x);
        ys.push(local.y);
      }
      xs.sort(Reflect.compare);
      ys.sort(Reflect.compare);
      if (!materia.casting.CastingProfileGeometry.containsRect(outline, xs[0], ys[0],
        xs[3] - xs[0], ys[3] - ys[0])) throw "Insert contact footprint is outside its receiver panel: " + source.id;
    }
    var bounds = materia.casting.CastingProfileGeometry.bounds(outline);
    var piece = materia.tooling.casting.CastingMoldPlanner.box(
      bounds.width,
      bounds.height,
      thickness,
      F.compose(panel.pose, F.translation(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2, 0))
    );
    var cuts:Array<MoldVolumeCut> = [];
    for (operation in panel.panel.operations) if (operation.kind
      != materia.tooling.panel.PanelPlan.PanelOperationKind.OutsideProfile) {
      var z = operation.side
        == materia.tooling.ToolingEnums.MachiningSide.PositiveZ ? thickness - operation.depth : 0.0;
      cuts.push({operation: operation, pose: F.compose(panel.pose, F.translation(0, 0, z))});
    }
    return contactPlan(source, sourceToMold, face, panel.panel.id, [piece], cuts, false, fixings);
  }

  static function contactBasis(
    source:CastingToolingInsert,
    sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
    face:materia.assembly.AssemblyRecord.AssemblyFrame
  ):ContactBasis {
    var normal = F.transformVector(face, 0, 0, 1);
    var downward = Math.abs(normal.x) < EPS && Math.abs(normal.y) < EPS && Math.abs(normal.z + 1) < EPS;
    var frame = downward ? F.identity() : F.compose(face, F.fromRotationMatrix(0, 0, 0, [1, 0, 0, 0, -1, 0, 0, 0, -1]));
    var stock = F.compose(sourceToMold,
      source.pose), mouth = materia.casting.InsertSpec.InsertBodyCodec.mouth(source.body);
    var scale = switch mouth {
      case Box(width, depth, height):
        Math.max(width, Math.max(depth, height));
      case Cylinder(diameter, height):
        Math.max(diameter, height);
    };
    var footprint:ContactFootprint;
    switch mouth {
      case Box(width, depth, height):
        try {
          var bounds = materia.tooling.casting.CastingMoldPlanner.box(
            width,
            depth,
            height,
            F.compose(F.inverse(frame), stock)
          );
          footprint = ContactFootprint.Rectangle({
            x: bounds.x,
            y: bounds.y,
            z: 0,
            width: bounds.width,
            depth: bounds.depth,
            height: 1
          });
        } catch (error:Dynamic) {
          if (Std.string(error).indexOf("axis-aligned") < 0) throw error;
          footprint = ContactFootprint.Unsupported("Oblique steel contact needs an exact finite-face contact model");
        }
      case Cylinder(diameter, _):
        var center = F.transformPoint(F.inverse(frame), face.x, face.y, face.z);
        footprint = ContactFootprint.Disk(center.x, center.y, diameter / 2);
    };
    return {frame: frame, planeZ: downward ? face.z : 0, footprint : footprint, referenceLengthMm : scale};
  }

  /** A search bound is never credited as circular steel. */
  static function footprintBounds(footprint:ContactFootprint):Null<Box3> return switch footprint {
    case ContactFootprint.Rectangle(bounds):
      bounds;
    case ContactFootprint.Disk(x, y, radius):
      {x: x - radius, y: y - radius, z: 0, width: 2 * radius, depth: 2 * radius, height: 1};
    case ContactFootprint.Unsupported(_):
      null;
  };

  /** Certify a finite rectangle in the disk analytically, including thin annuli.
	 * The closest point has positive radial margin m. Moving inward by at most
	 * sqrt(2)*m/(4sqrt(2)), then to a rectangle corner by the same bound, leaves
	 * at least m/2 of margin. No bounding-square corner becomes a steel contact.
	 */
  static function certifiedPatch(footprint:ContactFootprint, patch:Box3):Null<Box3> {
    switch footprint {
      case ContactFootprint.Rectangle(_):
        return patch;
      case ContactFootprint.Unsupported(_):
        return null;
      case ContactFootprint.Disk(x, y, radius):
        var nearestX = Math.max(patch.x, Math.min(x, patch.x + patch.width));
        var nearestY = Math.max(patch.y, Math.min(y, patch.y + patch.depth));
        var dx = nearestX - x, dy = nearestY - y, margin = radius - Math.sqrt(dx * dx + dy * dy);
        if (margin <= 0) return null;
        var inset = Math.min(Math.min(patch.width / 4, patch.depth / 4), margin / (4 * Math.sqrt(2)));
        var centerX = Math.max(patch.x + inset, Math.min(nearestX, patch.x + patch.width - inset));
        var centerY = Math.max(patch.y + inset, Math.min(nearestY, patch.y + patch.depth - inset));
        return {x: centerX - inset, y: centerY - inset, z: 0, width: 2 * inset, depth: 2 * inset, height: 1};
    }
  }

  static function contactPlan(
    source:CastingToolingInsert,
    sourceToMold:materia.assembly.AssemblyRecord.AssemblyFrame,
    face:materia.assembly.AssemblyRecord.AssemblyFrame,
    datumId:String,
    pieces:Array<CastingMoldPlan.MoldBox>,
    cuts:Array<MoldVolumeCut>,
    hasProfiles:Bool,
    fixings:Array<InsertFixtureFixing>
  ):DatumLocationPlan {
    var pose = F.compose(sourceToMold, source.pose), basis = contactBasis(source, sourceToMold, face);
    var inverse = F.inverse(basis.frame), localPose = F.compose(inverse, pose), plane = basis.planeZ;
    var origin:FixtureVector = {x: face.x, y: face.y, z: face.z};
    var scale = basis.referenceLengthMm;
    var contacts:Array<FixtureConstraint> = [], reasons:Array<String> = [], patches:Array<Box3> = [];
    switch basis.footprint {
      case ContactFootprint.Unsupported(reason):
        reasons.push(reason);
      case ContactFootprint.Rectangle(_), ContactFootprint.Disk(_, _, _):
    }
    if (hasProfiles) reasons.push("Profile-derived datum contact needs an exact finite-face contact model");
    if (reasons.length == 0) {
      var footprint:Box3 = cast footprintBounds(basis.footprint);
      for (world in pieces) {
        var piece = materia.tooling.casting.CastingMoldPlanner.box(
          world.width,
          world.depth,
          world.height,
          F.compose(inverse, F.translation(world.x + world.width / 2, world.y + world.depth / 2, world.z))
        );
        if (Math.abs(piece.z + piece.height - plane) <= EPS) {
          var patch = BoxAlgebra.intersection(
            footprint,
            {x: piece.x, y: piece.y, z: 0, width: piece.width, depth: piece.depth, height: 1}
          );
          if (patch != null) patches.push(patch);
        }
      }
      var excluded:Array<Box3> = [];
      for (hole in InsertFixturePlanner.holes(source)) {
        var entry = F.compose(localPose, hole.frame), axis = F.transformVector(entry, 0, 0, 1);
        var end = F.transformPoint(entry, 0, 0, hole.depth);
        var diameter = hole.thread == null ? hole.diameter : hole.thread.majorDiameter;
        if (hole.counterbore != null) diameter = Math.max(diameter, hole.counterbore.diameter);
        if (hole.relief != null) diameter = Math.max(diameter, hole.relief.diameter);
        var radius = diameter / 2;
        var rx = radius * Math.sqrt(Math.max(0, 1 - axis.x * axis.x));
        var ry = radius * Math.sqrt(Math.max(0, 1 - axis.y * axis.y));
        var rz = radius * Math.sqrt(Math.max(0, 1 - axis.z * axis.z));
        if (plane >= Math.min(entry.z,
          end.z) - rz - EPS && plane <= Math.max(entry.z, end.z) + rz + EPS) excluded.push({
            x: Math.min(entry.x, end.x) - rx - EPS,
            y: Math.min(entry.y, end.y) - ry - EPS,
            z: 0,
            width: Math.abs(entry.x - end.x) + 2 * (rx + EPS),
            depth: Math.abs(entry.y - end.y) + 2 * (ry + EPS),
            height: 1
          });
      }
      for (cut in cuts) {
        var b = materia.casting.CastingProfileGeometry.bounds(cut.operation.contour);
        var xs:Array<Float> = [], ys:Array<Float> = [], zs:Array<Float> = [];
        for (x in [b.x, b.x + b.width]) for (y in [b.y, b.y + b.height]) for (z in [0.0, cut.operation.depth]) {
          var point = F.transformPoint(F.compose(inverse, cut.pose), x, y, z);
          xs.push(point.x);
          ys.push(point.y);
          zs.push(point.z);
        }
        xs.sort(Reflect.compare);
        ys.sort(Reflect.compare);
        zs.sort(Reflect.compare);
        if (plane >= zs[0] - EPS && plane <= zs[7] + EPS) excluded.push({
          x: xs[0] - EPS,
          y: ys[0] - EPS,
          z: 0,
          width: xs[7] - xs[0] + 2 * EPS,
          depth: ys[7] - ys[0] + 2 * EPS,
          height: 1
        });
      }
      for (boxes in [patches, excluded]) boxes.sort((a, b) -> Reflect.compare(
        materia.tooling.ToolingCodec.canonical(a),
        materia.tooling.ToolingCodec.canonical(b)
      ));
      var certified:Array<Box3> = [];
      for (candidate in BoxAlgebra.subtract(patches, excluded)) {
        var patch = certifiedPatch(basis.footprint, candidate);
        if (patch != null) certified.push(patch);
      }
      patches = certified;
      patches.sort((a, b) -> {var area = Reflect.compare(b.width * b.depth, a.width * a.depth);
      return area != 0 ? area : Reflect.compare(
        materia.tooling.ToolingCodec.canonical(a),
        materia.tooling.ToolingCodec.canonical(b)
      );
      });
      if (patches.length
        == 0) reasons.push("No finite contact patch certified outside the source holes and datum cuts");
      else {
        var patch = patches[0];
        // Three non-collinear interior witnesses span the continuous plane's row space.
        for (index in 0...3) contacts.push({
          id: materia.tooling.ToolingIds.derive("contact-", source.id, "-" + index),
          sourceId: datumId,
          role: Location,
          point: F.transformPoint(
            basis.frame,
            patch.x + patch.width * (index == 1 ? 0.75 : 0.25),
            patch.y + patch.depth * (index == 2 ? 0.75 : 0.25),
            plane
          ),
          normal : F.transformVector(basis.frame, 0, 0, 1)
        });
      }
    }
    for (fixing in fixings) {
      var axis = F.transformVector(fixing.entry, 0, 0, 1);
      contacts.push({
        id: fixing.hardwareId,
        sourceId: fixing.hardwareId,
        role: Preload,
        point: {x: fixing.entry.x, y: fixing.entry.y, z: fixing.entry.z},
        normal: {x: -axis.x, y: -axis.y, z: -axis.z}
      });
    }
    contacts.sort((a, b) -> Reflect.compare(a.id, b.id));
    return {
      contacts: contacts,
      origin: origin,
      referenceLengthMm: scale,
      report: FixtureConstraints.analyze(contacts, origin, scale, RANK_TOLERANCE),
      unsupportedReasons: reasons
    };
  }

  public static function validation(insertId:String, location:DatumLocationPlan):ToolingValidation {
    var rule = "tooling.fixture.location." + insertId;
    if (location.unsupportedReasons.length > 0) return {
      rule: rule,
      affectedIds: [insertId],
      status: materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
      severity: materia.tooling.ToolingDiagnostic.ToolingSeverity.Error,
      message: location.unsupportedReasons.join("; "),
      evidence: []
    };
    var directions:Array<String> = [];
    for (motion in location.report.freeMotions) for (coordinate in motion
      .affectedCoordinates) if (directions.indexOf(coordinate) < 0) directions.push(coordinate);
    return ToolingReadiness.pending(
      rule,
      [insertId],
      "Insert location rank " + location.report.rank + "/6; free directions: " + directions.join(", ")
      + ". Datum-to-bed pins and clearance fixing screws provide no insert-to-datum lateral location."
    );
  }
}
