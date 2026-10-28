package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Model;
import cadkit.modeling.Scope;
import cadkit.modeling.Vector;
import materia.casting.Box3;
import materia.assembly.AssemblyFrames as F;
import materia.tooling.process.AggregateGate;
import materia.tooling.process.AggregateGate.AggregateGap;
import materia.tooling.process.AggregateGate.AggregateReport;
import materia.tooling.process.AggregateGate.AggregateBoundary;
import materia.tooling.process.CastingProcessSpec.CastabilityLimits;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.ToolingCodec;

typedef WallObstacle = {
  var ids:Array<String>;
  var insert:Bool;
  var featureIds:Array<String>;
  var assemblyIds:Array<String>;
  var tooling:Bool;
  var part:Part;
  var box:Box3;
}
typedef WallPlane = {
  var face:Model;
  var box:Box3;
  var axis:Int;
  var coordinate:Float;
  var normal:Float;
}

/** Source geometry only. Bounds cull queries and locate findings; native BREP
 * distances and Boolean material prisms determine every reported thickness. */
class CastingWallReview {
  static inline var EPS = 1e-7;
  static function bounds(part:Model):Box3 {
    var b = part.shape.bounds(), a = b.get_min(), c = b.get_max();
    return {
      x: a.get_x(),
      y: a.get_y(),
      z: a.get_z(),
      width: c.get_x() - a.get_x(),
      depth: c.get_y() - a.get_y(),
      height: c.get_z() - a.get_z()
    };
  }
  static function lo(b:Box3, axis:Int):Float return axis == 0 ? b.x : axis == 1 ? b.y : b.z;
  static function size(b:Box3, axis:Int):Float return axis == 0 ? b.width : axis == 1 ? b.depth : b.height;
  static function overlap(a:Box3, b:Box3, axis:Int):Float return Math.min(
    lo(a, axis) + size(a, axis),
    lo(b, axis) + size(b, axis)
  ) - Math.max(
    lo(a, axis),
    lo(b, axis)
  );
  static function vector(axis:Int, value:Float):Vector return new Vector(
    axis == 0 ? value : 0,
    axis == 1 ? value : 0,
    axis == 2 ? value : 0
  );
  static function separation(a:Box3, b:Box3):Float {
    var squared = 0.0;
    for (axis in 0...3) {
      var d = Math.max(
        0,
        Math.max(lo(a, axis) - lo(b, axis) - size(b, axis), lo(b, axis) - lo(a, axis) - size(a, axis))
      );
      squared += d * d;
    }
    return Math.sqrt(squared);
  }
  public static function fromPlan(plan:CastingMoldPlan, limits:CastabilityLimits):AggregateReport {
    var scope = new Scope();
    try {
      var source = scope.own(CastingMoldCad.source(plan));
      var obstacles = sourceObstacles(plan, scope);
      var bodyParts =
        [for (prism in plan.document.input.envelope) scope.own(machinekit.structural.CastingCavityCad.prism(prism))];
      var bodyLocal = scope.own(Part.fuseAll(bodyParts));
      var body = scope.own(machinekit.assembly.PosedParts.moved(bodyLocal, plan.document.setup.sourceToMold));
      var coverage:Array<materia.tooling.ToolingValidation> = [];
      var gaps = measure(plan.document.input.id, source, obstacles, limits, scope, body, coverage);
      var report = AggregateGate.evaluate(
        plan.document.input.id,
        materia.tooling.casting.CastingMoldPlanner.fingerprint(plan),
        limits,
        gaps,
        null,
        coverage
      );
      scope.close();
      return report;
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }
  public static function sourceObstacles(plan:CastingMoldPlan, scope:Scope):Array<WallObstacle> {
    var obstacles:Array<WallObstacle> = [];
    for (cavity in materia.casting.CastingCavities.derive(plan.document.input.envelope, plan.document.input.features)) {
      var feature = plan.document.input.features.filter(feature -> feature.id == cavity.featureId)[0];
      var local = scope.own(machinekit.structural.CastingCavityCad.part(cavity));
      var world = scope.own(machinekit.assembly.PosedParts.moved(local, plan.document.setup.sourceToMold));
      var ids = [feature.id];
      if (feature.owner != null) ids.push(feature.owner);
      obstacles.push({
        ids: ids,
        insert: feature.kind == materia.casting.CastingFeature.CastingFeatureKind.InsertRecess,
        featureIds: [feature.id],
        assemblyIds: feature.kind == materia.casting.CastingFeature.CastingFeatureKind.InsertRecess
        && feature.owner != null ? [feature.owner] : [],
        tooling : feature.kind != materia.casting.CastingFeature.CastingFeatureKind.InsertRecess,
        part : world,
        box : bounds(world)
      });
    }
    for (anchor in CastingMoldCad.anchorComponents(plan)) {
      var local = scope.own(anchor.part.geometry());
      var world = scope.own(machinekit.assembly.PosedParts.moved(local, anchor.pose));
      // Occupancy copy, exactly as ToolingSourceCasting/embeddedParts: the real
      // hardware keeps its machining metadata; resin exclusion has no threads.
      world.shape.setThreadedFaces([]);
      var features = plan.document.input.features.filter(feature -> feature.owner == anchor.insertId
        && feature.kind == materia.casting.CastingFeature.CastingFeatureKind.InsertRecess);
      obstacles.push({
        ids: [anchor.id, anchor.insertId, anchor.anchorId],
        insert: true,
        featureIds: features.length == 0 ? [anchor.insertId] : [for (feature in features) feature.id],
        assemblyIds : [anchor.insertId],
        tooling : false,
        part : world,
        box : bounds(world)
      });
    }
    return obstacles;
  }
  public static function measure(
    sourceId:String,
    source:Part,
    obstacles:Array<WallObstacle>,
    limits:CastabilityLimits,
    scope:Scope,
    ?body:Part,
    ?coverage:Array<materia.tooling.ToolingValidation>
  ):Array<AggregateGap> {
    var maximum = AggregateGate.captureLimit(limits);
    var rows = new Map<String, AggregateGap>(), faces:Array<Model> = [], faceBoxes:Array<Box3> = [];
    var vertexKeys:Array<Map<String, Bool>> = [];
    for (index in 0...source.shape.subshapeCount(CadKit.ShapeKind.Face)) {
      var face = scope.own(new Model(source.shape.subshape(CadKit.ShapeKind.Face, index)));
      faces.push(face);
      faceBoxes.push(bounds(face));
      var keys = new Map<String, Bool>();
      for (vertexIndex in 0...face.shape.subshapeCount(CadKit.ShapeKind.Vertex)) {
        var vertex = face.shape.subshape(CadKit.ShapeKind.Vertex, vertexIndex);
        try {
          var point = vertex.position();
          keys.set(ToolingCodec.canonical([point.get_x(), point.get_y(), point.get_z()]), true);
        } catch (error:Dynamic) {
          vertex.close();
          throw error;
        }
        vertex.close();
      }
      vertexKeys.push(keys);
    }
    function ownership(contact:Model, point:Bool = false):AggregateBoundary {
      var box = bounds(contact), ids:Array<String> = [], insert = false;
      var features:Array<String> = [], steelFeatures:Array<String> = [], assemblies:Array<String> = [], tooling = false;
      for (obstacle in obstacles) {
        if (separation(box, obstacle.box) > EPS) continue;
        var matches = false;
        if (point) matches = contact.shape.distance(obstacle.part.shape) <= EPS;
        else {
          var common = new Model(contact.shape.common(obstacle.part.shape)), area:Float;
          try area = common.shape.area() catch (error:Dynamic) {
            common.close();
            throw error;
          }
          common.close();
          matches = area > EPS;
        }
        if (!matches) continue;
        for (id in obstacle.ids) if (ids.indexOf(id) < 0) ids.push(id);
        insert = insert || obstacle.insert;
        for (id in obstacle.featureIds) {
          if (features.indexOf(id) < 0) features.push(id);
          if (obstacle.insert && steelFeatures.indexOf(id) < 0) steelFeatures.push(id);
        }
        for (id in obstacle.assemblyIds) if (assemblies.indexOf(id) < 0) assemblies.push(id);
        tooling = tooling || obstacle.tooling;
      }
      if (ids.length == 0) ids.push(sourceId);
      ids.sort(Reflect.compare);
      if (insert && steelFeatures.length > 0) features = steelFeatures;
      if (features.length == 0) features.push(sourceId);
      features.sort(Reflect.compare);
      assemblies.sort(Reflect.compare);
      return {ids: ids, featureIds: features, assemblyIds: assemblies, insert: insert, tooling: tooling};
    }
    function add(
      a:AggregateBoundary,
      b:AggregateBoundary,
      distance:Float,
      region:Box3,
      method:String,
      witness:Null<{first: {x:Float, y:Float, z:Float}, second: {x:Float, y:Float, z:Float}}>,
      ?axis:materia.tooling.process.OrthogonalSections.SectionAxis,
      allowContact:Bool = false
    ):Void {
      if (distance < 0 || distance <= EPS && !allowContact || distance >= maximum - EPS) return;
      var sourceIds = a.ids.copy();
      for (id in b.ids) if (sourceIds.indexOf(id) < 0) sourceIds.push(id);
      sourceIds.sort(Reflect.compare);
      for (kind in AggregateGate.classes(a, b)) for (featurePair in AggregateGate.pairs(a, b)) {
        var row:AggregateGap = {
          id: "gap",
          kind: kind,
          first: a,
          second: b,
          featurePair: featurePair,
          sourceIds: sourceIds,
          measuredMm: distance,
          axis: axis,
          region: region,
          method: method,
          witness: witness
        };
        row.id = materia.tooling.ToolingIds.derive(
          "aggregate-gap-",
          haxe.crypto.Sha256.encode(ToolingCodec.canonical(row)).substr(0, 24)
        );
        rows.set(row.id, row);
      }
    }
    function length(shape:cadkit.Shape):Float {
      var result = 0.0;
      for (index in 0...shape.subshapeCount(CadKit.ShapeKind.Edge)) {
        var edge = shape.subshape(CadKit.ShapeKind.Edge, index);
        try result += edge.edgeLength() catch (error:Dynamic) {
          edge.close();
          throw error;
        }
        edge.close();
      }
      return result;
    }
    // Boolean distributivity: Q \ (body \ obstacles) equals
    // (Q \ body) union (Q intersect each obstacle). Query only overlapping
    // obstacle bounds; the exact primitives remain authoritative.
    function outsideParts(query:Model, local:Scope):Array<Model> {
      if (body == null) return [local.own(new Model(query.shape.cut(source.shape)))];
      var result = [local.own(new Model(query.shape.cut(body.shape)))], box = bounds(query);
      for (obstacle in obstacles) if (separation(box,
        obstacle.box) <= EPS) result.push(local.own(new Model(query.shape.common(obstacle.part.shape))));
      return result;
    }
    function missingVolume(query:Model, local:Scope):Bool {
      for (part in outsideParts(query, local)) if (part.shape.volume() > EPS) return true;
      return false;
    }
    // A body without a cavity still has a minimum wall (e.g. a 20 mm slab).
    function materialSpan(lower:WallPlane, upper:WallPlane):Bool {
      if (lower.axis != upper.axis) return true;
      var delta = upper.coordinate - lower.coordinate;
      if (delta <= EPS || delta >= maximum - EPS) return true;
      for (axis in 0...3) if (axis != lower.axis && overlap(lower.box, upper.box, axis) <= EPS) return true;
      var local = new Scope();
      try {
        var moved = local.own(new Model(upper.face.shape.translate(vector(lower.axis, -delta).native())));
        var patch = local.own(new Model(lower.face.shape.common(moved.shape)));
        if (patch.shape.area() > EPS) {
          var prism = local.own(new Part(patch.shape.extrude(vector(lower.axis, delta).native())));
          for (missing in outsideParts(prism, local)) if (missing.shape.volume() > EPS) {
            // Remove the entire exact shadow of this intervening void.
            var swept:Part;
            try swept = local.own(SweptRemoval.sweep(
              local.own(new Part(missing.shape.cloneShape())),
              vector(lower.axis, -2 * delta)
            )) catch (error:Dynamic) {
              if (Std.string(error).indexOf("Unsupported removal boundary surface") < 0) throw error;
              var owner = ownership(missing), region = bounds(prism);
              var id = haxe.crypto.Sha256.encode(ToolingCodec.canonical({
                source: sourceId,
                owners: owner.ids,
                region: region
              })).substr(
                0,
                24
              );
              var check = materia.tooling.process.FillAnalysis.check(
                "process.castability.aggregate-coverage." + id,
                owner.ids,
                materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported,
                "Exact opposing-plane gap coverage is unavailable across this lofted boundary: "
                + ToolingCodec.canonical(region) + ". " + Std.string(error)
                + ". Exact closest-segment checks continue; no bound or sampled replacement is credited.",
                []
              );
              if (coverage == null) throw check.message;
              if (coverage.filter(row -> row.rule == check.rule).length == 0) coverage.push(check);
              local.close();
              return false;
            }
            var shadow = local.own(new Model(patch.shape.common(swept.shape)));
            patch = local.own(new Model(patch.shape.cut(shadow.shape)));
          }
          if (patch.shape.area() > EPS) prism = local.own(new Part(patch.shape.extrude(vector(lower.axis,
            delta).native())));
          if (patch.shape.area() > EPS) {
            if (missingVolume(prism, local)) throw "Aggregate wall shadow differs from source: " + sourceId;
            var upperPatch = local.own(new Model(patch.shape.translate(vector(lower.axis, delta).native())));
            var mask:Null<Model> = null;
            for (obstacle in obstacles) if (obstacle.insert) for (end in [patch, upperPatch]) {
              if (separation(bounds(end), obstacle.box) > EPS) continue;
              var contact = local.own(new Model(end.shape.common(obstacle.part.shape)));
              if (contact.shape.area() <= EPS) continue;
              if (end == upperPatch) contact = local.own(new Model(contact.shape.translate(vector(lower.axis,
                -delta).native())));
              mask = mask == null ? contact : local.own(new Model(mask.shape.fuse(contact.shape)));
            }
            var pieces:Array<Model> = mask == null ? [patch] : [
              local.own(new Model(patch.shape.cut(mask.shape))),
              local.own(new Model(patch.shape.common(mask.shape)))
            ];
            for (piece in pieces) if (piece.shape.area() > EPS) {
              var top = local.own(new Model(piece.shape.translate(vector(lower.axis, delta).native())));
              var volume = local.own(new Part(piece.shape.extrude(vector(lower.axis, delta).native())));
              add(
                ownership(piece),
                ownership(top),
                delta,
                bounds(volume),
                "Exact opposing-plane material prism; intervening void shadows subtracted",
                null,
                [
                  materia.tooling.process.OrthogonalSections.SectionAxis.X,
                  materia.tooling.process.OrthogonalSections.SectionAxis.Y,
                  materia.tooling.process.OrthogonalSections.SectionAxis.Z
                ][lower.axis]
              );
            }
          }
        }
        local.close();
        return true;
      } catch (error:Dynamic) {
        local.close();
        throw error;
      }
    }
    // Touching separate steel assemblies still have zero clear spacing. Their
    // shared interface disappears from the resin BREP, so certify contact from
    // the actual solids clipped to the source envelope, not a fictitious ray.
    if (body != null) for (left in 0...obstacles.length) for (right in left + 1...obstacles.length) {
      var a = obstacles[left], b = obstacles[right];
      if (!a.insert || !b.insert || a.assemblyIds.filter(id -> b.assemblyIds.indexOf(id) >= 0).length > 0 || separation(
        a.box,
        b.box
      ) > EPS) continue;
      if (a.part.shape.distance(b.part.shape) > EPS) continue;
      var local = new Scope();
      try {
        // Solid/solid common omits a shared boundary with no volume.
        // Intersect each boundary face instead, retaining the exact contact.
        for (faceIndex in 0...a.part.shape.subshapeCount(CadKit.ShapeKind.Face)) {
          var face = local.own(new Model(a.part.shape.subshape(CadKit.ShapeKind.Face, faceIndex)));
          var shared = local.own(new Model(face.shape.common(b.part.shape)));
          var contact = local.own(new Model(shared.shape.common(body.shape)));
          if (contact.shape.subshapeCount(CadKit.ShapeKind.Vertex) + contact.shape.subshapeCount(CadKit.ShapeKind.Edge)
            + contact.shape.subshapeCount(CadKit.ShapeKind.Face) > 0) add(
              {ids: a.ids, featureIds: a.featureIds, assemblyIds: a.assemblyIds, insert: true, tooling: false},
              {ids: b.ids, featureIds: b.featureIds, assemblyIds: b.assemblyIds, insert: true, tooling: false},
              0,
              bounds(contact),
              "Exact contact between separate steel assemblies inside the casting envelope",
              null,
              null,
              true
            );
        }
        local.close();
      } catch (error:Dynamic) {
        local.close();
        throw error;
      }
    }
    var planeByIndex = new Map<Int, WallPlane>();
    for (index in 0...faces.length) {
      var face = faces[index];
      if (face.shape.surfaceKind() != CadKit.SurfaceKind.Plane) continue;
      var n = face.shape.faceNormal(),
        c = face.shape.center(), normal = [n.get_x(), n.get_y(), n.get_z()], coordinate = [
          c.get_x(),
          c.get_y(),
          c.get_z()
        ];
      for (axis in 0...3) if (Math.abs(Math.abs(normal[axis]) - 1) < 1e-9) planeByIndex.set(
        index,
        {face: face, box: faceBoxes[index], axis: axis, coordinate: coordinate[axis], normal: normal[axis]}
      );
    }
    if (Sys.getEnv("MATERIA_AGGREGATE_PROGRESS") == "1") Sys.println("Aggregate faces: " + faces.length);
    for (left in 0...faces.length) {
      if (Sys.getEnv("MATERIA_AGGREGATE_PROGRESS") == "1" && left % 100
        == 0) Sys.println("Aggregate face " + left + "/" + faces.length + "; gaps " + [for (_ in rows) 1].length);
      for (right in left + 1...faces.length) {
        if (separation(faceBoxes[left], faceBoxes[right]) >= maximum - EPS) continue;
        // Coincident endpoints prove zero face distance. Such pairs never enter
        // the positive-gap query; avoid invoking the general extrema solver.
        var touching = false;
        for (key in vertexKeys[left].keys()) if (vertexKeys[right].exists(key)) {
          touching = true;
          break;
        }
        if (touching) continue;
        var a = planeByIndex.get(left), b = planeByIndex.get(right);
        if (a != null && b != null && a.axis == b.axis && a.normal * b.normal < 0) {
          var lower = a.normal < 0 ? a : b, upper = a.normal < 0 ? b : a;
          var overlaps = true;
          for (axis in 0...3) if (axis != a.axis && overlap(a.box, b.box, axis) <= EPS) overlaps = false;
          if (overlaps) {
            if (materialSpan(lower, upper)) continue;
          }
        }
        var first = faces[left].shape.distanceWitness(faces[right].shape);
        if (first.get_distance() <= EPS || first.get_distance() >= maximum - EPS) continue;
        for (index in 0...first.get_count()) {
          var pair = index == 0 ? first : faces[left].shape.distanceWitness(faces[right].shape, index);
          var start = Vector.fromNative(pair.get_first()),
            end = Vector.fromNative(pair.get_second()), local = new Scope();
          try {
            var segment = local.own(cadkit.modeling.Curve.line(start, end));
            var clear = true;
            for (outside in outsideParts(segment, local)) if (length(outside.shape) > EPS) {
              clear = false;
              break;
            }
            if (clear) {
              var box = bounds(segment);
              var nearby = [for (index in 0...faces.length) if (separation(box,
                faceBoxes[index]) <= EPS) faces[index].shape];
              var localBoundary = local.own(new Model(cadkit.Shape.compound(nearby)));
              var contact = local.own(new Model(segment.shape.common(localBoundary.shape)));
              // A path running along any free-surface interval is not a material wall. The whole
              // segment is checked by BREP Booleans; no point/pose samples are used.
              if (length(contact.shape) <= EPS) {
                var firstVertex = local.own(new Model(segment.shape.subshape(CadKit.ShapeKind.Vertex, 0)));
                var secondVertex = local.own(new Model(segment.shape.subshape(CadKit.ShapeKind.Vertex, 1)));
                add(
                  ownership(firstVertex, true),
                  ownership(secondVertex, true),
                  pair.get_distance(),
                  bounds(segment),
                  "Exact closest segment inside resin",
                  {first: {x: start.x, y: start.y, z: start.z}, second: {x: end.x, y: end.y, z: end.z}}
                );
              }
            }
            local.close();
          } catch (error:Dynamic) {
            local.close();
            throw error;
          }
        }
      }
    }
    var result = [for (row in rows) row];
    result.sort((a, b) -> Reflect.compare(a.id, b.id));
    return result;
  }
}
