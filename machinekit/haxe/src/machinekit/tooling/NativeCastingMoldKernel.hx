package machinekit.tooling;

import cadkit.modeling.Part;
import cadkit.modeling.Scope;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingRegion;
import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.CastingMoldKernel;
import materia.tooling.casting.CastingMoldKernel.CastingRegionMoments;
import materia.tooling.casting.CastingMoldKernel.CastingMassProperties;

/** Exact CAD queries used by non-box planning. Caches retain numbers only;
 * every native resource closes before a query returns, including throw paths.
 */
class NativeCastingMoldKernel implements CastingMoldKernel {
  final regionCache:Map<String, CastingRegionMoments> = [];
  final regionKeys:Array<String> = [];
  final massCache:Map<String, CastingMassProperties> = [];
  final massKeys:Array<String> = [];
  static inline var CACHE_LIMIT = 4096;

  public function new() {
  }

  public function bounds(region:CastingRegion, affectedId:String):materia.casting.Box3 {
    var scope = new Scope();
    try {
      var source = scope.own(machinekit.structural.CastingRegionCad.part(region));
      var intervals:Array<Array<Float>> = [];
      for (axis in 0...3) {
        var frame = CastingSectionContours.normalFrame(
          {
            x: axis == 0 ? 1 : 0,
            y : axis == 1 ? 1 : 0,
            z : axis == 2 ? 1 : 0
          },
          affectedId
        );
        var local = scope.own(machinekit.assembly.PosedParts.moved(source, F.inverse(frame)));
        var geometry = new CastingFillGeometry(affectedId, local);
        try intervals.push(geometry.heightInterval()) catch (error:Dynamic) {
          geometry.close();
          throw error;
        }
        geometry.close();
      }
      var result = {
        x: intervals[0][0],
        y: intervals[1][0],
        z: intervals[2][0],
        width: intervals[0][1] - intervals[0][0],
        depth: intervals[1][1] - intervals[1][0],
        height: intervals[2][1] - intervals[2][0]
      };
      scope.close();
      return result;
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }

  public function moments(region:CastingRegion):CastingRegionMoments {
    var key = haxe.crypto.Sha256.encode(ToolingCodec.canonical(region));
    var cached = regionCache.get(key);
    if (cached != null) return copy(cached);
    var part = machinekit.structural.CastingRegionCad.part(region);
    var result:CastingRegionMoments;
    try result = measure(part) catch (error:Dynamic) {
      part.close();
      throw error;
    }
    part.close();
    if (regionKeys.length >= CACHE_LIMIT) regionCache.remove(regionKeys.shift());
    regionKeys.push(key);
    regionCache.set(key, copy(result));
    return result;
  }

  public function section(region:CastingRegion, plane:materia.assembly.AssemblyRecord.AssemblyFrame,
    affectedId:String):Array<materia.casting.CastingSection.CastingSectionContour> {
    var source = machinekit.structural.CastingRegionCad.part(region);
    try {
      var result = CastingSectionContours.section(source, plane, affectedId);
      source.close();
      return result;
    } catch (error:Dynamic) {
      source.close();
      throw error;
    }
  }

  public function prismaticSection(
    region:CastingRegion,
    direction: {x: Float, y: Float, z: Float},
    toleranceMm3:Float,
    affectedId:String
  ):Null<materia.casting.CastingSection.CastingPrismSection> {
    if (!Math.isFinite(toleranceMm3) || toleranceMm3 < 0) throw "Invalid prism tolerance: " + affectedId;
    var frame = CastingSectionContours.normalFrame(direction, affectedId), scope = new Scope();
    try {
      var source = scope.own(machinekit.structural.CastingRegionCad.part(region));
      if (source.isEmpty()) {
        scope.close();
        return null;
      }
      var local = scope.own(machinekit.assembly.PosedParts.moved(source, F.inverse(frame)));
      var geometry = new CastingFillGeometry(affectedId, local), interval:Array<Float>;
      try interval = geometry.heightInterval() catch (error:Dynamic) {
        geometry.close();
        throw error;
      }
      geometry.close();
      var depth = interval[1] - interval[0];
      if (depth <= 0) {
        scope.close();
        return null;
      }
      var contours:Array<materia.casting.CastingSection.CastingSectionContour>;
      try contours = CastingSectionContours.section(
        local,
        F.translation(0, 0, (interval[0] + interval[1]) / 2),
        affectedId
      ) catch (error:Dynamic) {
        var message = Std.string(error);
        if (message.indexOf("Unsupported section curve") < 0
          && message.indexOf("Ambiguous section junction") < 0) throw error;
        scope.close();
        return null;
      }
      if (contours.length == 0) {
        scope.close();
        return null;
      }
      var pose = F.compose(frame, F.translation(0, 0, interval[0]));
      var candidate = scope.own(machinekit.structural.CastingRegionCad.part(materia.tooling.casting.MoldRegions.section(
        contours,
        pose,
        depth
      )));
      var missing = scope.own(source.subtractAll([candidate])), excess = scope.own(candidate.subtractAll([source]));
      var difference = missing.volume() + excess.volume();
      if (!Math.isFinite(difference)) throw "Nonfinite prism difference: " + affectedId;
      var result:materia.casting.CastingSection.CastingPrismSection = {pose: pose, depth: depth, contours: contours};
      scope.close();
      return difference <= toleranceMm3 ? result : null;
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }

  public function prismaticBands(region:CastingRegion,
    width:Float, depth:Float, height:Float, id:String):Array<materia.casting.CastingSection.CastingSectionBand> {
    var source = machinekit.structural.CastingRegionCad.part(region);
    var levels:Array<Float> = [];
    try {
      // Every depth transition of a line/arc 2.5D prism has a vertex.
      // The whole-band prism proof below rejects curved/hidden variations;
      // these levels never approximate a non-prismatic surface.
      for (index in 0...source.shape.subshapeCount(CadKit.ShapeKind.Vertex)) {
        var vertex = source.shape.subshape(CadKit.ShapeKind.Vertex, index);
        try levels.push(vertex.position().get_z()) catch (error:Dynamic) {
          vertex.close();
          throw error;
        }
        vertex.close();
      }
      source.close();
    } catch (error:Dynamic) {
      source.close();
      throw error;
    }
    levels.sort(Reflect.compare);
    var unique:Array<Float> = [];
    for (level in levels) if (unique.length == 0 || level - unique[unique.length - 1] > 1e-7) unique.push(level);
    levels = unique;
    if (levels.length < 2) throw "Solid has no positive prismatic depth: " + id;
    levels[0] = 0;
    levels[levels.length - 1] = height;
    var bands:Array<materia.casting.CastingSection.CastingSectionBand> = [];
    var index = levels.length - 1;
    while (index > 0) {
      var top = levels[index], bottom = levels[index - 1];
      index--;
      if (top - bottom < 1e-7) continue;
      var slice:CastingRegion = Intersection([
        region,
        materia.tooling.casting.MoldRegions.box({
          x: 0,
          y: 0,
          z: bottom,
          width: width,
          depth: depth,
          height: top - bottom
        })
      ]);
      var prism = prismaticSection(slice, {x: 0, y: 0, z: 1}, 1e-6, id);
      if (prism == null) throw "Non-prismatic machining band " + bottom + ".." + top + " mm";
      bands.push({topMm: top, bottomMm: bottom, contours: prism.contours});
    }
    return bands;
  }

  static function copy(value:CastingRegionMoments):CastingRegionMoments return {
    volumeMm3: value.volumeMm3,
    firstMomentMm4: {
      x: value.firstMomentMm4.x,
      y: value.firstMomentMm4.y,
      z: value.firstMomentMm4.z
    }
  };

  static function measure(part:Part):CastingRegionMoments {
    if (part.isEmpty()) return {volumeMm3: 0, firstMomentMm4: {x: 0, y: 0, z: 0}};
    var properties = part.massProperties(), volume = properties.volume, center = properties.centerOfMass;
    var result = {volumeMm3: volume,
      firstMomentMm4: {x: volume * center.x, y: volume * center.y, z: volume * center.z}};
    for (value in [volume, result.firstMomentMm4.x, result.firstMomentMm4.y,
      result.firstMomentMm4.z]) if (!Math.isFinite(value)) throw "Nonfinite native casting moments";
    if (volume < 0) throw "Negative native casting volume";
    return result;
  }

  public function castingMass(document:ToolingDocument):CastingMassProperties {
    var d = ToolingCodec.copy(document), key = ToolingCodec.documentFingerprint(d);
    var cached = massCache.get(key);
    if (cached != null) return copyMass(cached);
    var scope = new Scope();
    var total = {mass: 0.0, x: 0.0, y: 0.0, z: 0.0};
    try {
      var resin = scope.own(ToolingSourceCasting.fromInput(d.input).geometry());
      contribution(total, measure(resin), d.setup.sourceToMold, d.recipe.mixDensityKgM3);
      for (insert in d.input.inserts) {
        var local = scope.own(ToolingInsertCad.component(insert).geometry());
        local.shape.setThreadedFaces([]);
        var parts = [local];
        var spec:materia.casting.InsertSpec = {
          body: insert.body,
          holes: insert.holes,
          origin: insert.origin,
          interfaces: insert.interfaces,
          anchorage: insert.anchorage
        };
        for (hardware in machinekit.tooling.InsertAnchorGeometry.components(spec)) {
          var raw = scope.own(hardware.part.geometry());
          raw.shape.setThreadedFaces([]);
          parts.push(scope.own(machinekit.assembly.PosedParts.moved(raw, hardware.pose)));
        }
        var steel = scope.own(Part.fuseAll(parts));
        contribution(total, measure(steel), F.compose(d.setup.sourceToMold, insert.pose), d.recipe.steelDensityKgM3);
      }
      scope.close();
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
    if (!Math.isFinite(total.mass + total.x + total.y + total.z)
      || total.mass <= 0) throw "Invalid native casting mass: " + d.input.id;
    var result = {
      massKg: total.mass,
      centerOfMassMoldMm: {
        x: total.x / total.mass,
        y: total.y / total.mass,
        z: total.z / total.mass
      }
    };
    if (massKeys.length >= CACHE_LIMIT) massCache.remove(massKeys.shift());
    massKeys.push(key);
    massCache.set(key, copyMass(result));
    return result;
  }

  public function resinIntersections(document:ToolingDocument, regions:Array<CastingRegion>):Array<Float> {
    if (regions.length == 0) return [];
    var d = ToolingCodec.copy(document), scope = new Scope();
    try {
      var local = scope.own(ToolingSourceCasting.fromInput(d.input).geometry());
      var resin = scope.own(machinekit.assembly.PosedParts.moved(local, d.setup.sourceToMold));
      var bounds = resin.shape.bounds();
      var low = cadkit.modeling.Vector.fromNative(bounds.get_min()),
        high = cadkit.modeling.Vector.fromNative(bounds.get_max());
      var height = Math.min(high.z, d.setup.fillHeightMm) - low.z;
      var volumes:Array<Float> = [];
      if (height > 0) {
        var blank = scope.own(Part.box(high.x - low.x, high.y - low.y, height));
        var clip = scope.own(blank.translated(new cadkit.modeling.Vector(
          (low.x + high.x) / 2,
          (low.y + high.y) / 2,
          low.z
        )));
        var filled = scope.own(resin.intersect(clip));
        for (region in regions) {
          var query = new Scope();
          try {
            var candidate = query.own(machinekit.structural.CastingRegionCad.part(region));
            volumes.push(measure(query.own(filled.intersect(candidate))).volumeMm3);
            query.close();
          } catch (error:Dynamic) {
            query.close();
            throw error;
          }
        }
      } else for (region in regions) volumes.push(0);
      scope.close();
      return volumes;
    } catch (error:Dynamic) {
      scope.close();
      throw error;
    }
  }

  static function copyMass(value:CastingMassProperties):CastingMassProperties return {
    massKg: value.massKg,
    centerOfMassMoldMm: {
      x: value.centerOfMassMoldMm.x,
      y: value.centerOfMassMoldMm.y,
      z: value.centerOfMassMoldMm.z
    }
  };

  static function contribution(
    total: {mass: Float, x: Float, y: Float, z: Float},
    moments:CastingRegionMoments,
    pose:materia.assembly.AssemblyRecord.AssemblyFrame,
    densityKgM3:Float
  ):Void {
    if (moments.volumeMm3 == 0) return;
    var factor = densityKgM3 * 1e-9, first = moments.firstMomentMm4;
    var center = F.transformPoint(
      pose,
      first.x / moments.volumeMm3,
      first.y / moments.volumeMm3,
      first.z / moments.volumeMm3
    );
    var mass = moments.volumeMm3 * factor;
    total.mass += mass;
    total.x += mass * center.x;
    total.y += mass * center.y;
    total.z += mass * center.z;
  }
}
