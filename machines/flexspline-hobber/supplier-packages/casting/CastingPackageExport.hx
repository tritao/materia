import hobber.*;
import hobber.HobberSkeleton;
import machinekit.assembly.PosedParts;
import hobber.MineralCastBed;
import hobber.MineralCastColumn;
import machinekit.structural.CastInsert;
import machinekit.structural.CastingCavityCad;
import machinekit.structural.MineralFoundation;
import machinekit.tooling.CastingSectionContours;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InsertStages.InsertStage;

/** Writes one mineral casting of FlexsplineHobber (the base or the column) for a casting supplier: STEP of the
    as-cast body, STEP of the body with its cast-in steel (full and light), a STEP of each distinct insert, exact
    sections of the body, and <kind>-package.json for casting_package.py. Both castings are built by the same
    MineralCastBed / MineralCastColumn FlexsplineHobber installs.
    Usage: hl main.hl <output dir> bed|column. Each run takes one to a few minutes (it runs the full anchor sizing). */
class CastingPackageExport {
  static inline final SECTION_ARC = 5.0;
  static final STAGE = InsertStage.BeforeCasting;

  static function pt(p:{x:Float, y:Float, z:Float}):Array<Float> return [p.x, p.y, p.z];

  static function axes(pose:AssemblyFrame):Dynamic return {
    x: pt(F.transformVector(pose, 1, 0, 0)),
    y: pt(F.transformVector(pose, 0, 1, 0)),
    z: pt(F.transformVector(pose, 0, 0, 1))
  };

  static function boxJson(b:PosedParts.PartBox):Dynamic return {
    min: [b.minX, b.minY, b.minZ],
    max: [b.maxX, b.maxY, b.maxZ]
  };

  static function bodyJson(body:InsertBody):Dynamic return switch body {
    case Primitive(Box(w, d, h)):
      {kind: "box", width: w, depth: d, height: h};
    case Primitive(Cylinder(d, h)):
      {kind: "cylinder", diameter: d, height: h};
    case CrimpedSocket(Cylinder(d, h), tail):
      {
        kind: "crimped-socket",
        diameter: d,
        boreLength: h,
        tailLength: tail.length,
        tailWidth: tail.width,
        tailThickness: tail.thickness
      };
    default:
      throw "unexpected insert body";
  };

  static function holeJson(pose:AssemblyFrame, hole:materia.casting.InsertHole, beforeCasting:Array<String>):Dynamic {
    var at = F.compose(pose, hole.frame), thread = hole.thread;
    return {
      id: hole.id,
      entry: pt({x: at.x, y: at.y, z: at.z}),
      direction: pt(F.transformVector(at, 0, 0, 1)),
      diameter: hole.diameter,
      depth: hole.depth,
      extent: Std.string(hole.extent),
      thread: thread == null ? null : thread.designation,
      threadClass: thread == null ? null : thread.toleranceClass,
      threadPitch: thread == null ? null : thread.pitch,
      tapDrill: hole.tapDrill,
      threadDepth: hole.threadDepth,
      clearanceDiameter: hole.clearanceDiameter,
      counterbore: hole.counterbore,
      relief: hole.relief,
      route: beforeCasting.indexOf(hole.id) >= 0 ? "before casting (in the pour)" : "after casting"
    };
  }

  /** Section of the as-cast body, world coordinates, arcs sampled. */
  static function section(body:cadkit.modeling.Part, name:String, axis:String, offset:Float):Dynamic {
    var direction = axis == "x" ? {x: 1.0, y: 0.0, z: 0.0} : axis == "y" ? {x: 0.0, y: 1.0, z: 0.0} : {x: 0.0, y: 0.0, z: 1.0};
    var plane = F.compose(
      F.translation(direction.x * offset, direction.y * offset, direction.z * offset),
      CastingSectionContours.normalFrame(direction, name)
    );
    var contours = CastingSectionContours.section(body, plane, name);
    return {
      id: name,
      axis: axis,
      offset: offset,
      contours: [for (c in contours) {
        depth: c.depth,
        points: {
          var out:Array<Array<Float>> = [];
          for (s in c.contour) {
            var local:Array<{x:Float, y:Float}> = [];
            if (Std.string(s.kind).toLowerCase() == "line") local = [s.start, s.end];
            else {
              var n = Std.int(Math.max(2, Math.ceil(Math.abs(s.sweep) * 180 / Math.PI / SECTION_ARC)));
              var centre = s.center;
              if (centre == null) throw "arc without centre";
              var a0 = Math.atan2(s.start.y - centre.y, s.start.x - centre.x);
              var r = Math.sqrt(Math.pow(s.start.x - centre.x, 2) + Math.pow(s.start.y - centre.y, 2));
              for (i in 0...n + 1) local.push({
                x: centre.x + r * Math.cos(a0 + s.sweep * i / n),
                y: centre.y + r * Math.sin(a0 + s.sweep * i / n)
              });
            }
            for (p in local) {
              var w = pt(F.transformPoint(plane, p.x, p.y, 0));
              if (out.length == 0 || Math.abs(out[out.length - 1][0] - w[0]) + Math.abs(out[out.length - 1][1] - w[1])
                + Math.abs(out[out.length - 1][2] - w[2]) > 1e-9) out.push(w);
            }
          }
          out;
        }
      }]
    };
  }

  static function main() {
    var args = Sys.args();
    if (args.length != 2 || (args[1] != "bed" && args[1] != "column")) throw "usage: hl main.hl <output dir> bed|column";
    var dir = args[0], kind = args[1];
    sys.FileSystem.createDirectory(dir);
    sys.FileSystem.createDirectory(dir + "/inserts");
    var began = Sys.time();
    var foundation:MineralFoundation = kind == "bed" ? new MineralCastBed() : new MineralCastColumn();
    var sized = Sys.time() - began;
    var body = foundation.body, cache = new PosedParts(), closers:Array<cadkit.modeling.Part> = [];
    var dims = HobberSkeleton.reference().foundationDimensions(kind);
    var asCast = body.asCastGeometry();
    closers.push(asCast);
    var bounds = PosedParts.boxOf(asCast);
    var byWeight = body.castingByWeight();
    var volume = asCast.volume();
    var mass = byWeight.kg;
    var physical = materia.project.MaterialLibrary.require("mineral-cast").physical;
    asCast.shape.exportStep('$dir/$kind-casting.step');

    // Cavities: every design cut that reaches the body, except the insert recesses (listed with their inserts).
    var cavities:Array<Dynamic> = [], recesses:Array<Dynamic> = [];
    var features = body.castingFeatures();
    for (cavity in body.finishedCavities()) {
      var feature = features.filter(f -> f.id == cavity.featureId)[0];
      var part = CastingCavityCad.part(cavity), b = PosedParts.boxOf(part);
      var row:Dynamic = {
        id: cavity.featureId,
        kind: Std.string(feature.kind),
        openings: [for (o in feature.openings) o.target],
        owner: feature.owner,
        draftDegrees: feature.draft.degrees,
        cornerRadius: feature.cornerRadius,
        volumeMm3: part.volume(),
        min: [b.minX, b.minY, b.minZ],
        max: [b.maxX, b.maxY, b.maxZ]
      };
      part.close();
      if (feature.kind == InsertRecess) recesses.push(row);
      else cavities.push(row);
    }

    // Inserts: distinct geometries get one STEP each; every placement is listed.
    var posed:Array<cadkit.Shape> = [], posedLight:Array<cadkit.Shape> = [], inserts:Array<Dynamic> = [];
    posed.push(asCast.shape);
    posedLight.push(asCast.shape);
    var files:Map<String, String> = [], quantity:Map<String, Array<Int>> = [], used:Map<String, Bool> = [];
    var steps:Array<Dynamic> = [];
    // Identical steel (same body, holes, origin and stock) shares one STEP file.
    function keyOf(part:CastInsert):String return part.parameterToken("body") + "|" + part.parameterToken("holes") + "|"
      + part.parameterToken("origin") + "|" + part.parameterToken("stockMaterial");
    function slug(text:String):String return StringTools.replace(StringTools.replace(
      StringTools.replace(text.toLowerCase(), " ", "-"),
      "hobber-",
      ""
    ), kind + "-", "");
    var counts:Map<String, Array<Int>> = [];
    for (row in foundation.insertLocations()) {
      var k = keyOf(row.part);
      if (!counts.exists(k)) counts.set(k, [0]);
      counts.get(k)[0]++;
    }
    for (row in foundation.insertLocations()) {
      var service = row.part, spec = service.insertSpec();
      var raw = new CastInsert(service.values().copy().setToken("buildStage", STAGE));
      var key = keyOf(service);
      var local = cache.local(raw);
      if (!files.exists(key)) {
        var base = counts.get(key)[0] > 1 ? slug(service.parameterToken("label")) : slug(service.parameterToken("partId") == "" ? service.parameterToken("label") : service.parameterToken("partId"));
        var name = base, n = 1;
        while (used.exists(name)) name = base + "-" + (++n);
        used.set(name, true);
        files.set(key, 'inserts/$kind-insert-$name.step');
        quantity.set(key, [0]);
        local.shape.exportStep(dir + "/" + files.get(key));
        var lb = PosedParts.boxOf(local);
        steps.push({
          step: files.get(key),
          body: bodyJson(spec.body),
          stockMassKg: raw.massProperties().mass,
          localBounds: boxJson(lb),
          firstPlacement: row.id
        });
      }
      quantity.get(key)[0]++;
      var placed = PosedParts.moved(local, row.pose);
      closers.push(placed);
      var box = PosedParts.boxOf(placed);
      posed.push(placed.shape);
      posedLight.push(placed.shape);
      var lb = PosedParts.boxOf(local);
      var before = [for (h in materia.casting.InsertStages.holes(spec, STAGE)) h.id];
      var origin:Dynamic = switch spec.origin {
        case Fabricated(m):
          {kind: "made by us or the foundry from bar", process: Std.string(service.bom.manufacturingProcess), material: Std.string(m.material.grade)};
        case Catalogue(p):
          {
            kind: "bought catalogue part",
            manufacturer: p.manufacturer,
            reference: p.reference,
            document: p.document,
            geometryBasis: Std.string(p.geometryBasis)
          };
      };
      var finish = service.parameterToken("finish");
      var topZ = lb.maxZ;
      var datum:Dynamic = null;
      if (finish == "ground-after-casting" || finish == "machined-after-casting" || finish == "replicated-after-casting")
        datum = {
          id: row.id + "/mount",
          normal: pt(F.transformVector(row.pose, 0, 0, 1)),
          point: pt(F.transformPoint(row.pose, 0, 0, topZ)),
          finish: finish
        };
      var anchors = machinekit.tooling.InsertAnchorGeometry.components(spec), anchorRows:Array<Dynamic> = [];
      for (a in anchors) {
        var ap = F.compose(row.pose, a.pose), part = cache.local(a.part);
        var moved = PosedParts.moved(part, ap);
        closers.push(moved);
        moved.shape.setThreadedFaces([]);
        posed.push(moved.shape);
        anchorRows.push({
          id: a.anchorId,
          description: a.part.bom.description,
          massKg: a.part.massProperties().mass,
          partNumber: a.part.bom.partNumber,
          position: pt({x: ap.x, y: ap.y, z: ap.z}),
          direction: pt(F.transformVector(ap, 0, 0, 1))
        });
      }
      inserts.push({
        id: row.id,
        role: service.parameterToken("role"),
        label: service.parameterToken("label"),
        partId: service.parameterToken("partId"),
        partNumber: service.bom.partNumber,
        description: service.bom.description,
        step: files.get(key),
        body: bodyJson(spec.body),
        material: service.materialSpec(),
        stockMaterial: service.parameterToken("stockMaterial"),
        minimumTensileStrengthMPa: service.insertMaterial().minimumTensileStrengthMPa,
        origin: origin,
        finish: finish,
        massKg: raw.massProperties().mass,
        position: pt({x: row.pose.x, y: row.pose.y, z: row.pose.z}),
        axes: axes(row.pose),
        mountNormal: pt(F.transformVector(row.pose, 0, 0, 1)),
        localBounds: boxJson(lb),
        bounds: boxJson(box),
        positionToleranceMm: service.parameterNumber("positionTolerance"),
        precisionInterfaces: spec.interfaces.length,
        datumFace: datum,
        openings: [for (o in row.openings) o.target],
        holes: [for (h in spec.holes) holeJson(row.pose, h, before)],
        anchorage: materia.casting.InsertAnchorage.InsertAnchorageCodec.record(spec.anchorage),
        anchors: anchorRows
      });
    }
    for (s in steps) {
      var count = 0;
      for (key in files.keys()) if (files.get(key) == s.step) count = quantity.get(key)[0];
      Reflect.setField(s, "quantity", count);
    }
    cadkit.Shape.compound(posed).exportStep('$dir/$kind-casting-with-inserts.step');
    cadkit.Shape.compound(posedLight).exportStep('$dir/$kind-casting-with-inserts-light.step');

    // Sections of the as-cast body: the six faces (0.5 mm inside) and internal planes.
    var lo = bounds, sections:Array<Dynamic> = [];
    var faceInset = 0.5;
    sections.push(section(asCast, "face-z-max", "z", lo.maxZ - faceInset));
    sections.push(section(asCast, "face-z-min", "z", lo.minZ + faceInset));
    sections.push(section(asCast, "face-y-min", "y", lo.minY + faceInset));
    sections.push(section(asCast, "face-y-max", "y", lo.maxY - faceInset));
    sections.push(section(asCast, "face-x-min", "x", lo.minX + faceInset));
    sections.push(section(asCast, "face-x-max", "x", lo.maxX - faceInset));
    var table = inserts.filter(i -> i.id == "tablePlate");
    if (kind == "bed") {
      for (z in [150.0, 400.0, 700.0]) sections.push(section(asCast, 'cut-z-$z', "z", z));
      for (y in [0.0, 220.0, 330.0]) sections.push(section(asCast, 'cut-y-$y', "y", y));
      var tablePosition:Array<Float> = table[0].position;
      sections.push(section(asCast, "cut-x-table", "x", tablePosition[0]));
    } else {
      for (z in [100.0, 250.0]) sections.push(section(asCast, 'cut-z-$z', "z", z));
      for (x in [0.0, 145.0]) sections.push(section(asCast, 'cut-x-$x', "x", x));
      for (y in [-40.0, 90.0]) sections.push(section(asCast, 'cut-y-$y', "y", y));
    }

    var lifting = [for (l in foundation.liftingPoints()) {id: l.id, insertId: l.insertId, holeId: l.holeId}];
    var feet:Array<Dynamic> = [];
    if (kind == "bed") for (f in HobberSkeleton.reference().feet()) feet.push({
      x: f.x,
      y: f.y,
      plate: [MineralCastBed.FOOT_PLATE_WIDTH, MineralCastBed.FOOT_PLATE_WIDTH, MineralCastBed.FOOT_PLATE_THICKNESS]
    });
    var datumScheme = body.foundationDatumScheme();
    var data = {
      source: "materia machinekit/haxe/src/machinekit/hobbing/" + (kind == "bed" ? "MineralCastBed.hx" : "MineralCastColumn.hx")
        + " (HobberFoundationSizing, FoundationLayout), as FlexsplineHobber installs it",
      units: "mm, kg; casting frame: x/y centred on the body, z = 0 at the cast underside, +z up",
      kind: kind,
      casting: {
        nominal: dims,
        bounds: boxJson(bounds),
        volumeMm3: volume,
        massKg: mass,
        centreOfMass: {
          var com = new machinekit.structural.MineralCasting(body.values().copy().setToken("buildStage", STAGE)).massProperties().centreOfMass;
          [com.x, com.y, com.z];
        },
        density: physical.density,
        material: byWeight.material,
        youngsModulusPa: physical.youngsModulusPa,
        dampingLossFactor: physical.dampingLossFactor,
        thermalExpansionPerK: physical.thermalExpansionPerK,
        thermalConductivityWPerMK: physical.thermalConductivityWPerMK,
        propertiesAssumed: physical.propertiesAssumed,
        steps: {
          body: '$kind-casting.step',
          withInserts: '$kind-casting-with-inserts.step',
          withInsertsLight: '$kind-casting-with-inserts-light.step'
        },
        datumScheme: datumScheme == null ? null : "declared",
        levellingFootPlates: feet,
        coolantReservedLitres: kind == "bed" ? 7.92 : null
      },
      cavities: cavities,
      insertRecesses: recesses,
      inserts: inserts,
      insertSteps: steps,
      liftingPoints: lifting,
      sections: sections
    };
    sys.io.File.saveContent('$dir/$kind-package.json', haxe.Json.stringify(data, null, " "));
    for (p in closers) p.close();
    cache.close();
    Sys.println('wrote $dir/$kind: ${mass} kg, ${inserts.length} inserts, ${cavities.length} cavities, ${sections.length} sections');
    // Profiling: MACHINEKIT_MEMO_REPORT=1 prints the sizing time and each build-time memo's hits and misses.
    if (Sys.getEnv("MACHINEKIT_MEMO_REPORT") != null) {
      Sys.println('sized $kind in ${Math.round(sized * 10) / 10} s');
      for (line in machinekit.cache.Memos.report()) Sys.println(line);
    }
  }
}
