package camkit.additive;

import cadkit.PlanarRegion;
import camkit.CamContour;
import camkit.CamPocketPlanner;
import cadkit.PlanarRegion.PlanarLoop;
import cadkit.PlanarRegion.PlanarSegment;
import cadkit.Geometry;
import cadkit.modeling.Part;
import toolpathkit.path.MoveKind;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.Point3;
import toolpathkit.path.PrintFeature;
import toolpathkit.path.Provenance;
import toolpathkit.path.ToolpathOp;
import toolpathkit.path.ToolpathProgram;
import toolpathkit.setup.Setup;
import toolpathkit.tool.ToolLibrary;

/** What a slice came to, for reports and tests. Lengths in metres, volumes in m^3. */
class SliceReport {
  public var layers:Int = 0;
  public var moves:Int = 0;
  public var retractions:Int = 0;
  public var extrudedLength:Float = 0.0;
  public var travelLength:Float = 0.0;
  /** Seconds the slice took. */
  public var seconds:Float = 0.0;
  public var depositedVolume:Float = 0.0;

  public function new() {}
}

class SliceResult {
  public final program:ToolpathProgram;
  public final report:SliceReport;

  public function new(program:ToolpathProgram, report:SliceReport) {
    this.program = program;
    this.report = report;
  }
}

/**
 * Slices a part into a toolpath program, from the B-rep: each layer is a planar section of the solid,
 * perimeters are offsets of it, top and bottom skins are what the layers above and below do not cover,
 * and infill is scanlines across the rest. Boundaries stay exact lines and arcs, so a circle is an arc.
 *
 * The part is placed on the bed at z = 0, centred on the profile's bed centre. Geometry is in the
 * part's millimetres; the program is in metres.
 */
class Slicer {
  public static function slice(part:Part, settings:SlicerSettings, ?profile:PrinterProfile, ?objectName:String = "part"):SliceResult {
    if (part == null || settings == null) throw "slicing needs a part and settings";
    settings.validate();
    var began = Sys.time();
    var machine = profile == null ? PrinterProfile.bed220() : profile;
    var run = new SliceRun(part, settings, machine, objectName);
    try {
      var program = run.run();
      run.report.seconds = Sys.time() - began;
      run.close();
      return new SliceResult(program, run.report);
    } catch (error:Dynamic) {
      run.close();
      throw error;
    }
  }
}

private enum Group {
  PerimeterLoops(k:Int);
  SolidFill;
  SparseFill;
}

/** One slice in progress: owns the regions it makes and closes them. */
private class SliceRun {
  public final report:SliceReport = new SliceReport();
  final part:Part;
  final settings:SlicerSettings;
  final profile:PrinterProfile;
  final objectName:String;
  final regions:Array<PlanarRegion> = [];
  var placed:Null<cadkit.Shape> = null;
  final ops:Array<ToolpathOp> = [];
  // Nozzle and extruder state; the nozzle starts where homing leaves it.
  var x = 0.0;
  var y = 0.0;
  var z = 0.0;
  var retracted = false;
  var hasDeposited = false;
  var beadWidth = 0.0;
  var beadHeight = 0.0;
  var fan = 0.0;
  // The layer being printed: its section as polygons, for deciding whether a travel stays inside the part.
  var inside:Array<Array<Float>> = [];
  var span:Provenance = Provenance.cam(0);

  public function new(part:Part, settings:SlicerSettings, profile:PrinterProfile, objectName:String) {
    this.part = part;
    this.settings = settings;
    this.profile = profile;
    this.objectName = objectName;
    x = profile.home.x; y = profile.home.y; z = profile.home.z;
  }

  public function close():Void {
    for (region in regions) region.close();
    regions.resize(0);
    if (placed != null) placed.close();
    placed = null;
  }

  function keep(region:PlanarRegion):PlanarRegion {
    regions.push(region);
    return region;
  }

  // ---- the run ----------------------------------------------------------------------------

  public function run():ToolpathProgram {
    var bounds = part.shape.bounds();
    var low = bounds.get_min(), high = bounds.get_max();
    var millimetres = 1000.0;
    placed = part.shape.translate(Geometry.vec3(profile.bedCenterX * millimetres - (low.get_x() + high.get_x()) * 0.5,
      profile.bedCenterY * millimetres - (low.get_y() + high.get_y()) * 0.5, -low.get_z()));
    var height = (high.get_z() - low.get_z()) / millimetres;
    var layers = Std.int(Math.max(1, Math.round((height - settings.firstLayerHeight) / settings.layerHeight) + 1));
    report.layers = layers;

    var s = settings;
    var perimeterCount = s.perimeters;
    var sections:Array<PlanarRegion> = [], cores:Array<PlanarRegion> = [];
    for (layer in 0...layers) {
      var top = layerTop(layer), thickness = layer == 0 ? s.firstLayerHeight : s.layerHeight;
      var section = keep(PlanarRegion.section(placed, (top - thickness / 2.0) * millimetres));
      sections.push(section);
      // The infill region starts where the last perimeter's share of the layer ends, at this layer's bead spacing
      // (a first layer of its own height has its own).
      var spacing = s.beadSpacing(thickness);
      var coreInset = s.lineWidth / 2.0 + (perimeterCount - 1) * spacing + spacing / 2.0;
      cores.push(perimeterCount == 0 || section.isEmpty() ? section : keep(section.offset(-coreInset * millimetres)));
    }

    startSequence();
    for (layer in 0...layers) printLayer(layer, sections, cores);
    ops.push(ToolpathOp.PrintObject(objectName, false, Provenance.cam(layers, "object")));
    endSequence();
    var program = new ToolpathProgram(ops, new ToolLibrary(), [new Setup("1", new Point3(0, 0, 0))]);
    report.depositedVolume = program.depositedVolume();
    return program;
  }

  function layerTop(layer:Int):Float return settings.firstLayerHeight + layer * settings.layerHeight;

  // ---- start and end -----------------------------------------------------------------------

  function startSequence():Void {
    var start = Provenance.cam(0, "start");
    ops.push(ToolpathOp.Heat("bed", settings.bedTemperature, false, start));
    ops.push(ToolpathOp.Heat("hotend", settings.hotendTemperature, false, start));
    ops.push(ToolpathOp.Heat("bed", settings.bedTemperature, true, start));
    ops.push(ToolpathOp.Heat("hotend", settings.hotendTemperature, true, start));
    // A purge line along the front of the bed: one bead, laid at the purge height.
    var from = profile.purgeStart, to = profile.purgeEnd;
    travelTo(from.x, from.y, from.z, start);
    unretract(start);
    if (profile.purgePrimeLength > 0.0)
      ops.push(ToolpathOp.FilamentMove(profile.purgePrimeLength, settings.retractSpeed, start));
    var purgeWidth = profile.purgeWidth, purgeHeight = profile.purgeHeight;
    var area = purgeHeight * (purgeWidth - purgeHeight) + Math.PI * purgeHeight * purgeHeight / 4.0;
    ops.push(ToolpathOp.Deposit(area, purgeWidth, purgeHeight, start));
    ops.push(ToolpathOp.Feature(Skirt, start));
    ops.push(ToolpathOp.Move(Cut, PathGeometry.Line(from, to), profile.purgeSpeed, settings.pathTolerance, start));
    hasDeposited = true;
    report.moves += 1;
    report.extrudedLength += from.distanceTo(to);
    x = to.x; y = to.y; z = to.z;
    beadWidth = 0.0; beadHeight = 0.0;
    retract(start);
  }

  function endSequence():Void {
    var end = Provenance.cam(report.layers, "end");
    ops.push(ToolpathOp.DepositOff(end));
    beadWidth = 0.0; beadHeight = 0.0;
    retract(end);
    ops.push(ToolpathOp.Fan("part", 0.0, end));
    ops.push(ToolpathOp.Heat("hotend", 0.0, false, end));
    ops.push(ToolpathOp.Heat("bed", 0.0, false, end));
    var lifted = new Point3(x, y, z + profile.parkLift);
    ops.push(ToolpathOp.Move(Rapid, PathGeometry.Line(new Point3(x, y, z), lifted), settings.travelSpeed, 0.0, end));
    ops.push(ToolpathOp.Move(Rapid, PathGeometry.Line(lifted, new Point3(profile.home.x, profile.home.y, lifted.z)), settings.travelSpeed, 0.0, end));
    ops.push(ToolpathOp.End(end));
    report.moves += 2;
  }

  // ---- layers ---------------------------------------------------------------------------------

  function printLayer(layer:Int, sections:Array<PlanarRegion>, cores:Array<PlanarRegion>):Void {
    var s = settings;
    var section = sections[layer];
    var top = layerTop(layer), thickness = layer == 0 ? s.firstLayerHeight : s.layerHeight;
    var marker = Provenance.cam(layer + 1, "layer");
    ops.push(ToolpathOp.Layer(layer, top, marker));
    var duty = layer >= s.fanFromLayer ? s.fanDuty : 0.0;
    if (duty != fan) {
      ops.push(ToolpathOp.Fan("part", duty, marker));
      fan = duty;
    }
    if (section.isEmpty()) {
      if (layer == 0) ops.push(ToolpathOp.PrintObject(objectName, true, marker));
      return;
    }
    inside = polygons(section.loops(), 0.00005);
    var millimetres = 1000.0;
    var spacing = s.beadSpacing(thickness);

    if (layer == 0 && s.skirtLoops > 0) {
      var skirt:Array<PlanarLoop> = [];
      for (k in 0...s.skirtLoops) {
        var grown = keep(section.offset((s.skirtDistance + s.lineWidth / 2.0 + k * spacing) * millimetres));
        for (loop in grown.loops()) if (loop.outer) skirt.push(loop);
      }
      printLoops(skirt, top, thickness, PrintFeature.Skirt, layer, "skirt", s.skirtSpeed);
    }
    // The skirt belongs to no object: cancelling the part must leave it.
    if (layer == 0) ops.push(ToolpathOp.PrintObject(objectName, true, Provenance.cam(1, "object")));

    // Perimeters from the outside in.
    for (k in 0...s.perimeters) {
      var offset = s.lineWidth / 2.0 + k * spacing;
      var ring = keep(section.offset(-offset * millimetres));
      if (ring.isEmpty()) break;
      printLoops(ring.loops(), top, thickness, k == 0 ? PrintFeature.ExternalPerimeter : PrintFeature.Perimeter, layer,
        k == 0 ? "external-perimeter" : "perimeter", layer == 0 ? s.firstLayerSpeed : (k == 0 ? s.externalPerimeterSpeed : s.perimeterSpeed));
    }

    // What is inside the perimeters: solid where the layers above or below do not cover it, sparse elsewhere.
    var core = cores[layer];
    if (core.isEmpty()) return;
    var covered:Null<PlanarRegion> = null;
    var allSolid = s.infillDensity >= 0.999;
    if (!allSolid) covered = coveredByNeighbours(layer, cores);
    var solid = allSolid ? core : (covered == null ? core : keep(core.cut(covered)));
    var sparse:Null<PlanarRegion> = null;
    if (!allSolid && covered != null && s.infillDensity > 0.0) sparse = keep(core.common(covered));
    var angle = s.infillAngles[layer % s.infillAngles.length];
    if (!solid.isEmpty()) {
      var topmost = isTop(layer, cores);
      printFill(solid, angle, spacing, top, thickness, topmost ? PrintFeature.TopSolidInfill : PrintFeature.SolidInfill,
        layer, "solid-infill", layer == 0 ? s.firstLayerSpeed : (topmost ? s.topSolidInfillSpeed : s.solidInfillSpeed));
    }
    if (sparse != null && !sparse.isEmpty())
      printFill(sparse, angle, s.lineWidth / s.infillDensity, top, thickness, PrintFeature.Infill, layer, "infill",
        layer == 0 ? s.firstLayerSpeed : s.infillSpeed);
  }

  /** Is layer `layer` the last of its stack, with nothing above it? */
  function isTop(layer:Int, cores:Array<PlanarRegion>):Bool return layer + 1 >= cores.length || cores[layer + 1].isEmpty();

  /** The part of the core that the `topLayers` above and the `bottomLayers` below all cover; null when none of it is. */
  function coveredByNeighbours(layer:Int, cores:Array<PlanarRegion>):Null<PlanarRegion> {
    var windowStart = layer - settings.bottomLayers, windowEnd = layer + settings.topLayers;
    if (windowStart < 0 || windowEnd >= cores.length) return null;
    var common:Null<PlanarRegion> = null;
    for (other in windowStart...windowEnd + 1) {
      if (other == layer) continue;
      if (cores[other].isEmpty()) return null;
      common = common == null ? cores[other] : keep(common.common(cores[other]));
      if (common.isEmpty()) return null;
    }
    return common;
  }

  // ---- printing paths --------------------------------------------------------------------------

  function printLoops(loops:Array<PlanarLoop>, top:Float, thickness:Float, feature:PrintFeature, layer:Int,
      name:String, speed:Float):Void {
    if (loops.length == 0) return;
    var pending = loops.copy();
    var scale = 0.001;
    ops.push(ToolpathOp.Feature(feature, Provenance.cam(layer + 1, name)));
    var tag = Provenance.cam(layer + 1, name);
    while (pending.length > 0) {
      // The loop with a vertex nearest the nozzle, entered at that vertex.
      var best = -1, bestIndex = 0, bestDistance = Math.POSITIVE_INFINITY;
      for (index in 0...pending.length) {
        var loop = pending[index];
        for (at in 0...loop.segments.length) {
          var d = distance2(loop.segments[at].x0 * scale, loop.segments[at].y0 * scale);
          if (d < bestDistance) { bestDistance = d; best = index; bestIndex = at; }
        }
      }
      var loop = pending.splice(best, 1)[0].startingAt(bestIndex);
      if (loop.length() * scale < 2.0 * settings.lineWidth) continue;
      var first = loop.segments[0];
      travelTo(first.x0 * scale, first.y0 * scale, top, tag);
      extrude(loop.segments, top, thickness, speed, tag, scale);
    }
  }

  function printFill(region:PlanarRegion, angle:Float, spacing:Float, top:Float, thickness:Float, feature:PrintFeature,
      layer:Int, name:String, speed:Float):Void {
    var scale = 0.001;
    var tag = Provenance.cam(layer + 1, name);
    var lines = fillLines(region, angle, spacing, top);
    if (lines.length == 0) return;
    ops.push(ToolpathOp.Feature(feature, tag));
    var pending = lines;
    while (pending.length > 0) {
      var best = -1, bestEnd = false, bestDistance = Math.POSITIVE_INFINITY;
      for (index in 0...pending.length) {
        var line = pending[index];
        var d0 = distance2(line[0], line[1]), d1 = distance2(line[2], line[3]);
        if (d0 < bestDistance) { bestDistance = d0; best = index; bestEnd = false; }
        if (d1 < bestDistance) { bestDistance = d1; best = index; bestEnd = true; }
      }
      var line = pending.splice(best, 1)[0];
      var a = bestEnd ? [line[2], line[3]] : [line[0], line[1]], b = bestEnd ? [line[0], line[1]] : [line[2], line[3]];
      travelTo(a[0], a[1], top, tag);
      var segment = new PlanarSegment(false, a[0] / scale, a[1] / scale, b[0] / scale, b[1] / scale, 0, 0, 0, 0, 0);
      extrude([segment], top, thickness, speed, tag, scale);
    }
  }

  /** Scanline passes across every face of the region, as [x0, y0, x1, y1] in metres. */
  function fillLines(region:PlanarRegion, angle:Float, spacing:Float, top:Float):Array<Array<Float>> {
    var scale = 0.001, cosine = Math.cos(angle), sine = Math.sin(angle);
    var faces = new Map<Int, Array<PlanarLoop>>();
    var order:Array<Int> = [];
    for (loop in region.loops(0.01)) {
      if (!faces.exists(loop.face)) { faces.set(loop.face, []); order.push(loop.face); }
      faces.get(loop.face).push(loop);
    }
    var lines:Array<Array<Float>> = [];
    var minimum = settings.lineWidth * 0.5;
    for (face in order) {
      var outer:Null<CamContour> = null, holes:Array<CamContour> = [];
      for (loop in faces.get(face)) {
        // Rotate so the scanlines run along x: x' = x cos + y sin, y' = -x sin + y cos.
        var points = [for (point in sample(loop, 0.00001)) new Point3(point[0] * cosine + point[1] * sine, -point[0] * sine + point[1] * cosine, 0.0)];
        var contour = new CamContour(points, true);
        if (loop.outer) outer = contour; else holes.push(contour);
      }
      if (outer == null) continue;
      for (pass in CamPocketPlanner.fill(outer, spacing, holes)) {
        if (pass.right - pass.left < minimum) continue;
        // Back to the bed's frame, in metres.
        lines.push([pass.left * cosine - pass.y * sine, pass.left * sine + pass.y * cosine,
          pass.right * cosine - pass.y * sine, pass.right * sine + pass.y * cosine]);
      }
    }
    return lines;
  }

  /** Vertices (metres) of a loop, arcs cut into chords within `tolerance` millimetres of the arc. */
  static function sample(loop:PlanarLoop, tolerance:Float):Array<Array<Float>> {
    var points:Array<Array<Float>> = [];
    var scale = 0.001;
    for (segment in loop.segments) {
      points.push([segment.x0 * scale, segment.y0 * scale]);
      if (!segment.arc) continue;
      var steps = Std.int(Math.max(2, Math.ceil(Math.abs(segment.sweep) / (2.0 * Math.acos(Math.max(0.0, 1.0 - tolerance / (segment.radius * scale)))))));
      for (step in 1...steps) {
        var angle = segment.startAngle + segment.sweep * step / steps;
        points.push([(segment.cx + segment.radius * Math.cos(angle)) * scale, (segment.cy + segment.radius * Math.sin(angle)) * scale]);
      }
    }
    return points;
  }

  static function polygons(loops:Array<PlanarLoop>, tolerance:Float):Array<Array<Float>> {
    var result:Array<Array<Float>> = [];
    for (loop in loops) {
      var flat:Array<Float> = [];
      for (point in sample(loop, tolerance)) { flat.push(point[0]); flat.push(point[1]); }
      result.push(flat);
    }
    return result;
  }

  /** Is the point inside the section, by the even-odd rule over all its loops? */
  function contains(px:Float, py:Float):Bool {
    var odd = false;
    for (polygon in inside) {
      var count = Std.int(polygon.length / 2);
      var j = count - 1;
      for (i in 0...count) {
        var xi = polygon[2 * i], yi = polygon[2 * i + 1], xj = polygon[2 * j], yj = polygon[2 * j + 1];
        if ((yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi) odd = !odd;
        j = i;
      }
    }
    return odd;
  }

  // ---- moves ------------------------------------------------------------------------------------

  function distance2(px:Float, py:Float):Float return (px - x) * (px - x) + (py - y) * (py - y);

  /** A travel to (tx, ty) at height `tz`, retracting first when it is long and leaves the part. */
  function travelTo(tx:Float, ty:Float, tz:Float, tag:Provenance):Void {
    var length = Math.sqrt((tx - x) * (tx - x) + (ty - y) * (ty - y));
    if (length < 1e-9 && Math.abs(tz - z) < 1e-9) return;
    if (!retracted && length > settings.retractMinTravel && leavesPart(tx, ty)) retract(tag);
    var from = new Point3(x, y, z);
    var lift = retracted ? settings.zHop : 0.0;
    if (lift > 0.0) {
      var up = new Point3(x, y, z + lift), across = new Point3(tx, ty, tz + lift);
      ops.push(ToolpathOp.Move(Rapid, PathGeometry.Line(from, up), settings.travelSpeed, 0.0, tag));
      ops.push(ToolpathOp.Move(Rapid, PathGeometry.Line(up, across), settings.travelSpeed, settings.pathTolerance, tag));
      ops.push(ToolpathOp.Move(Rapid, PathGeometry.Line(across, new Point3(tx, ty, tz)), settings.travelSpeed, 0.0, tag));
      report.moves += 3;
    } else {
      ops.push(ToolpathOp.Move(Rapid, PathGeometry.Line(from, new Point3(tx, ty, tz)), settings.travelSpeed, settings.pathTolerance, tag));
      report.moves += 1;
    }
    report.travelLength += length;
    x = tx; y = ty; z = tz;
  }

  /** Does the straight travel from the nozzle to (tx, ty) leave the printed section? */
  function leavesPart(tx:Float, ty:Float):Bool {
    var length = Math.sqrt((tx - x) * (tx - x) + (ty - y) * (ty - y));
    var steps = Std.int(Math.max(2, Math.ceil(length / 0.0005)));
    for (step in 0...steps + 1) {
      var t = step / steps;
      if (!contains(x + (tx - x) * t, y + (ty - y) * t)) return true;
    }
    return false;
  }

  function retract(tag:Provenance):Void {
    if (!hasDeposited || retracted || settings.retractLength <= 0.0) return;
    ops.push(ToolpathOp.FilamentMove(-settings.retractLength, settings.retractSpeed, tag));
    retracted = true;
    report.retractions += 1;
  }

  function unretract(tag:Provenance):Void {
    if (retracted) {
      // A retraction made before a travel is undone where the next bead starts.
      ops.push(ToolpathOp.FilamentMove(settings.retractLength, settings.retractSpeed, tag));
      retracted = false;
    }
  }

  function extrude(segments:Array<PlanarSegment>, top:Float, thickness:Float, speed:Float, tag:Provenance, scale:Float):Void {
    unretract(tag);
    var width = settings.lineWidth;
    if (beadWidth != width || beadHeight != thickness) {
      ops.push(ToolpathOp.Deposit(settings.beadArea(thickness), width, thickness, tag));
      beadWidth = width; beadHeight = thickness;
    }
    for (segment in segments) {
      var a = new Point3(segment.x0 * scale, segment.y0 * scale, top);
      var b = new Point3(segment.x1 * scale, segment.y1 * scale, top);
      var geometry = segment.arc ?
        PathGeometry.Arc(new Point3(segment.cx * scale, segment.cy * scale, top), segment.radius * scale, segment.startAngle, segment.sweep) :
        PathGeometry.Line(a, b);
      var length = segment.length() * scale;
      if (length < 1e-9) continue;
      ops.push(ToolpathOp.Move(Cut, geometry, speed, settings.pathTolerance, tag));
      hasDeposited = true;
      report.moves += 1;
      report.extrudedLength += length;
      x = b.x; y = b.y; z = top;
    }
  }
}
