import cadkit.Shape;
import machinekit.component.ComponentValues;
import machinekit.motion.SealedScalePart;
import machinekit.motion.YuhengScaleCad;

/**
 * `SealedScalePart` (JFT-40) against the maker's STEP models of ML 70, 120 and 220 (`YuhengScaleCad`, not committed): length, housing section, fixing holes and
 * counterbores, volume, head body, mounting slots, strain relief, cable stub and air ports, all measured from the files by slicing their solids and cylinders.
 * Skipped with a message for a file that is absent. The STEP is one assembly of simplified solids, so the volume is compared as a bound (the model is the
 * full envelope of the section, the STEP has the groove and the sealing lips), not as an equality.
 *
 * STEP frame to part frame (headSide +x): z = Z - housing start; y = (flush face X max) - X, from the free face toward the seat; x = Y - the hole axis's Y.
 */
class SealedScaleStepTests {
  static function check(condition:Bool, message:String):Void if (!condition) throw 'Sealed scale STEP: $message';
  static function near(a:Float, b:Float, message:String, tolerance:Float = 0.3):Void check(Math.abs(a - b) <= tolerance, '$message: STEP $a vs model $b (tolerance $tolerance)');

  /** How many of the three files were compared. */
  public static function run():Int {
    var compared = 0;
    for (ml in YuhengScaleCad.lengths()) {
      if (!YuhengScaleCad.available(ml)) {
        Sys.println('Sealed scale STEP: ML $ml skipped, ${YuhengScaleCad.path(ml)} is not in the supplier library (machinekit/scripts/prepare-yuheng-cad)');
        continue;
      }
      compare(ml);
      compared++;
    }
    return compared;
  }

  static function compare(ml:Float):Void {
    var cad = YuhengScaleCad.load(ml);
    check(cad != null && cad.sha256 == YuhengScaleCad.sha256(ml), "the file is the pinned one");
    var part = cad.geometry(), step = part.shape;
    var housing = new SealedScalePart(new ComponentValues().setNumber("measuringLength", ml).setToken("variant", "housing"));
    var head = new SealedScalePart(new ComponentValues().setNumber("measuringLength", ml).setToken("variant", "head"));
    var solids = [for (i in 0...step.subshapeCount(CadKit.ShapeKind.Solid)) step.subshape(CadKit.ShapeKind.Solid, i)];
    // The housing's extrusion is the largest solid; solids centred above its reading face are the head's.
    var big = solids[0];
    for (s in solids) if (s.volume() > big.volume()) big = s;
    var bb = big.bounds(), readingY = bb.get_max().get_y(), flushX = bb.get_max().get_x();
    var housingSolids = [for (s in solids) if ((s.bounds().get_min().get_y() + s.bounds().get_max().get_y()) / 2 < readingY) s];
    var headSolids = [for (s in solids) if ((s.bounds().get_min().get_y() + s.bounds().get_max().get_y()) / 2 >= readingY) s];
    var z0 = 1e9, z1 = -1e9, y0 = 1e9, volume = 0.0;
    for (s in housingSolids) {
      var b = s.bounds();
      z0 = Math.min(z0, b.get_min().get_z());
      z1 = Math.max(z1, b.get_max().get_z());
      y0 = Math.min(y0, b.get_min().get_y());
      volume += s.volume();
    }
    var m = housing.series;
    // Housing: overall length, section, thickness.
    near(z1 - z0, housing.overallLength(), 'ML $ml housing length (ML + 138)');
    near(bb.get_max().get_y() - y0, m.housingWidth, 'ML $ml housing width across the travel');
    near(bb.get_max().get_x() - bb.get_min().get_x(), m.housingThickness, 'ML $ml extrusion thickness');
    // Fixing holes: cylinders along X of Ø9.6, 11.5 from each end, 13.5 above the bottom face, counterbores Ø15.
    var holes = [], bores = [];
    for (s in housingSolids) for (c in cylinders(s)) if (Math.abs(Math.abs(c.d[0]) - 1) < 1e-6) {
      if (Math.abs(c.r - 4.8) < 1e-6) holes.push(c);
      if (Math.abs(c.r - 7.5) < 1e-6) bores.push(c);
    }
    var ends = unique([for (c in holes) round(c.o[2] - z0)]);
    ends.sort(Reflect.compare);
    check(ends.length == 2, 'ML $ml has two fixing holes, found ${ends.length}');
    near(ends[0], 11.5, 'ML $ml first hole from the start end');
    near(ends[1], housing.overallLength() - 11.5, 'ML $ml second hole from the far end');
    for (c in holes) {
      near(2 * c.r, (m.holeDiameter ?? 0), 'ML $ml hole diameter', 0.01);
      near(c.o[1] - y0, (m.holeOffset ?? 0), 'ML $ml hole axis above the bottom face');
    }
    for (c in bores) {
      near(2 * c.r, (m.holeCounterbore ?? 0), 'ML $ml counterbore diameter', 0.01);
      near(c.max[0] - c.min[0], (m.holeCounterboreDepth ?? 0), 'ML $ml counterbore depth', 0.2);
    }
    check(bores.length >= 4, 'ML $ml has a counterbore on each face of each hole');
    // Volume: the model's section is the envelope, so it bounds the STEP's solids from above, within a tenth.
    var model = housing.geometry(), mv = model.volume();
    check(mv >= volume && mv <= volume * 1.12, 'ML $ml housing volume: model $mv against STEP $volume (the model is the envelope, up to 12 % above)');
    model.close();
    // Head: body size, its gap above the housing, the slots, the strain relief and the cable.
    var body = headSolids[0];
    for (s in headSolids) if (s.volume() > body.volume()) body = s;
    var hb = body.bounds();
    near(hb.get_max().get_z() - hb.get_min().get_z(), m.headLength, 'ML $ml head body length');
    near(hb.get_max().get_x() - hb.get_min().get_x(), m.headThickness, 'ML $ml head body thickness');
    near(hb.get_max().get_y() - readingY, housing.gap() + m.headWidth, 'ML $ml head top above the reading face (gap + height)');
    var slotsAt = [], slotWidth = [], boreWidth = [], slotY = [], boreTop:Array<Float> = [];
    for (c in cylinders(body)) if (Math.abs(Math.abs(c.d[0]) - 1) < 1e-6) {
      if (Math.abs(c.r - 2.3) < 1e-6) {
        slotsAt.push(c.o[2] - hb.get_min().get_z());
        slotY.push(c.o[1] - readingY - housing.gap());
        slotWidth.push(2 * c.r);
      }
      if (Math.abs(2 * c.r - (m.headCounterbore ?? 0)) < 1e-6) {
        boreWidth.push(c.max[0] - c.min[0]);
        boreTop.push(c.o[1] + c.r);
      }
    }
    // The counterbores are the maker's: their diameter is the datasheet's, and the wall they leave to the head's top edge is
    // measured here rather than assumed (it is thin on this head).
    // The STEP splits each arc face in two, so the four arcs (two slots, both ends) appear as four or eight faces.
    check(boreWidth.length == 4 || boreWidth.length == 8, 'ML $ml head has a counterbore arc at each end of both slots, found ${boreWidth.length}');
    var thinnest = Math.POSITIVE_INFINITY;
    for (top in boreTop) thinnest = Math.min(thinnest, hb.get_max().get_y() - top);
    Sys.println('ML $ml head counterbore wall to the top edge (maker STEP): ' + Math.round(thinnest * 100) / 100 + ' mm');
    slotsAt.sort(Reflect.compare);
    check(slotsAt.length == 4, 'ML $ml head has two slots of two arcs, found ${slotsAt.length} arcs');
    near(slotsAt[1] - slotsAt[0], m.headHoleSlot, 'ML $ml slot length (arc centres)', 0.01);
    var pitch = (slotsAt[2] + slotsAt[3]) / 2 - (slotsAt[0] + slotsAt[1]) / 2;
    near(pitch, m.headHolePitch, 'ML $ml slot pitch', 0.01);
    var centre = ((slotsAt[0] + slotsAt[1]) / 2 + (slotsAt[2] + slotsAt[3]) / 2) / 2;
    // The model's pair centre from the body's cable end: its slot offset from the finger less the body's start.
    var modelCentre = (head.headHoleAlong(-1) + head.headHoleAlong(1)) / 2 - head.bodyBox()[4];
    near(centre, modelCentre, 'ML $ml slot pair centre from the body start', 0.01);
    for (w in slotWidth) near(w, (m.headHoleDiameter ?? 0), 'ML $ml slot width', 0.01);
    for (y in slotY) near(y, m.headHoleHeight, 'ML $ml slot axis above the gap face', 0.01);
    for (w in boreWidth) near(w, (m.headCounterboreDepth ?? 0), 'ML $ml head counterbore depth', 0.2);
    // Strain relief (the head solid reaching furthest toward the start end besides the cable) and cable stub.
    var boot:Shape = null, cable:Shape = null;
    for (s in headSolids) {
      var b = s.bounds();
      if (s != body && Math.abs(b.get_max().get_y() - b.get_min().get_y() - m.bootSize) < 0.05 && Math.abs(b.get_max().get_x() - b.get_min().get_x() - m.bootSize) < 0.05) boot = s;
    }
    check(boot != null, 'ML $ml head has a ${m.bootSize} mm square strain relief');
    near(hb.get_min().get_z() - boot.bounds().get_min().get_z(), m.bootLength, 'ML $ml strain relief beyond the body', 0.05);
    var lowest = 1e9;
    for (s in headSolids) if (s.bounds().get_min().get_z() < lowest) {
      lowest = s.bounds().get_min().get_z();
      cable = s;
    }
    var cb = cable.bounds();
    near(hb.get_min().get_z() - lowest, m.bootLength + m.cableStraight, 'ML $ml cable stub end beyond the body');
    var stubDiameter = cb.get_max().get_x() - cb.get_min().get_x();
    check(stubDiameter <= m.cableDiameter && m.cableDiameter - stubDiameter < 1, 'ML $ml cable: STEP stub Ø$stubDiameter, the model keeps the cable datasheet\'s Ø${m.cableDiameter}');
    near((cb.get_max().get_y() + cb.get_min().get_y()) / 2 - readingY - housing.gap(), m.headWidth / 2, 'ML $ml cable axis at the head\'s mid-height', 0.1);
    // Air ports: M5 plugs in both end faces, 7 mm below the reading face, centred on the thickness.
    var plugs = [for (s in housingSolids) if (isPlug(s, readingY)) s];
    check(plugs.length == 2, 'ML $ml has an M5 plug in each end face, found ${plugs.length}');
    for (s in plugs) {
      var b = s.bounds();
      near(readingY - (b.get_max().get_y() + b.get_min().get_y()) / 2, m.airFromReadingFace, 'ML $ml air port below the reading face', 0.1);
      near((b.get_max().get_x() + b.get_min().get_x()) / 2, flushX - m.housingThickness / 2, 'ML $ml air port centred on the thickness', 0.3);
    }
    for (s in solids) s.close();
    part.close();
    cad.close();
    Sys.println('Sealed scale STEP: ML $ml compared (length ${housing.overallLength()}, holes, counterbores, head, slots, boot, cable, air ports)');
  }

  /** An M5 plug: a 6 mm round solid, 5.5 mm long, flush with a housing end. */
  static function isPlug(s:Shape, readingY:Float):Bool {
    var b = s.bounds();
    var w = b.get_max().get_x() - b.get_min().get_x(), h = b.get_max().get_y() - b.get_min().get_y(), l = b.get_max().get_z() - b.get_min().get_z();
    return Math.abs(w - 6) < 0.1 && Math.abs(h - 6) < 0.1 && Math.abs(l - 5.5) < 0.1 && b.get_max().get_y() < readingY;
  }

  static function round(v:Float):Float return Math.round(v * 100) / 100;

  static function unique(values:Array<Float>):Array<Float> {
    var out:Array<Float> = [];
    for (v in values) if ([for (o in out) if (Math.abs(o - v) < 0.05) o].length == 0) out.push(v);
    return out;
  }

  /** The cylindrical faces of a solid: radius, axis point and direction, bounds. */
  static function cylinders(solid:Shape):Array<{r:Float, o:Array<Float>, d:Array<Float>, min:Array<Float>, max:Array<Float>}> {
    var out = [];
    for (j in 0...solid.subshapeCount(CadKit.ShapeKind.Face)) {
      var face = solid.subshape(CadKit.ShapeKind.Face, j);
      if (face.surfaceKind() == CadKit.SurfaceKind.Cylinder) {
        var a = face.faceAxis(), o = a.get_origin(), d = a.get_direction(), b = face.bounds();
        out.push({r: round(a.get_radius()), o: [o.get_x(), o.get_y(), o.get_z()], d: [d.get_x(), d.get_y(), d.get_z()],
          min: [b.get_min().get_x(), b.get_min().get_y(), b.get_min().get_z()], max: [b.get_max().get_x(), b.get_max().get_y(), b.get_max().get_z()]});
      }
      face.close();
    }
    return out;
  }
}
