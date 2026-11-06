package machinekit.structural;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.VolumeProperties;
import materia.casting.Box3;

/**
 * Volume properties of material given as disjoint orthogonal boxes less a set of solids (embedded hardware, drilled
 * receivers), without building the material's B-rep. The boxes integrate in closed form; each solid removes only its
 * share inside the boxes, found box by box: a solid inside one box removes all of itself, one clear of a box none, and
 * only a solid crossing a box face needs a boolean against that one box. Overlapping solids would be removed twice, so
 * an overlap between two of them inside the material makes the result unavailable (null) and the caller integrates
 * the real solid instead.
 */
class BoxMaterialVolume {
  /** Shared volume (mm³) below which two solids only touch: the casting cavity contact threshold. */
  static inline final CONTACT_VOLUME = 1e-8;

  /**
   * What each keyed solid contributed before: its bounds and integrated properties, whether two of them overlap, and its
   * share inside a material box. Sizing integrates many layouts holding the same cavities and anchor screws at the same
   * places; a keyed solid is built only when one of these is not yet known.
   */
  static final keptBounds = new machinekit.cache.Memo<PartBox>("box-volume-bounds", 2048);
  static final keptWhole = new machinekit.cache.Memo<cadkit.PhysicalProperties>("box-volume-whole", 2048);
  static final keptOverlaps = new machinekit.cache.Memo<Bool>("box-volume-overlaps", 2048);
  static final keptShares = new machinekit.cache.Memo<Null<cadkit.PhysicalProperties>>("box-volume-shares", 2048);

  /** `boxes` must be pairwise disjoint (as `BoxAlgebra.material` returns them). The caller owns `tools`. */
  public static function of(boxes:Array<Box3>, tools:Array<Part>):Null<VolumeProperties> return integrate(boxes,
    [for (tool in tools) {key: null, build: () -> tool}], false);

  /**
   * The same for solids each named by `key` (its shape and pose, or null for one not kept) and built by `build` only when
   * something about it is not yet known. The solids built here are closed here.
   */
  public static function ofKeyed(boxes:Array<Box3>, tools:Array<KeyedSolid>):Null<VolumeProperties> return integrate(boxes,
    tools, true);

  static function integrate(boxes:Array<Box3>, tools:Array<KeyedSolid>, owned:Bool):Null<VolumeProperties> {
    if (boxes.length == 0) return null;
    var built:Array<Null<Part>> = [for (_ in tools) null];
    function solid(index:Int):Part {
      if (built[index] == null) built[index] = tools[index].build();
      return built[index];
    }
    try {
      var result = sum(boxes, tools, solid);
      if (owned) for (part in built) if (part != null) part.close();
      return result;
    } catch (error:Dynamic) {
      if (owned) for (part in built) if (part != null) part.close();
      throw error;
    }
  }

  static function sum(boxes:Array<Box3>, tools:Array<KeyedSolid>, solid:Int->Part):Null<VolumeProperties> {
    var sums = new Moments(boxes[0].x + boxes[0].width / 2, boxes[0].y + boxes[0].depth / 2,
      boxes[0].z + boxes[0].height / 2);
    for (box in boxes) sums.addBox(box);
    var toolBoxes:Array<PartBox> = [];
    for (index in 0...tools.length) {
      toolBoxes.push(keptBounds.get(tools[index].key, () -> PosedParts.boxOf(solid(index))));
    }
    for (i in 0...tools.length) for (j in i + 1...tools.length) {
      if (PosedParts.apart(toolBoxes[i], toolBoxes[j], 0)) continue;
      var a = tools[i].key, b = tools[j].key, pair = a == null || b == null ? null : a + "#" + b;
      var overlap = keptOverlaps.get(pair, () -> {
        var shared = solid(i).intersect(solid(j));
        var volume = shared.volume();
        shared.close();
        return volume > CONTACT_VOLUME;
      });
      if (overlap) return null;
    }
    for (t in 0...tools.length) {
      var key = tools[t].key, bounds = toolBoxes[t];
      for (box in boxes) {
        if (PosedParts.apart(bounds, limits(box), 0)) continue;
        if (inside(bounds, box)) {
          sums.addSolid(keptWhole.get(key, () -> solid(t).massProperties()), -1);
          continue;
        }
        var cellKey = key == null ? null : key + "#" + box.x + "," + box.y + "," + box.z + "," + box.width + ","
          + box.depth + "," + box.height;
        var share = keptShares.get(cellKey, () -> {
          var cell = Part.box(box.width, box.depth, box.height);
          var placed = cell.translated(new Vector(box.x + box.width / 2, box.y + box.depth / 2, box.z));
          cell.close();
          var common = solid(t).intersect(placed);
          placed.close();
          var share:Null<cadkit.PhysicalProperties> = null;
          try {
            if (common.volume() > 0) share = common.massProperties();
          } catch (error:Dynamic) {
            common.close();
            throw error;
          }
          common.close();
          return share;
        });
        if (share != null) sums.addSolid(share, -1);
      }
    }
    return sums.result();
  }

  static function limits(box:Box3):PartBox return {
    minX: box.x,
    minY: box.y,
    minZ: box.z,
    maxX: box.x + box.width,
    maxY: box.y + box.depth,
    maxZ: box.z + box.height
  };

  static function inside(bounds:PartBox, box:Box3):Bool return bounds.minX >= box.x && bounds.minY >= box.y
    && bounds.minZ >= box.z && bounds.maxX <= box.x + box.width && bounds.maxY <= box.y + box.depth
    && bounds.maxZ <= box.z + box.height;
}

private typedef PartBox = machinekit.assembly.PosedParts.PartBox;

/** A solid named by its shape and pose (`key`, or null for one not kept), built on demand by `build`. */
typedef KeyedSolid = {key:Null<String>, build:() -> Part};

/** What `BoxMaterialVolume` knows about keyed solids. */
/** Signed zeroth, first and second volume moments about a fixed reference point near the material. */
private class Moments {
  final rx:Float;
  final ry:Float;
  final rz:Float;
  var v = 0.0;
  var fx = 0.0;
  var fy = 0.0;
  var fz = 0.0;
  var sxx = 0.0;
  var syy = 0.0;
  var szz = 0.0;
  var sxy = 0.0;
  var sxz = 0.0;
  var syz = 0.0;

  public function new(rx:Float, ry:Float, rz:Float) {
    this.rx = rx;
    this.ry = ry;
    this.rz = rz;
  }

  public function addBox(box:Box3):Void {
    var volume = box.width * box.depth * box.height;
    var cx = box.x + box.width / 2 - rx, cy = box.y + box.depth / 2 - ry, cz = box.z + box.height / 2 - rz;
    add(volume, cx, cy, cz, volume * box.width * box.width / 12, volume * box.depth * box.depth / 12,
      volume * box.height * box.height / 12, 0, 0, 0, 1);
  }

  /** A solid's integrated properties; `sign` -1 removes it. */
  public function addSolid(properties:cadkit.PhysicalProperties, sign:Float):Void {
    var c = properties.centerOfMass, i = properties.inertia;
    // Centroidal covariance from the inertia tensor (off-diagonals are minus the products).
    add(properties.volume, c.x - rx, c.y - ry, c.z - rz, (i.yy + i.zz - i.xx) / 2, (i.xx + i.zz - i.yy) / 2,
      (i.xx + i.yy - i.zz) / 2, -i.xy, -i.xz, -i.yz, sign);
  }

  function add(volume:Float, cx:Float, cy:Float, cz:Float, cxx:Float, cyy:Float, czz:Float, cxy:Float, cxz:Float,
      cyz:Float, sign:Float):Void {
    v += sign * volume;
    fx += sign * volume * cx;
    fy += sign * volume * cy;
    fz += sign * volume * cz;
    sxx += sign * (cxx + volume * cx * cx);
    syy += sign * (cyy + volume * cy * cy);
    szz += sign * (czz + volume * cz * cz);
    sxy += sign * (cxy + volume * cx * cy);
    sxz += sign * (cxz + volume * cx * cz);
    syz += sign * (cyz + volume * cy * cz);
  }

  public function result():Null<VolumeProperties> {
    if (!(v > 0)) return null;
    var cx = fx / v, cy = fy / v, cz = fz / v;
    var xx = sxx - v * cx * cx, yy = syy - v * cy * cy, zz = szz - v * cz * cz;
    var xy = sxy - v * cx * cy, xz = sxz - v * cx * cz, yz = syz - v * cy * cz;
    return new VolumeProperties(v, new Vector(cx + rx, cy + ry, cz + rz),
      new InertiaTensor(yy + zz, -xy, -xz, xx + zz, -yz, xx + yy));
  }
}
