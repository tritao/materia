package machinekit.component;

import cadkit.modeling.Part;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * The flange a part bolts to its host with, as the host must be cut for it: a footprint on the flange face, centred on the
 * face frame's origin, and the tool that cuts a plate lying ahead of the flange (face frame z from 0 to the depth) for the
 * part's pilot and bolts. The tool is in the face frame, which is the part's own frame unless `pose` says otherwise. `AssemblyIntegrity` checks that the host a flanged part is fixed to carries this cut-out.
 */
class MountingFace {
  /** The flange's footprint along the part's x and y, mm. */
  public final width:Float;
  public final height:Float;
  /** The face frame in the part's frame: origin on the flange face, +Z into the host. */
  public final pose:AssemblyFrame;
  final cutout:Float -> Part;
  /** The cut-out for a host that threads the flange's bolts (pilot and tap-drill holes), where the flange takes that. */
  final tapped:Null<Float -> Part>;

  public function new(width:Float, height:Float, cutout:Float -> Part, ?pose:AssemblyFrame, ?tapped:Float -> Part) {
    this.tapped = tapped;
    if (!(width > 0) || !(height > 0) || !Math.isFinite(width) || !Math.isFinite(height)) throw "A mounting face needs a positive finite footprint";
    this.width = width;
    this.height = height;
    this.cutout = cutout;
    this.pose = pose ?? AssemblyFrames.identity();
  }

  /** A new owned tool in the face frame, with clearance holes for through bolts; the caller closes it. */
  public function tool(depth:Float):Part return cutout(depth);

  /** Whether the flange has clearance holes only and bolts into a tapped host (a servo on its mount). */
  public function acceptsTapped():Bool return tapped != null;

  /**
	 * The tool the host must be cut with: the tapped host's tap-drill holes when the flange takes them (clearance holes in the
	 * flange, bolts threading into the host), otherwise the clearance holes. A host cut for through bolts has holes at least
	 * the tap drill wide, so it passes too.
	 */
  public function hostTool(depth:Float):Part {
    var make = tapped;
    return make == null ? cutout(depth) : make(depth);
  }

  /** The bolt a catalogue clearance hole of this diameter belongs to: thread name, nominal diameter and tap-drill, mm. */
  public static function boltFor(holeDiameter:Float):{name:String, diameter:Float, tapDrill:Float} {
    var best = "M3", bestDiameter = 3.0, bestGap = Math.POSITIVE_INFINITY;
    for (row in [{hole: 3.4, name: "M3", diameter: 3.0}, {hole: 4.5, name: "M4", diameter: 4.0}, {hole: 5.5, name: "M5", diameter: 5.0},
      {hole: 6.6, name: "M6", diameter: 6.0}, {hole: 9.0, name: "M8", diameter: 8.0}, {hole: 11.0, name: "M10", diameter: 10.0}]) {
      var gap = Math.abs(row.hole - holeDiameter);
      if (gap < bestGap) {
        bestGap = gap;
        best = row.name;
        bestDiameter = row.diameter;
      }
    }
    var screw = SocketHeadCapScrew.catalog().get(best);
    if (screw == null) throw 'No $best screw row';
    return {name: best, diameter: bestDiameter, tapDrill: screw.tapDrill};
  }

  /**
	 * The seat of a part that stands on a flat base on tapped holes: `holes` are bolt axes in the part's x and z on the base
	 * plane y = `planeY` (the part's +Y is up from the base); the seat is cut `depth` deep downward (-Y) with the tap-drill hole
	 * of each bolt. The footprint is the holes' box with a margin, centred on them.
	 */
  public static function tappedFoot(holes:Array<{x:Float, z:Float, diameter:Float}>, planeY:Float):Null<MountingFace> {
    if (holes.length == 0) return null;
    var minX = Math.POSITIVE_INFINITY, maxX = Math.NEGATIVE_INFINITY, minZ = Math.POSITIVE_INFINITY, maxZ = Math.NEGATIVE_INFINITY;
    var widest = 0.0;
    for (hole in holes) {
      minX = Math.min(minX, hole.x);
      maxX = Math.max(maxX, hole.x);
      minZ = Math.min(minZ, hole.z);
      maxZ = Math.max(maxZ, hole.z);
      widest = Math.max(widest, hole.diameter);
    }
    var cx = (minX + maxX) / 2, cz = (minZ + maxZ) / 2, margin = 2 * widest;
    var local = [for (hole in holes) {x: hole.x - cx, y: hole.z - cz, drill: boltFor(hole.diameter).tapDrill}];
    return new MountingFace(
      Math.max(maxX - minX, widest) + 2 * margin,
      Math.max(maxZ - minZ, widest) + 2 * margin,
      depth -> {
        if (!(depth > 0)) throw "A seat cut-out needs a positive depth";
        return Solids.union([for (hole in local) Part.cylinderSpan(hole.drill / 2, 0, depth, hole.x, hole.y)]);
      },
      AssemblyFrames.fromRotationMatrix(cx, planeY, cz, [1, 0, 0, 0, 0, -1, 0, 1, 0])
    );
  }
}
