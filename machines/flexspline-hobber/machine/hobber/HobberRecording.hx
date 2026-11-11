package hobber;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/**
 * Helpers for hobber parts that record in their own frame while the layout still gives their shapes in the part datum frame:
 * a datum-frame box as a box in the part's frame, and a frame as a recorded datum. As the layout moves to skeleton frames,
 * the calls fall away.
 */
class HobberRecording {
  /**
   * The axis-aligned box `x0..x1, y0..y1, z0..z1` of the frame `toOwn` maps from, as a box in the part's frame. The part's axes
   * are a permutation of that frame's, so the box stays axis-aligned.
   */
  public static function box(b:DocumentBuilder, toOwn:AssemblyFrame, x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float):Feature {
    var lo = [1e300, 1e300, 1e300], hi = [-1e300, -1e300, -1e300];
    for (x in [x0, x1]) for (y in [y0, y1]) for (z in [z0, z1]) {
      var p = F.transformPoint(toOwn, x, y, z), v = [p.x, p.y, p.z];
      for (i in 0...3) {
        lo[i] = Math.min(lo[i], v[i]);
        hi[i] = Math.max(hi[i], v[i]);
      }
    }
    return b.boxBetween(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
  }

  /** `frame` (in the part's frame) recorded as the datum `name`: its +Z is the plane's normal, its +X the plane's x direction. */
  public static function datum(b:DocumentBuilder, name:String, frame:AssemblyFrame):Void {
    var o = F.transformPoint(frame, 0, 0, 0), x = F.transformVector(frame, 1, 0, 0), z = F.transformVector(frame, 0, 0, 1);
    b.datum(name, new Plane(new Vector(o.x, o.y, o.z), new Vector(x.x, x.y, x.z), new Vector(z.x, z.y, z.z)));
  }
}
