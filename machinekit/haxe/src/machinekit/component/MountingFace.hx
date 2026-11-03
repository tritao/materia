package machinekit.component;

import cadkit.modeling.Part;
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

  public function new(width:Float, height:Float, cutout:Float -> Part, ?pose:AssemblyFrame) {
    if (!(width > 0) || !(height > 0) || !Math.isFinite(width) || !Math.isFinite(height)) throw "A mounting face needs a positive finite footprint";
    this.width = width;
    this.height = height;
    this.cutout = cutout;
    this.pose = pose ?? AssemblyFrames.identity();
  }

  /** A new owned tool in the face frame; the caller closes it. */
  public function tool(depth:Float):Part return cutout(depth);
}
