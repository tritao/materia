package machinekit.component;

import cadkit.modeling.Part;

/**
 * The flange a part bolts to its host with, as the host must be cut for it: a footprint on the part's z = 0 plane, centred
 * on the origin, and the tool that cuts a plate lying ahead of the flange (z from 0 to the depth) for the part's pilot and
 * bolts. `AssemblyIntegrity` checks that the host a flanged part is fixed to carries this cut-out.
 */
class MountingFace {
  /** The flange's footprint along the part's x and y, mm. */
  public final width:Float;
  public final height:Float;
  final cutout:Float -> Part;

  public function new(width:Float, height:Float, cutout:Float -> Part) {
    if (!(width > 0) || !(height > 0) || !Math.isFinite(width) || !Math.isFinite(height)) throw "A mounting face needs a positive finite footprint";
    this.width = width;
    this.height = height;
    this.cutout = cutout;
  }

  /** A new owned tool in the part's frame; the caller closes it. */
  public function tool(depth:Float):Part return cutout(depth);
}
