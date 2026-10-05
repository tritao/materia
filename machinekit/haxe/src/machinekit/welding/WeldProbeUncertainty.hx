package machinekit.welding;

import cadkit.modeling.Vector;
import machinekit.welding.WeldProbeGeometry.WeldProbeFace;
import machinekit.welding.WeldProbeParkingBounds.WeldProbeRegionBounds;

/** Search-ray intersection bounds in the CAD model's length unit, for the current probe placement. */
interface WeldProbeUncertainty {
  public function region(face:WeldProbeFace, point:Vector):WeldProbeRegionBounds;
}

/** Bind observed uncertainty without making CAD geometry depend on a runtime registration implementation. */
class WeldProbeObservedBounds implements WeldProbeUncertainty {
  final bounds:WeldProbeFace->Vector->WeldProbeRegionBounds;
  public function new(bounds:WeldProbeFace->Vector->WeldProbeRegionBounds) {
    if (bounds == null) throw "Observed probe uncertainty needs a region provider";
    this.bounds = bounds;
  }
  public function region(face:WeldProbeFace, point:Vector):WeldProbeRegionBounds return bounds(face, point);
}
