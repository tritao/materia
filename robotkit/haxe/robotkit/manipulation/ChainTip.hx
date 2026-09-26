package robotkit.manipulation;

import robotkit.model.LinkId;
import robotkit.model.FrameId;

/** Where a KinematicChain ends: at a link's own origin, or at a mounted frame. */
enum ChainTip {
  Link(id:LinkId);
  Frame(id:FrameId);
}
