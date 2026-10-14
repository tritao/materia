package robotkit.runtime;

import NativeKitSim;
import nativekit.sim.SimObject;
import nativekit.sim.SimPose;

/** Retains an identified environment body on a live fixture and reports its actual native ownership. */
interface SimulationObjectFixture {
  function retain(body:SimObject, world:SimPose):Void;
  function carries(body:SimObject):Bool;
  /** The native body a fastener threads into: the member's own, or its robot link's. */
  function threadedBody():nksim_body;
}
