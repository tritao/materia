package robotkit.runtime;

import nativekit.sim.SimObject;
import nativekit.sim.SimPose;

/** Retains an identified environment body on a live fixture and reports its actual native ownership. */
interface SimulationObjectFixture {
  function retain(body:SimObject, world:SimPose):Void;
  function carries(body:SimObject):Bool;
}
