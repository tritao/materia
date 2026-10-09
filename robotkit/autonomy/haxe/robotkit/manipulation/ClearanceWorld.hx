package robotkit.manipulation;

import robotkit.manipulation.ClearanceViolation;

/**
 * What process planning asks of a clearance world (COLLISION.md CL-D12):
 * margin violations with per-body inflation, the closest pair, sampled
 * straight joint sweeps, and the motion bounds of its bodies, for an arm at
 * joint values. `robotkit.collision.CollisionClearance` implements it on a
 * collisionkit world.
 *
 * Margins: a pair must keep `margin` (or a smaller `wanted`); with
 * `contact`, a tool body need keep only `contactMargin` from fixed bodies.
 * `displacements` (one per body) widen a pair's required distance by both
 * bodies' bound, as the continuous proof needs.
 */
interface ClearanceWorld {
  /** The arm whose joint values the queries take. */
  function group():KinematicGroup;

  /** The first pair closer than it may be at `q`, or null. */
  function violation(q:Array<Float>, ?contact:Bool, ?wanted:Float, ?displacements:Array<Float>):Null<ClearanceViolation>;
  /** The closest checked pair, clear or not, or null when none is checked. */
  function closest(q:Array<Float>, ?contact:Bool, ?wanted:Float):Null<ClearanceViolation>;
  /** The first violation along the straight joint motion, sampled at most `maxJointStep` per joint. */
  function sweep(from:Array<Float>, to:Array<Float>, ?contact:Bool, ?maxJointStep:Float, ?wanted:Float,
    ?contactAt:Array<Float>->Bool, ?endpointsChecked:Bool):Null<ClearanceViolation>;
  /** The closest pair over the same sampled sweep. */
  function closestSweep(from:Array<Float>, to:Array<Float>, ?contact:Bool, ?maxJointStep:Float,
    ?wanted:Float):Null<ClearanceViolation>;
  /** CL-D5's motion envelope of the bodies over joint boxes. */
  function motionEnvelope(lower:Array<Float>, upper:Array<Float>):ClearanceMotionEnvelope;
  /** Per-body displacement bounds for joint errors at `q`. */
  function displacementBounds(q:Array<Float>, errors:Array<Float>):Array<Float>;
  function tcpDisplacementBound(q:Array<Float>, errors:Array<Float>):Float;
  /** Checks the bound against an actual displacement; returns the largest ratio. */
  function auditDisplacement(q:Array<Float>, perturbed:Array<Float>, errors:Array<Float>,
    ?envelope:ClearanceMotionEnvelope):Float;
  /** The same geometry on another compiled group (a worker's). */
  function withGroup(group:KinematicGroup):ClearanceWorld;
  function pairCount():Int;
  function bodyCount():Int;
  function movingNames():Array<String>;
}
