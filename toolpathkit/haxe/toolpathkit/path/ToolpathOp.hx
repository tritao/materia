package toolpathkit.path;

import toolpathkit.path.Provenance;
import toolpathkit.path.PathGeometry;
import toolpathkit.path.ArcPlane;

/** Controller-independent CNC operations in LinuxCNC block order. */
enum ToolpathOp {
  /** Selects a work-coordinate setup. IDs have no controller-specific meaning. */
  SetSetup(id:String, provenance:Provenance);
  /**
    Programmed points in the active setup's work coordinates, as G-code
    writes them: with G43 active they name the tool tip, and the machine's
    controlled point is the active length above. `ToolpathFrame` maps them.
  **/
  Move(kind:MoveKind, geometry:PathGeometry, feed:Float,
    tolerance:Float, provenance:Provenance);
  /** A move of the controlled point in absolute machine coordinates. */
  MachineMove(kind:MoveKind, geometry:PathGeometry, feed:Float,
    tolerance:Float, provenance:Provenance);
  /** Add measured master motion to the follower's independently planned motion. */
  /** Absolute auxiliary joint position, SI units; independent planned coordinate when geared. */
  PositionJoint(joint:String, position:Float, provenance:Provenance);
  /** Digital barrier; the machine binding supplies the measured signal. */
  WaitSignal(channel:String, expected:Bool, timeoutSeconds:Float, provenance:Provenance);
  /** Modal axial feed in metres/work revolution on Cut lines; distance=0 restores explicit feeds. */
  FeedPerRevolution(master:String, axis:String, distance:Float, provenance:Provenance);
  ElectronicGear(follower:String, terms:Array<GearTerm>, provenance:Provenance);
  ReleaseGear(follower:String, provenance:Provenance);
  Dwell(seconds:Float, span:Provenance);
  Spindle(direction:SpindleDirection, rpm:Float, provenance:Provenance);
  Coolant(mist:Bool, flood:Bool, provenance:Provenance);
  ToolChange(number:Int, span:Provenance);
  /**
    G43 Hn sets tool length offset `number` (G49 is number 0). Later `Move`
    points are programmed at the tip, `length` below the controlled point.
  **/
  ToolLengthOffset(number:Int, length:Float, span:Provenance);
  OptionalStop(span:Provenance);
  ProgramStop(span:Provenance);
  End(span:Provenance);

  // Printing. Extrusion is a process quantity along the path, not an axis: the program stores the
  // bead laid per distance, which is machine independent. The filament diameter belongs to the
  // material and the extruder, so G-code import and export convert `E` with it.

  /**
    Modal: every following `Cut` move lays a bead of this cross-section until the next `Deposit` or
    `DepositOff`. `area` is in m², and `width` by `height` is the bead's footprint, which its
    rounded cross-section cannot exceed. `Rapid`, `Link` and `Retract` moves never deposit.
  **/
  Deposit(area:Float, width:Float, height:Float, provenance:Provenance);
  /** Ends deposition: following `Cut` moves lay nothing. */
  DepositOff(provenance:Provenance);
  /**
    An extruder-only move, in metres of filament at `speed` metres of filament per second.
    Positive primes, negative retracts. It is how retraction, unretraction and purge are written.
  **/
  FilamentMove(length:Float, speed:Float, provenance:Provenance);
  /** Sets a heater, such as `hotend` or `bed`, to `celsius`; with `wait`, later moves wait until it is there. */
  Heat(heater:String, celsius:Float, wait:Bool, provenance:Provenance);
  /** Sets a fan to a duty from 0 to 1. */
  Fan(fan:String, duty:Float, provenance:Provenance);
  /** Marker: the layer `index` (from 0) printed at height `z`. Lowering ignores it. */
  Layer(index:Int, z:Float, provenance:Provenance);
  /** Marker: the moves up to the next marker are of this kind. Lowering ignores it. */
  Feature(kind:PrintFeature, provenance:Provenance);
  /** Marker: the part named `name` starts (or, with `start` false, ends). Lowering ignores it. */
  PrintObject(name:String, start:Bool, provenance:Provenance);
}
