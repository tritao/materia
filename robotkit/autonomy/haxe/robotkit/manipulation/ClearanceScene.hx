package robotkit.manipulation;

/**
 * A clearance world whose cell can change while a program runs (CL-D6,
 * CL-D9): validation replays the program's changes in time order, with the
 * arm at `q` when each happens. `reset` returns to the cell as built.
 */
interface ClearanceScene {
  function apply(change:ClearanceChange, q:Array<Float>):Void;
  function reset():Void;
}
