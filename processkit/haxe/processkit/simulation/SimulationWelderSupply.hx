package processkit.simulation;

import haxe.Int64;
import processkit.tool.WeldSensor.WeldReading;

/** A supply exercised by the CAD simulation; its feedback drives both the process and deposited metal. */
interface SimulationWelderSupply {
  public function observe(dt:Float, timestamp:Int64, grounded:Bool):WeldReading;
  public function safe():Void;
  public function reset():Void;
}
