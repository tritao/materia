package processkit.simulation;

import processkit.screwdriving.ScrewdriverReading;

typedef ScrewdriverControlState = {var start:Bool; var reverse:Bool; var vacuum:Bool; var program:Float;}
typedef ScrewdriverJointState = {var feedPosition:Float; var spindlePosition:Float; var spindleSpeed:Float;}

/** Hardware-in-loop boundary: executed controls, physical encoders and acquired process frames. */
interface SimulationScrewdriverSupply {
  function controls():ScrewdriverControlState;
  function joints():ScrewdriverJointState;
  function publish(reading:ScrewdriverReading):Void;
  function reading():ScrewdriverReading;
  function reset():Void;
  function safe():Void;
}
