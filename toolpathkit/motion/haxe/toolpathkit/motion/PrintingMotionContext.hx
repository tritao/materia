package toolpathkit.motion;

import materia.assembly.AssemblyDefinition;
import robotkit.model.RobotModel;
import robotkit.model.SteadyLoads;
import robotkit.runtime.RobotRuntimeBlueprint;

/** Assembly placement and execution context; initial positions are in authored assembly units. */
typedef PrintingMotionContext = {var definition:AssemblyDefinition;
var initialPositions:Array<Float>;
var metresPerUnit:Float;
var model:RobotModel;
var runtime:RobotRuntimeBlueprint;
var fixedTimestepSeconds:Float;
var channels:ToolpathChannels;
var steady:SteadyLoads;
}
