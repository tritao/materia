package robotkit.tool;

import robotkit.world.ProcessChannelDeclaration;

/**
 * The process channels a robot's tool is worked by, with the safe value and stop policy the tool itself needs: what a
 * stop must do to an arc, a wire or a vacuum belongs to the tool, not to whoever sets the robot up. A robot takes them
 * with `RobotRuntimeBlueprint.addTool`.
 */
interface ToolChannels {
  function declarations():Array<ProcessChannelDeclaration>;
}
