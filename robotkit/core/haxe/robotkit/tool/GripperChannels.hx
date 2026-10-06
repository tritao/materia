package robotkit.tool;

import robotkit.execution.ProcessChannelDeclaration;
import robotkit.execution.ProcessEventValue;

/** One logical jaw command. It defaults open on a fault and retains its value
 * through an ordinary stop so a stopped arm keeps the part it is carrying. */
class GripperChannels implements ToolChannels {
	public static inline var KEEPS_ON_STOP:Bool = true;

	public final channel:String;

	public function new(channel:String) {
		if (channel == null || channel.length == 0) throw "Gripper channel needs a name";
		this.channel = channel;
	}

	public function declarations():Array<ProcessChannelDeclaration>
		return [new ProcessChannelDeclaration(channel, ProcessEventValue.Digital(false), KEEPS_ON_STOP)];
}
