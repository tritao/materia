package machinekit.robotics;

import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;

/** Frame conversions for end effector (EOAT) integration with RobotKit. */
class EndEffectorFrames {
	/** Convert a MachineKit +Y approach to a RobotKit +Z approach. */
	public static function approachYToZ(frame:AssemblyFrame):AssemblyFrame {
		var half = Math.sqrt(0.5);
		return AssemblyFrames.compose(frame,
			{x: 0, y: 0, z: 0, qx: half, qy: 0, qz: 0, qw: half});
	}

	/** Convert a millimetre frame to metres with a plain position and quaternion. */
	public static function toMetres(frame:AssemblyFrame):{position:{x:Float, y:Float, z:Float},
			quaternion:{x:Float, y:Float, z:Float, w:Float}} {
		return {position: {x: frame.x / 1000, y: frame.y / 1000, z: frame.z / 1000},
			quaternion: {x: frame.qx, y: frame.qy, z: frame.qz, w: frame.qw}};
	}

	public static function toRobotFrame(frame:AssemblyFrame):{position:{x:Float, y:Float, z:Float},
			quaternion:{x:Float, y:Float, z:Float, w:Float}}
		return toMetres(approachYToZ(frame));
}
