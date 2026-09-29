package machinekit.robotics;

import materia.assembly.AssemblyFrames;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import machinekit.units.Millimetres;
import machinekit.units.Metres;

/** Frame conversions for end effector (EOAT) integration with RobotKit. */
class EndEffectorFrames {
	/** Coordinates in a +Y connector basis expressed in a +Z robot basis. */
	public static function pointYToZ(x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float}
		return {x: x, y: -z, z: y};

	/** Change both connector-relative bases from MachineKit +Y to RobotKit +Z. */
	public static function approachYToZ(frame:AssemblyFrame):AssemblyFrame {
		var half = Math.sqrt(0.5);
		var toRobot:AssemblyFrame = {x: 0, y: 0, z: 0, qx: half, qy: 0, qz: 0, qw: half};
		return AssemblyFrames.compose(AssemblyFrames.compose(toRobot, frame),
			AssemblyFrames.inverse(toRobot));
	}

	/** Convert a millimetre frame to metres with a plain position and quaternion. */
	public static function toMetres(frame:AssemblyFrame):RobotFrame {
		var x:Metres = (new Millimetres(frame.x)).metres();
		var y:Metres = (new Millimetres(frame.y)).metres();
		var z:Metres = (new Millimetres(frame.z)).metres();
		return {position: {x: x, y: y, z: z},
			quaternion: {x: frame.qx, y: frame.qy, z: frame.qz, w: frame.qw}};
	}

	public static function toRobotFrame(frame:ConnectorFrame):RobotFrame
		return toMetres(approachYToZ(frame.raw()));
}
