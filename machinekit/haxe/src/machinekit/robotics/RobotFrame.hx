package machinekit.robotics;

/** Metre frame whose approach axis is +Z. */
typedef RobotFrame = {
	var position:{x:Float, y:Float, z:Float};
	var quaternion:{x:Float, y:Float, z:Float, w:Float};
}
