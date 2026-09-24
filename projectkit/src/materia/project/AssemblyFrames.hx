package materia.project;

import materia.project.AssemblyRecord.AssemblyFrame;

/** Rigid-frame arithmetic shared by CAD generators and project consumers. */
class AssemblyFrames {
	public static function identity():AssemblyFrame
		return {x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1};

	public static function translation(x:Float, y:Float, z:Float):AssemblyFrame
		return {x: x, y: y, z: z, qx: 0, qy: 0, qz: 0, qw: 1};

	public static function turnY(radians:Float):AssemblyFrame {
		if (!Math.isFinite(radians)) throw "Assembly angle must be finite";
		return {x: 0, y: 0, z: 0, qx: 0, qy: Math.sin(radians / 2), qz: 0,
			qw: Math.cos(radians / 2)};
	}

	/** Frame whose local Y axis points along the supplied vector. */
	public static function alongY(x:Float, y:Float, z:Float,
			dx:Float, dy:Float, dz:Float):AssemblyFrame {
		var length = Math.sqrt(dx * dx + dy * dy + dz * dz);
		if (!Math.isFinite(length) || length < 1e-9) throw "Assembly axis has zero length";
		var ux = dx / length, uy = dy / length, uz = dz / length;
		if (uy < -0.999999) return {x: x, y: y, z: z, qx: 1, qy: 0, qz: 0, qw: 0};
		var factor = Math.sqrt(2 * (1 + uy));
		return {x: x, y: y, z: z, qx: uz / factor, qy: 0,
			qz: -ux / factor, qw: factor / 2};
	}

	public static function compose(parent:AssemblyFrame, child:AssemblyFrame):AssemblyFrame {
		var point = transformPoint(parent, child.x, child.y, child.z);
		return {x: point.x, y: point.y, z: point.z,
			qx: parent.qw * child.qx + parent.qx * child.qw + parent.qy * child.qz - parent.qz * child.qy,
			qy: parent.qw * child.qy - parent.qx * child.qz + parent.qy * child.qw + parent.qz * child.qx,
			qz: parent.qw * child.qz + parent.qx * child.qy - parent.qy * child.qx + parent.qz * child.qw,
			qw: parent.qw * child.qw - parent.qx * child.qx - parent.qy * child.qy - parent.qz * child.qz};
	}

	public static function inverse(frame:AssemblyFrame):AssemblyFrame {
		var reverse:AssemblyFrame = {x: 0, y: 0, z: 0, qx: -frame.qx, qy: -frame.qy,
			qz: -frame.qz, qw: frame.qw};
		var point = transformPoint(reverse, -frame.x, -frame.y, -frame.z);
		reverse.x = point.x; reverse.y = point.y; reverse.z = point.z;
		return reverse;
	}

	public static function transformPoint(frame:AssemblyFrame, x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float} {
		var tx = 2 * (frame.qy * z - frame.qz * y);
		var ty = 2 * (frame.qz * x - frame.qx * z);
		var tz = 2 * (frame.qx * y - frame.qy * x);
		return {x: x + frame.qw * tx + frame.qy * tz - frame.qz * ty + frame.x,
			y: y + frame.qw * ty + frame.qz * tx - frame.qx * tz + frame.y,
			z: z + frame.qw * tz + frame.qx * ty - frame.qy * tx + frame.z};
	}
}
