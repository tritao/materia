package materia.assembly;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;
import materia.assembly.AssemblyDefinition.AssemblyVector;

/** Rigid-frame arithmetic shared by CAD generators and project consumers. */
class AssemblyFrames {
	public static function identity():AssemblyFrame
		return {x: 0, y: 0, z: 0, qx: 0, qy: 0, qz: 0, qw: 1};

	public static function translation(x:Float, y:Float, z:Float):AssemblyFrame
		return {x: x, y: y, z: z, qx: 0, qy: 0, qz: 0, qw: 1};

	/** Build a frame from a row-major 3x3 rotation matrix and an origin. */
	public static function fromRotationMatrix(x:Float, y:Float, z:Float, matrix:Array<Float>):AssemblyFrame {
		if (matrix == null || matrix.length != 9) throw "Assembly rotation matrix must have nine values";
		for (value in matrix) if (!Math.isFinite(value)) throw "Assembly rotation matrix must be finite";
		var m00 = matrix[0], m01 = matrix[1], m02 = matrix[2];
		var m10 = matrix[3], m11 = matrix[4], m12 = matrix[5];
		var m20 = matrix[6], m21 = matrix[7], m22 = matrix[8];
		var qx:Float, qy:Float, qz:Float, qw:Float;
		var trace = m00 + m11 + m22;
		if (trace > 0) {
			var s = Math.sqrt(trace + 1) * 2;
			qw = s / 4; qx = (m21 - m12) / s; qy = (m02 - m20) / s; qz = (m10 - m01) / s;
		} else if (m00 > m11 && m00 > m22) {
			var s = Math.sqrt(1 + m00 - m11 - m22) * 2;
			qw = (m21 - m12) / s; qx = s / 4; qy = (m01 + m10) / s; qz = (m02 + m20) / s;
		} else if (m11 > m22) {
			var s = Math.sqrt(1 + m11 - m00 - m22) * 2;
			qw = (m02 - m20) / s; qx = (m01 + m10) / s; qy = s / 4; qz = (m12 + m21) / s;
		} else {
			var s = Math.sqrt(1 + m22 - m00 - m11) * 2;
			qw = (m10 - m01) / s; qx = (m02 + m20) / s; qy = (m12 + m21) / s; qz = s / 4;
		}
		var norm = Math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
		if (!Math.isFinite(norm) || norm < 1e-12) throw "Assembly rotation matrix has no valid quaternion";
		var result:AssemblyFrame = {x: x, y: y, z: z, qx: qx / norm, qy: qy / norm, qz: qz / norm, qw: qw / norm};
		AssemblyCodec.validateFrame(result);
		return result;
	}

	/** Row-major 3x3 matrix corresponding to the frame's rotation. */
	public static function toRotationMatrix(frame:AssemblyFrame):Array<Float> {
		AssemblyCodec.validateFrame(frame);
		var norm = Math.sqrt(frame.qx * frame.qx + frame.qy * frame.qy + frame.qz * frame.qz + frame.qw * frame.qw);
		var x = frame.qx / norm, y = frame.qy / norm, z = frame.qz / norm, w = frame.qw / norm;
		return [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
			2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
			2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)];
	}

	public static function turnY(radians:Float):AssemblyFrame {
		if (!Math.isFinite(radians)) throw "Assembly angle must be finite";
		return {x: 0, y: 0, z: 0, qx: 0, qy: Math.sin(radians / 2), qz: 0,
			qw: Math.cos(radians / 2)};
	}

	/** Motion transform for a joint axis expressed in its parent connector frame. */
	public static function axisMotion(type:AssemblyJointType, axis:AssemblyVector,
			coordinate:Float):AssemblyFrame {
		if (axis == null || !Math.isFinite(axis.x) || !Math.isFinite(axis.y) || !Math.isFinite(axis.z) ||
			!Math.isFinite(coordinate)) throw "Assembly joint motion must be finite";
		var length = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
		if (!Math.isFinite(length) || length < 1e-9) throw "Assembly joint axis has zero length";
		var x = axis.x / length, y = axis.y / length, z = axis.z / length;
		if (type == AssemblyJointType.Fixed) return identity();
		if (type == AssemblyJointType.Prismatic) return translation(x * coordinate, y * coordinate, z * coordinate);
		if (type == AssemblyJointType.Revolute || type == AssemblyJointType.Continuous) {
			var sine = Math.sin(coordinate / 2);
			return {x: 0, y: 0, z: 0, qx: x * sine, qy: y * sine, qz: z * sine,
				qw: Math.cos(coordinate / 2)};
		}
		throw 'Unsupported assembly joint type "$type"';
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

	public static function transformVector(frame:AssemblyFrame, x:Float, y:Float, z:Float):{x:Float, y:Float, z:Float} {
		var transformed = transformPoint(frame, x, y, z);
		return {x: transformed.x - frame.x, y: transformed.y - frame.y, z: transformed.z - frame.z};
	}
}
