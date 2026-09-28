package cadkit;

/** Symmetric inertia matrix about a stated centre of mass. */
class InertiaTensor {
	public final xx:Float;
	public final xy:Float;
	public final xz:Float;
	public final yy:Float;
	public final yz:Float;
	public final zz:Float;

	public function new(xx:Float, xy:Float, xz:Float, yy:Float, yz:Float, zz:Float) {
		for (value in [xx, xy, xz, yy, yz, zz]) if (!Math.isFinite(value))
			throw "inertia tensor must be finite";
		this.xx = xx;
		this.xy = xy;
		this.xz = xz;
		this.yy = yy;
		this.yz = yz;
		this.zz = zz;
	}

	public static function zero():InertiaTensor return new InertiaTensor(0, 0, 0, 0, 0, 0);

	public function add(other:InertiaTensor):InertiaTensor
		return new InertiaTensor(xx + other.xx, xy + other.xy, xz + other.xz,
			yy + other.yy, yz + other.yz, zz + other.zz);

	public function scaled(factor:Float):InertiaTensor
		return new InertiaTensor(xx * factor, xy * factor, xz * factor,
			yy * factor, yz * factor, zz * factor);

	/** Rotate from local axes into world axes using a unit quaternion. */
	public function rotated(qx:Float, qy:Float, qz:Float, qw:Float):InertiaTensor {
		if (!Math.isFinite(qx) || !Math.isFinite(qy) || !Math.isFinite(qz) || !Math.isFinite(qw) ||
			Math.abs(qx * qx + qy * qy + qz * qz + qw * qw - 1) > 1e-6)
			throw "inertia rotation needs a unit quaternion";
		var r = [
			[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
			[2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
			[2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]
		];
		var j = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
		var result:Array<Array<Float>> = [for (_ in 0...3) [0.0, 0.0, 0.0]];
		for (i in 0...3) for (k in 0...3) for (l in 0...3)
			result[i][k] += r[i][l] * (j[l][0] * r[k][0] + j[l][1] * r[k][1] + j[l][2] * r[k][2]);
		return new InertiaTensor(result[0][0], (result[0][1] + result[1][0]) / 2,
			(result[0][2] + result[2][0]) / 2, result[1][1],
			(result[1][2] + result[2][1]) / 2, result[2][2]);
	}

	/** Parallel axis theorem: shift from the centre of mass by (dx, dy, dz). */
	public function shifted(mass:Float, dx:Float, dy:Float, dz:Float):InertiaTensor {
		if (!Math.isFinite(mass) || mass < 0) throw "inertia shift needs nonnegative mass";
		return new InertiaTensor(xx + mass * (dy * dy + dz * dz), xy - mass * dx * dy,
			xz - mass * dx * dz, yy + mass * (dx * dx + dz * dz),
			yz - mass * dy * dz, zz + mass * (dx * dx + dy * dy));
	}
}
