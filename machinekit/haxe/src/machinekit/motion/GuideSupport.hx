package machinekit.motion;

/** A wrench at the centre of a rectangular four-block support. Forces are N, moments N m.
 * Local X spans the rails, Z follows travel, and Y is normal to the mounting plane. */
typedef GuideSupportWrench = {
	var fx:Float;
	var fy:Float;
	var fz:Float;
	var mx:Float;
	var my:Float;
	var mz:Float;
}

/** Load transmitted to one block, in the support's local frame. Positions are metres. */
typedef GuideBlockLoad = {
	var x:Float;
	var z:Float;
	var fx:Float;
	var fy:Float;
	var fz:Float;
	var magnitude:Float;
}

/** Reference load sharing for four equal-stiffness blocks on a rigid carriage.
 * The drive carries force along travel at its measured line of action; the blocks
 * carry transverse force and the remaining moments. Preload, flexible
 * castings and unequal block stiffness are not included; the capacity check needs an
 * explicit safety factor. This calculation is not a bearing-life estimate. */
class GuideSupport {
	public final railSpacing:Float;
	public final blockSpacing:Float;
	public final driveX:Float;
	public final driveY:Float;

	public function new(railSpacing:Float, blockSpacing:Float, driveX:Float, driveY:Float) {
		if (!Math.isFinite(railSpacing) || !(railSpacing > 0) ||
			!Math.isFinite(blockSpacing) || !(blockSpacing > 0))
			throw "Guide support needs finite positive rail and block spacing in metres";
		if (!Math.isFinite(driveX) || !Math.isFinite(driveY))
			throw "Guide drive line must be finite";
		this.railSpacing = railSpacing;
		this.blockSpacing = blockSpacing;
		this.driveX = driveX;
		this.driveY = driveY;
	}

	public function loads(wrench:GuideSupportWrench):Array<GuideBlockLoad> {
		for (value in [wrench.fx, wrench.fy, wrench.fz, wrench.mx, wrench.my, wrench.mz])
			if (!Math.isFinite(value)) throw "Guide support wrench must be finite";
		var across2 = railSpacing * railSpacing;
		var along2 = blockSpacing * blockSpacing;
		var mx = wrench.mx - driveY * wrench.fz;
		var my = wrench.my + driveX * wrench.fz;
		var result:Array<GuideBlockLoad> = [];
		for (sx in [-1, 1]) for (sz in [-1, 1]) {
			var x = sx * railSpacing / 2;
			var z = sz * blockSpacing / 2;
			var fx = wrench.fx / 4 + my * z / along2;
			var fy = wrench.fy / 4 + wrench.mz * x / across2 - mx * z / along2;
			var fz = 0.0;
			result.push({x: x, z: z, fx: fx, fy: fy, fz: fz,
				magnitude: Math.sqrt(fx * fx + fy * fy + fz * fz)});
		}
		return result;
	}

	/** Largest block utilization against an explicitly supplied static rating. */
	public function utilization(wrench:GuideSupportWrench, staticCapacity:Float, safetyFactor:Float):Float {
		if (!Math.isFinite(staticCapacity) || !(staticCapacity > 0) ||
			!Math.isFinite(safetyFactor) || safetyFactor < 1)
			throw "Guide capacity must be positive and its safety factor at least one";
		var worst = 0.0;
		for (load in loads(wrench)) worst = Math.max(worst, load.magnitude * safetyFactor / staticCapacity);
		return worst;
	}

	public function requireCapacity(wrench:GuideSupportWrench, staticCapacity:Float, safetyFactor:Float):Float {
		var used = utilization(wrench, staticCapacity, safetyFactor);
		if (used > 1) throw 'Guide block static capacity exceeded: ${used * 100}% utilization';
		return used;
	}
}
