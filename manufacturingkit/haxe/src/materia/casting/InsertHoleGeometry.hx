package materia.casting;

import materia.casting.InsertHole;

/** Exact stock geometry and mass properties for neutral hole patterns. */
class InsertHoleGeometry {
	public static function axis(hole:InsertHole):Int {
		var normal = materia.assembly.AssemblyFrames.transformVector(hole.frame, 0, 0, 1);
		var index = CastingBounds.axis(normal);
		if (index != null)
			return index;
		throw "Unsupported non-axial insert hole: " + hole.id;
	}

	static function maximumRadius(hole:InsertHole):Float {
		var diameter = hole.diameter;
		if (hole.counterbore != null) diameter = Math.max(diameter, hole.counterbore.diameter);
		if (hole.relief != null) diameter = Math.max(diameter, hole.relief.diameter);
		return diameter / 2;
	}

	/** Actual radial stock beside drilling, including counterbore and thread relief. */
	public static function radialWall(width:Float, depth:Float, height:Float, hole:InsertHole):Float {
		var direction = axis(hole), entry = [hole.frame.x, hole.frame.y, hole.frame.z];
		var low = [-width / 2, -depth / 2, 0.0], high = [width / 2, depth / 2, height];
		var radius = maximumRadius(hole);
		var wall = Math.POSITIVE_INFINITY;
		for (index in 0...3)
			if (index != direction)
				wall = Math.min(wall, Math.min(entry[index] - low[index], high[index] - entry[index]) - radius);
		return wall;
	}

	/** Exact insert volume and centroid for disjoint axial drilling patterns. */
	public static function massProperties(width:Float, depth:Float, height:Float, holes:Array<InsertHole>):{
		volume:Float, center:{x:Float, y:Float, z:Float}
	} {
		var volume = width * depth * height, mx = 0.0, my = 0.0, mz = volume * height / 2;
		var occupied:Array<Box3> = [];
		for (hole in holes) {
			var normal = materia.assembly.AssemblyFrames.transformVector(hole.frame, 0, 0, 1);
			var components = [normal.x, normal.y, normal.z];
			var axis = axis(hole);
			var low = [-width / 2, -depth / 2, 0.0], high = [width / 2, depth / 2, height];
			var entry = [hole.frame.x, hole.frame.y, hole.frame.z];
			var expected = components[axis] > 0 ? low[axis] : high[axis];
			if (Math.abs(entry[axis] - expected) > 1e-7)
				throw "Insert hole entry must lie on its stock face: " + hole.id;
			var available = high[axis] - low[axis];
			if (hole.extent == Through && hole.depth < available - 1e-7
				|| hole.extent == Blind && hole.depth > available + 1e-7)
				throw "Insert hole depth disagrees with its extent: " + hole.id;
			var length = Math.min(available, hole.depth);
			var radius = maximumRadius(hole);
			var boundsLow = [for (v in entry) v - radius], boundsHigh = [for (v in entry) v + radius];
			boundsLow[axis] = Math.min(entry[axis], entry[axis] + components[axis] * length);
			boundsHigh[axis] = Math.max(entry[axis], entry[axis] + components[axis] * length);
			for (index in 0...3)
				if (boundsLow[index] < low[index] - 1e-7 || boundsHigh[index] > high[index] + 1e-7)
					throw "Insert drilling breaks out of its stock: " + hole.id;
			var bounds:Box3 = {
				x: boundsLow[0], y: boundsLow[1], z: boundsLow[2],
				width: boundsHigh[0] - boundsLow[0], depth: boundsHigh[1] - boundsLow[1], height: boundsHigh[2] - boundsLow[2]
			};
			for (other in occupied)
				if (BoxAlgebra.intersection(bounds, other) != null)
					throw "Unsupported overlapping insert drilling mass: " + hole.id;
			occupied.push(bounds);
			var removed = Math.PI * hole.diameter * hole.diameter / 4 * length;
			var center = materia.assembly.AssemblyFrames.transformPoint(hole.frame, 0, 0, length / 2);
			volume -= removed;
			mx -= removed * center.x;
			my -= removed * center.y;
			mz -= removed * center.z;
			if (hole.counterbore != null) {
				var cb = hole.counterbore;
				removed = Math.PI * (cb.diameter * cb.diameter - hole.diameter * hole.diameter) / 4 * cb.depth;
				center = materia.assembly.AssemblyFrames.transformPoint(hole.frame, 0, 0, cb.depth / 2);
				volume -= removed;
				mx -= removed * center.x;
				my -= removed * center.y;
				mz -= removed * center.z;
			}
			if (hole.relief != null) {
				var relief = hole.relief, reliefLength = length - relief.start;
				removed = Math.PI * (relief.diameter * relief.diameter - hole.diameter * hole.diameter) / 4 * reliefLength;
				center = materia.assembly.AssemblyFrames.transformPoint(hole.frame, 0, 0, relief.start + reliefLength / 2);
				volume -= removed;
				mx -= removed * center.x;
				my -= removed * center.y;
				mz -= removed * center.z;
			}
		}
		if (!Math.isFinite(volume) || volume <= 0)
			throw "Insert drilling consumes its stock";
		return {volume: volume, center: {x: mx / volume, y: my / volume, z: mz / volume}};
	}
}
