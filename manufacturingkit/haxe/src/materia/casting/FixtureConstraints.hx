package materia.casting;

typedef FixtureVector = {
	var x:Float;
	var y:Float;
	var z:Float;
}

enum abstract FixtureConstraintRole(String) to String {
	var Location = "location";
	var Preload = "preload";
}

enum abstract FixtureCoordinate(String) to String {
	var Tx = "translation-x";
	var Ty = "translation-y";
	var Tz = "translation-z";
	var Rx = "rotation-x";
	var Ry = "rotation-y";
	var Rz = "rotation-z";
}

/** One contact direction in the insert's common fixture frame. Source IDs trace
 * to the planned pads, pin/hole or slot contacts; a clamp is preload, not a datum.
 */
typedef FixtureConstraint = {
	var id:String;
	var sourceId:String;
	var role:FixtureConstraintRole;
	var point:FixtureVector;
	var normal:FixtureVector;
}

typedef FixtureMotion = {
	var translationMm:FixtureVector;
	var rotationRad:FixtureVector;
	var affectedCoordinates:Array<FixtureCoordinate>;
}

typedef FixtureDependency = {
	var constraintIds:Array<String>;
	var coefficients:Array<Float>;
	var affectedCoordinates:Array<FixtureCoordinate>;
}

typedef FixtureConstraintReport = {
	var rank:Int;
	var freeMotions:Array<FixtureMotion>;
	var redundantLocations:Array<FixtureDependency>;
	var preloadIds:Array<String>;
}

/** Infinitesimal six-DOF location check. Rows are n·(t + rotation × lever).
 * Rotation coordinates are scaled by the declared reference length before
 * elimination; output converts them back to radians. This is a location/rank
 * check, not proof of preload, strength, pour access or post-cure removal.
 */
class FixtureConstraints {
	public static function analyze(contacts:Array<FixtureConstraint>, origin:FixtureVector,
			referenceLengthMm:Float, rankTolerance:Float):FixtureConstraintReport {
		vector(origin);
		if (!Math.isFinite(referenceLengthMm) || referenceLengthMm <= 0 || !Math.isFinite(rankTolerance)
			|| rankTolerance <= 0 || rankTolerance >= 1) throw "Invalid fixture rank scale or tolerance";
		if (contacts == null || contacts.length > 256) throw "Invalid fixture contact list";
		var sorted = contacts.copy();
		var ids:Map<String,Bool> = [];
		for (c in sorted) {
			CastingFeatureCodec.identifier(c.id);
			CastingFeatureCodec.identifier(c.sourceId);
			if (ids.exists(c.id)) throw "Duplicate fixture constraint: " + c.id;
			ids.set(c.id, true);
			if (c.role != Location && c.role != Preload) throw "Unknown fixture constraint role: " + c.id;
			vector(c.point);
			vector(c.normal);
			var n = c.normal;
			if (Math.abs(n.x * n.x + n.y * n.y + n.z * n.z - 1) > 1e-8) throw "Fixture contact normal must be unit length: " + c.id;
		}
		sorted.sort((a, b) -> Reflect.compare(a.id, b.id));
		var locating = sorted.filter(c -> c.role == Location);
		var preloads = [for (c in sorted) if (c.role == Preload) c.id];
		var original:Array<Array<Float>> = [];
		for (c in locating) {
			var n = c.normal, x = c.point.x - origin.x, y = c.point.y - origin.y, z = c.point.z - origin.z;
			var row = [n.x, n.y, n.z, (y * n.z - z * n.y) / referenceLengthMm,
				(z * n.x - x * n.z) / referenceLengthMm, (x * n.y - y * n.x) / referenceLengthMm];
			for (value in row)
				if (!Math.isFinite(value)) throw "Nonfinite fixture constraint row: " + c.id;
			original.push(row);
		}
		var a = [for (row in original) row.copy()];
		var combinations:Array<Array<Float>> = [for (i in 0...a.length) [for (j in 0...a.length) i == j ? 1.0 : 0.0]];
		var pivots:Array<Int> = [], rank = 0;
		for (column in 0...6) {
			var chosen = -1, largest = rankTolerance;
			for (row in rank...a.length) {
				var value = Math.abs(a[row][column]);
				if (value > largest) {
					largest = value;
					chosen = row;
				}
			}
			if (chosen < 0) continue;
			var temporary = a[rank];
			a[rank] = a[chosen];
			a[chosen] = temporary;
			temporary = combinations[rank];
			combinations[rank] = combinations[chosen];
			combinations[chosen] = temporary;
			var divisor = a[rank][column];
			for (j in 0...6) a[rank][j] /= divisor;
			for (j in 0...a.length) combinations[rank][j] /= divisor;
			for (row in 0...a.length) if (row != rank) {
				var factor = a[row][column];
				for (j in 0...6) a[row][j] -= factor * a[rank][j];
				for (j in 0...a.length) combinations[row][j] -= factor * combinations[rank][j];
			}
			pivots.push(column);
			rank++;
		}
		for (matrix in [a, combinations])
			for (row in matrix)
				for (value in row)
					if (!Math.isFinite(value)) throw "Ill-conditioned fixture constraints; revise the reference scale";
		var free:Array<FixtureMotion> = [];
		for (column in 0...6) if (pivots.indexOf(column) < 0) {
			var motion = [0.0, 0, 0, 0, 0, 0];
			motion[column] = 1;
			for (row in 0...rank) motion[pivots[row]] = -a[row][column];
			free.push({
				translationMm: {x: motion[0], y: motion[1], z: motion[2]},
				rotationRad: {x: motion[3] / referenceLengthMm, y: motion[4] / referenceLengthMm, z: motion[5] / referenceLengthMm},
				affectedCoordinates: coordinates(motion, rankTolerance)
			});
		}
		var redundant:Array<FixtureDependency> = [];
		for (row in rank...a.length) {
			var weights = combinations[row], maximum = 0.0, sign = 0.0;
			for (value in weights) if (Math.abs(value) > rankTolerance) {
				maximum = Math.max(maximum, Math.abs(value));
				if (sign == 0) sign = value > 0 ? 1 : -1;
			}
			if (maximum == 0) throw "Invalid fixture dependency";
			var memberIds:Array<String> = [], coefficients:Array<Float> = [], directions = [0.0, 0, 0, 0, 0, 0];
			for (i in 0...weights.length) if (Math.abs(weights[i] / maximum) > rankTolerance) {
				memberIds.push(locating[i].id);
				coefficients.push(sign * weights[i] / maximum);
				for (j in 0...6) directions[j] = Math.max(directions[j], Math.abs(original[i][j]));
			}
			redundant.push({constraintIds: memberIds, coefficients: coefficients,
				affectedCoordinates: coordinates(directions, rankTolerance)});
		}
		redundant.sort((a, b) -> Reflect.compare(a.constraintIds.join("|"), b.constraintIds.join("|")));
		return {rank: rank, freeMotions: free, redundantLocations: redundant, preloadIds: preloads};
	}

	static function coordinates(row:Array<Float>, tolerance:Float):Array<FixtureCoordinate> {
		var names:Array<FixtureCoordinate> = [Tx, Ty, Tz, Rx, Ry, Rz];
		return [for (i in 0...6) if (Math.abs(row[i]) > tolerance) names[i]];
	}
	static function vector(v:FixtureVector):Void {
		if (v == null || !Math.isFinite(v.x) || !Math.isFinite(v.y) || !Math.isFinite(v.z)) throw "Invalid fixture vector";
	}
}
