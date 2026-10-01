package cadkit.sketch;

/**
	How a part is recognised across solves (see `CachedPart`, `PartStructure`):
	its structure as a string key, and the numbers its solution depends on,
	compared exactly rather than through strings.
*/
class SketchSolveCache {
	/** The part's variables in order, constraint IDs, kinds and references, and the entities they reach. */
	public static function key(layout:SketchLayout, part:SketchPart):String {
		var key = new StringBuf(), field = String.fromCharCode(1), record = String.fromCharCode(2);
		for (variable in part.variables) { key.add(layout.variableNames[variable]); key.add(field); }
		key.add(record);
		for (index in part.constraints) {
			var c = layout.constraintList[index];
			key.add(c.id); key.add(field); key.add(c.kind);
			for (id in [c.first, c.second, c.third]) {
				key.add(field);
				if (id == null) continue;
				key.add(id);
				var e = layout.entities.get(id);
				if (e != null) { key.add("="); key.add(e.kind); key.add(":"); key.add(e.first); key.add(","); key.add(e.second == null ? "" : e.second); }
			}
			key.add(record);
		}
		return key.toString();
	}

	/** Everything numeric a part's solution depends on besides its seed: settings, scale, dimension values, fixed positions. */
	public static function values(layout:SketchLayout, part:SketchPart):Array<Float> {
		var settings = layout.sketch.settings;
		var values = [settings.tolerance, settings.rankTolerance, settings.maxIterations, settings.initialDamping, layout.normalizationScale];
		for (index in part.constraints) {
			var c = layout.constraintList[index];
			values.push(c.value);
			if (c.kind == "fixed") {
				var p = layout.points.get(c.first);
				if (p != null) { values.push(p.x); values.push(p.y); }
			}
		}
		return values;
	}

	public static function sameValues(a:Array<Float>, b:Array<Float>):Bool {
		if (a.length != b.length) return false;
		for (i in 0...a.length) if (a[i] != b[i]) return false;
		return true;
	}
}
