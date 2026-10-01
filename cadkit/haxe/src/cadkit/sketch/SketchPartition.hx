package cadkit.sketch;

import cadkit.solve.EnvelopeCholesky;

/**
	A sketch's parts: constraints grouped by what they reference (points, line
	ends, centres, radii), not by Jacobian entries, which can be zero by
	accident. Variables no constraint references belong to no part: they are
	free. Keeps each variable's part and local index.
*/
class SketchPartition {
	public final parts:Array<SketchPart> = [];
	/** The variables each constraint references (a superset of any pose's nonzeros). */
	public final references:Array<Array<Int>>;
	final variablePart:Array<Int>;
	final variableLocal:Array<Int>;

	public function new(layout:SketchLayout) {
		var count = layout.variableCount;
		var parent = [for (i in 0...count) i];
		references = [for (c in layout.constraintList) referencedVariables(layout, c)];
		for (list in references)
			for (k in 1...list.length)
				union(parent, list[0], list[k]);
		var slot = [for (_ in 0...count) -1];
		for (index in 0...layout.constraintList.length) {
			if (references[index].length == 0) continue;
			var root = find(parent, references[index][0]);
			if (slot[root] < 0) {
				slot[root] = parts.length;
				parts.push({id: parts.length, constraints: [], variables: [], position: [], first: [], ordered: false, envelope: []});
			}
			parts[slot[root]].constraints.push(index);
		}
		variablePart = [for (_ in 0...count) -1];
		variableLocal = [for (_ in 0...count) -1];
		for (variable in 0...count) {
			var id = slot[find(parent, variable)];
			if (id < 0) continue;
			variablePart[variable] = id;
			variableLocal[variable] = parts[id].variables.length;
			parts[id].variables.push(variable);
		}
	}

	public inline function partOf(variable:Int):Int return variablePart[variable];
	public inline function localIndex(variable:Int):Int return variableLocal[variable];

	/**
		Reverse Cuthill-McKee over the part's variable graph (two variables are
		adjacent when one constraint references both), then the envelope of its
		normal matrix in that order. Chains and grids keep a narrow band.
	*/
	public function order(part:SketchPart):Void {
		if (part.ordered) return;
		var k = part.variables.length;
		var neighbours:Array<Array<Int>> = [for (_ in 0...k) []];
		for (index in part.constraints) {
			var list = [for (v in references[index]) variableLocal[v]];
			for (a in list) for (b in list)
				if (a != b && neighbours[a].indexOf(b) < 0) neighbours[a].push(b);
		}
		var ordering = EnvelopeCholesky.order(neighbours);
		part.position = ordering.position;
		part.first = ordering.first;
		part.ordered = true;
	}

	/** A row's entries in the part's local numbering, repeated variables added together. */
	public function localEntries(row:SketchSparseRow, part:SketchPart):SketchSparseRow {
		var index:Array<Int> = [], value:Array<Float> = [];
		for (e in 0...row.index.length) {
			if (variablePart[row.index[e]] != part.id) continue;
			var local = variableLocal[row.index[e]];
			var at = index.indexOf(local);
			if (at < 0) { index.push(local); value.push(row.value[e]); } else value[at] += row.value[e];
		}
		return {index: index, value: value};
	}

	static function referencedVariables(layout:SketchLayout, c:SketchConstraint):Array<Int> {
		var result:Array<Int> = [];
		for (id in [c.first, c.second, c.third]) {
			if (id == null) continue;
			var p = layout.pointIndex.get(id);
			if (p != null) { result.push(p); result.push(p + 1); continue; }
			var e = layout.entities.get(id);
			if (e == null) continue;
			var first = layout.pointIndex.get(e.first);
			if (first != null) { result.push(first); result.push(first + 1); }
			if (e.second != null) { var second = layout.pointIndex.get(e.second); if (second != null) { result.push(second); result.push(second + 1); } }
			var r = layout.radiusIndex.get(id);
			if (r != null) result.push(r);
		}
		return result;
	}

	static function find(parent:Array<Int>, i:Int):Int {
		while (parent[i] != i) {
			parent[i] = parent[parent[i]];
			i = parent[i];
		}
		return i;
	}

	static function union(parent:Array<Int>, a:Int, b:Int):Void {
		var ra = find(parent, a), rb = find(parent, b);
		if (ra < rb) parent[rb] = ra; else if (rb < ra) parent[ra] = rb;
	}
}
