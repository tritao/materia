package cadkit;

import cadkit.EdgeQuery;

/** Lazy indexed edges; no native edge handles are allocated until at(). */
class EdgeCollection {
	private final owner:Shape;

	public function new(owner:Shape) {
		this.owner = owner;
	}

	public function count():Int {
		return owner.subshapeCount(CadKit.ShapeKind.Edge);
	}

	public function at(index:Int):Edge {
		return new Edge(owner.subshape(CadKit.ShapeKind.Edge, index));
	}

	public function first(predicate:Edge->Bool):Null<Edge> {
		var index = 0;
		var total = count();
		while (index < total) {
			var value = at(index);
			if (predicate(value))
				return value;
			value.close();
			index++;
		}
		return null;
	}

	public function query():EdgeQuery {
		return EdgeQuery.from(owner);
	}

	public function all():Array<Edge> {
		return query().all();
	}

	public function unique():Edge {
		return query().unique();
	}

}
