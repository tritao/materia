package cadkit;

import cadkit.VertexQuery;

/** Lazy indexed vertices; no native vertex handles are allocated until at(). */
class VertexCollection {
	private final owner:Shape;

	public function new(owner:Shape) {
		this.owner = owner;
	}

	public function count():Int {
		return owner.subshapeCount(CadKit.ShapeKind.Vertex);
	}

	public function at(index:Int):Vertex {
		return new Vertex(owner.subshape(CadKit.ShapeKind.Vertex, index));
	}

	public function first(predicate:Vertex->Bool):Null<Vertex> {
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

	public function query():VertexQuery {
		return VertexQuery.from(owner);
	}

	public function all():Array<Vertex> {
		return query().all();
	}

	public function unique():Vertex {
		return query().unique();
	}

}
