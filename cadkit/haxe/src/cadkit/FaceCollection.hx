package cadkit;

import cadkit.FaceQuery;

/** Lazy indexed faces; no native face handles are allocated until at(). */
class FaceCollection {
	private final owner:Shape;

	public function new(owner:Shape) {
		this.owner = owner;
	}

	public function count():Int {
		return owner.subshapeCount(CadKit.ShapeKind.Face);
	}

	public function at(index:Int):Face {
		return new Face(owner.subshape(CadKit.ShapeKind.Face, index), index);
	}

	public function first(predicate:Face->Bool):Null<Face> {
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

	public function query():FaceQuery {
		return FaceQuery.from(owner);
	}

	public function all():Array<Face> {
		return query().all();
	}

	public function unique():Face {
		return query().unique();
	}

}
