package cadkit.modeling;

import CadKit;
import cadkit.Shape;

class BuildPart {
	private var current:Null<Part>;
	private var closed:Bool;

	public function new() {
		current = null;
		closed = false;
	}

	private function check():Void {
		if (closed)
			throw "builder is closed";
	}

	public function add(part:Part, mode:Mode = Add):Void {
		check();
		if (current == null) {
			if (mode != Add)
				throw "first part operation must add";
			current = new Part(part.shape.cloneShape());
			return;
		}
		var next = current.combine(part, mode);
		current.close();
		current = next;
	}

	private function temporary(part:Part, mode:Mode):Void {
		try {
			add(part, mode);
			part.close();
		} catch (error:Dynamic) {
			part.close();
			throw error;
		}
	}

	public function box(width:Float, depth:Float, height:Float, mode:Mode = Add):Void {
		temporary(Part.box(width, depth, height), mode);
	}

	public function extrude(sketch:Sketch, amount:Float, mode:Mode = Add):Void {
		check();
		var part = sketch.extrude(amount);
		// Preserve extrusion history when it is the first operation.
		if (current == null && mode == Add) {
			current = part;
			return;
		}
		temporary(part, mode);
	}

	public function pattern(part:Part, locations:Array<Location>, mode:Mode = Add):Void {
		for (location in locations)
			temporary(part.placed(location), mode);
	}

	public function edges():Selection {
		check();
		if (current == null)
			return new Selection([]);
		return current.edges();
	}

	public function faces():Selection {
		check();
		if (current == null)
			return new Selection([]);
		return current.faces();
	}

	public function generated(kind:CadKit.ShapeKind):Selection {
		check();
		if (current == null)
			return new Selection([]);
		return current.generated(kind);
	}

	public function fillet(selection:Selection, radius:Float):Void {
		check();
		if (current == null)
			throw "fillet requires a part";
		var next = current.fillet(selection, radius);
		current.close();
		current = next;
	}

	public function chamfer(selection:Selection, distance:Float):Void {
		check();
		if (current == null)
			throw "chamfer requires a part";
		var next = current.chamfer(selection, distance);
		current.close();
		current = next;
	}

	public function finish():Part {
		check();
		if (current == null)
			return new Part(Shape.fromOwnedHandle(CadKit.compoundChecked([])));
		var result = current;
		current = null;
		return result;
	}

	public function close():Void {
		if (closed)
			return;
		closed = true;
		if (current != null)
			current.close();
		current = null;
	}

	public static function build(callback:BuildPart->Void):Part {
		var builder = new BuildPart();
		try {
			callback(builder);
			var result = builder.finish();
			builder.close();
			return result;
		} catch (error:Dynamic) {
			builder.close();
			throw error;
		}
	}
}
