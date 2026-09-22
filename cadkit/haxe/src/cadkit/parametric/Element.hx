package cadkit.parametric;

import cadkit.Shape;
import cadkit.parametric.Document;
import cadkit.parametric.ElementId;
import cadkit.parametric.Feature;
import cadkit.parametric.ParametricError;

/** Persistent identity and metadata for one independently addressable document output. */
class Element {
	public final document:Document;
	public final id:ElementId;
	public var name(default, null):String;
	public var output(default, null):Feature;

	public function new(document:Document, id:ElementId, name:String, output:Feature) {
		this.document = document;
		this.id = id;
		this.name = name;
		this.output = output;
	}

	public function shape():Shape {
		var result = output.currentShape();
		if (result == null)
			throw new ParametricError("element output has not been evaluated: " + id.value);
		return result;
	}

	public function restoreName(value:String):Void name = value;
	public function restoreOutput(value:Feature):Void output = value;
}
