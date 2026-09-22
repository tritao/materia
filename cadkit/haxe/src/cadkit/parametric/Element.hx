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
	public final kind:String;
	public var name(default, null):String;
	public var output(default, null):Null<Feature>;
	private var committedOutput:Null<Feature>;

	public function new(document:Document, id:ElementId, name:String, kind:String, ?output:Feature) {
		this.document = document;
		this.id = id;
		this.name = name;
		this.kind = kind;
		this.output = output;
		committedOutput = output;
	}

	public function shape():Shape {
		if (committedOutput == null)
			throw new ParametricError("element has no geometry output: " + id.value);
		var result = committedOutput.currentShape();
		if (result == null)
			throw new ParametricError("element output has not been evaluated: " + id.value);
		return result;
	}

	public function restoreName(value:String):Void name = value;
	public function restoreOutput(value:Feature):Void output = value;
	public function commitOutput():Void committedOutput = output;

	public function rename(value:String):Void document.renameElement(this, value);
	public function setOutput(value:Feature):Void document.setElementOutput(this, value);
}
