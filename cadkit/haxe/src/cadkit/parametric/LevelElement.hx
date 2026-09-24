package cadkit.parametric;

import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.UnitConversion;

class LevelElement extends Element {
	public var elevation(default, null):Float;
	public var offset(default, null):Float;
	public var relativeTo(default, null):Null<ElementReference>;

	public function new(document:Document, id:ElementId, name:String, elevation:Float, offset:Float = 0, ?relativeTo:ElementReference) {
		super(document, id, name, ElementKind.Level);
		if (!Math.isFinite(elevation) || !Math.isFinite(offset)) throw new ParametricError("level elevations must be finite");
		this.elevation=elevation; this.offset=offset; this.relativeTo=relativeTo;
	}
	public function setElevation(value:Float, unit:String="mm"):Void document.setLevelElevation(this, UnitConversion.toCanonical(value,"length",unit));
	public function restoreElevation(value:Float):Void { elevation=value; document.datumChanged(this); }
}
