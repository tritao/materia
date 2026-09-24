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
	public var localPlacement(default, null):Placement;
	public var placementParent(default, null):Null<ElementReference>;
	public var placementDerived(default, null):Bool;

	private final propertyValues:Map<String, TypedProperty>;
	private var committedOutput:Null<Feature>;
	private var directShape:Null<Shape>;
	private var placedShape:Null<Shape>;

	public function new(document:Document, id:ElementId, name:String, kind:String, ?output:Feature) {
		this.document = document;
		this.id = id;
		this.name = name;
		this.kind = kind;
		this.output = output;
		committedOutput = output;
		localPlacement = Placement.identity();
		placementParent = null;
		placementDerived = false;
		propertyValues = new Map();
		placedShape = null;
		directShape = null;
	}

	public function shape():Shape {
		var result:Null<Shape> = directShape;
		if (result == null && committedOutput != null)
			result = committedOutput.currentShape();
		if (result == null)
			throw new ParametricError("element has no committed geometry: " + id.value);
		var world = document.worldPlacement(this);
		if (world.isIdentity())
			return result;
		if (placedShape == null)
			placedShape = world.location.apply(result);
		return placedShape;
	}

	public function restoreName(value:String):Void
		name = value;

	public function property(name:String):Null<TypedProperty>
		return propertyValues.get(name);

	public function properties():Array<TypedProperty> {
		var names = [for (name in propertyValues.keys()) name];
		names.sort(Reflect.compare);
		return [for (name in names) propertyValues.get(name)];
	}

	/** Internal document history and codec path. */
	public function restoreProperty(name:String, value:Null<TypedProperty>):Void {
		if (value == null)
			propertyValues.remove(name);
		else
			propertyValues.set(name, value);
	}

	public function setProperty(value:TypedProperty):Void
		document.setElementProperty(this, value);

	public function removeProperty(name:String):Void
		document.removeElementProperty(this, name);

	public function restoreOutput(value:Feature):Void {
		output = value;
		clearPlacedShape();
	}

	public function cachedPlacedShape():Null<Shape>
		return placedShape;

	/** Publish the authored output pointer; the document retires the old placement cache afterward. */
	public function commitOutput():Void {
		committedOutput = output;
		placedShape = null;
	}

	public function restorePlacement(value:Placement, parent:Null<ElementReference>):Void {
		localPlacement = value;
		placementParent = parent;
		clearPlacedShape();
	}

	/** Generic authority marker used by adapters that derive placement from other authored data. */
	public function restorePlacementDerived(value:Bool):Void
		placementDerived = value;

	public function clearPlacedShape():Void {
		if (placedShape != null) {
			placedShape.close();
			placedShape = null;
		}
	}

	public function restoreDirectShape(value:Null<Shape>):Void {
		directShape = value;
		clearPlacedShape();
	}

	public function rename(value:String):Void
		document.renameElement(this, value);

	public function setOutput(value:Feature):Void
		document.setElementOutput(this, value);

	public function setPlacement(value:Placement):Void
		document.setElementPlacement(this, value);

	public function reparent(parent:Null<ElementReference>, preserveWorld:Bool):Void
		document.reparentElement(this, parent, preserveWorld);
}
