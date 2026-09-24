package cadkit.parametric;

/** Persistent typed edge between two document elements. */
class Relationship {
	public final document:Document;
	public final id:RelationshipId;
	public final typeName:String;
	public var source(default, null):ElementReference;
	public var target(default, null):ElementReference;

	private final propertyValues:Map<String, TypedProperty>;

	public function new(document:Document, id:RelationshipId, typeName:String, source:ElementReference, target:ElementReference) {
		if (typeName == null || StringTools.trim(typeName) == "")
			throw new ParametricError("relationship type name must not be empty");
		if (source == null || target == null)
			throw new ParametricError("relationship endpoints must not be null");
		this.document = document;
		this.id = id;
		this.typeName = typeName;
		this.source = source;
		this.target = target;
		propertyValues = new Map();
	}

	public function sourceElement():Element
		return document.resolveElement(source);

	public function targetElement():Element
		return document.resolveElement(target);

	/** Internal document history path. */
	public function restoreEndpoints(source:ElementReference, target:ElementReference):Void {
		this.source = source;
		this.target = target;
	}

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
		document.setRelationshipProperty(this, value);

	public function removeProperty(name:String):Void
		document.removeRelationshipProperty(this, name);

	public function remove():Void
		document.removeRelationship(id);
}
