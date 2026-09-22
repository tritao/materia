package cadkit.parametric;

/** Document-owned shared dimension. Bindings are structural; scalar edits are undoable. */
class NamedParameter {
	public final document:Document;
	public final name:String;
	public var value(get, never):Float;

	private final initialValue:Float;
	private final targets:Array<Parameter>;

	public function new(document:Document, name:String, value:Float) {
		this.document = document;
		this.name = name;
		initialValue = value;
		targets = [];
	}

	function get_value():Float {
		if (document.isClosed())
			throw new ParametricError("document is closed");
		return targets.length == 0 ? initialValue : targets[0].value;
	}

	public function bindings():Array<Parameter> {
		return targets.copy();
	}

	public function contains(parameter:Parameter):Bool {
		return targets.indexOf(parameter) >= 0;
	}

	public function bind(parameter:Parameter):Void {
		if (document.isClosed() || parameter.ownerFeature().document != document)
			throw new ParametricError("named parameter binding belongs to another document");
		if (document.parameter(name) != this)
			throw new ParametricError("named parameter is not registered");
		if (contains(parameter))
			throw new ParametricError("duplicate named parameter binding");
		for (named in document.namedParameters())
			if (named != this && named.contains(parameter))
				throw new ParametricError("feature parameter is already named");
		parameter.validateValue(value);
		if (parameter.value != value)
			throw new ParametricError("named parameter binding value differs from its dimension");
		targets.push(parameter);
	}

	public function set(next:Float):Void {
		if (document.isClosed())
			throw new ParametricError("document is closed");
		if (targets.length == 0)
			throw new ParametricError("named parameter has no feature bindings: " + name);
		// Parameter.set delegates to the document so direct feature edits obey sharing too.
		targets[0].set(next);
	}
}
