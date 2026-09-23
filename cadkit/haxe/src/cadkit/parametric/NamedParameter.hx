package cadkit.parametric;

import cadkit.parametric.ParameterExpression;
import cadkit.parametric.ParameterKind;
import cadkit.parametric.UnitConversion;

/** Document-owned typed value, optionally derived from an expression and bound to feature slots. */
class NamedParameter {
	public final document:Document;
	public final name:String;
	public final kind:String;
	public final unit:String;
	public var expression(default, null):Null<ParameterExpression>;
	public var value(get, never):Float;

	private var storedValue:Float;
	private final targets:Array<Parameter>;

	public function new(document:Document, name:String, canonicalValue:Float, kind:String = "scalar", unit:String = "1",
		?expression:ParameterExpression) {
		this.document = document;
		this.name = name;
		this.kind = ParameterKind.validate(kind);
		this.unit = UnitConversion.validateUnit(this.kind, unit);
		this.expression = expression;
		storedValue = canonicalValue;
		targets = [];
	}

	function get_value():Float {
		if (document.isClosed())
			throw new ParametricError("document is closed");
		if (expression != null)
			return document.expressionValue(this);
		return targets.length == 0 ? storedValue : targets[0].value;
	}

	public function valueIn(requestedUnit:String):Float {
		return UnitConversion.fromCanonical(value, kind, requestedUnit);
	}

	public function bindings():Array<Parameter> {
		return targets.copy();
	}

	public function dependencies():Array<String> {
		return expression == null ? [] : expression.dependencies.copy();
	}

	public function contains(parameter:Parameter):Bool {
		return targets.indexOf(parameter) >= 0;
	}

	public function bind(parameter:Parameter):Void {
		if (document.isClosed() || parameter.ownerFeature().document != document)
			throw new ParametricError("named parameter binding belongs to another document");
		if (!parameter.isRegistered())
			throw new ParametricError("cannot bind an inactive feature parameter: " + parameter.name);
		if (document.parameter(name) != this)
			throw new ParametricError("named parameter is not registered");
		if (contains(parameter))
			throw new ParametricError("duplicate named parameter binding");
		if (kind != ParameterKind.Scalar && parameter.kind != kind)
			throw new ParametricError("named parameter type " + kind + " cannot bind to " + parameter.kind + " feature parameter " + parameter.name);
		for (named in document.namedParameters())
			if (named != this && named.contains(parameter))
				throw new ParametricError("feature parameter is already named");
		var canonical = value;
		parameter.validateValue(canonical);
		if (parameter.value != canonical)
			throw new ParametricError("named parameter binding value differs from its dimension");
		targets.push(parameter);
	}

	public function unbind(parameter:Parameter):Void {
		if (document.isClosed())
			throw new ParametricError("document is closed");
		var index = targets.indexOf(parameter);
		if (index < 0)
			throw new ParametricError("feature parameter is not bound to " + name);
		if (targets.length == 1)
			storedValue = parameter.value;
		targets.splice(index, 1);
	}

	public function set(next:Float, ?inputUnit:String):Void {
		if (document.isClosed())
			throw new ParametricError("document is closed");
		if (expression != null)
			throw new ParametricError("expression parameters are read-only");
		var canonical = UnitConversion.toCanonical(next, kind, inputUnit == null ? unit : inputUnit);
		if (targets.length == 0) {
			document.setStandaloneNamedParameter(this, canonical);
			return;
		}
		targets[0].set(canonical);
	}

	public function restoreStored(next:Float):Void {
		storedValue = next;
	}

	public function replaceExpression(next:Null<ParameterExpression>):Void {
		expression = next;
	}

	public function synchronize(next:Float):Void {
		validateCanonical(next);
		validateSynchronized(next);
		storedValue = next;
		for (target in targets)
			target.restore(next);
	}

	public function validateSynchronized(next:Float):Void {
		for (target in targets)
			target.validateValue(next);
	}

	public function validateCanonical(next:Float):Void {
		if (!Math.isFinite(next))
			throw new ParametricError("named parameter must be finite: " + name);
		if (kind == ParameterKind.Count && next != Std.int(next))
			throw new ParametricError("count parameter must be an integer: " + name);
	}
}
