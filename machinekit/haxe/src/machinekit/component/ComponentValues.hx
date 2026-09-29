package machinekit.component;

import machinekit.component.ComponentValue.*;

/** Named typed values for one component recipe. */
class ComponentValues {
	final entries:Map<String, ComponentValue> = [];

	public function new() {}

	public function set(name:String, value:ComponentValue):ComponentValues {
		if (name == null || name.length == 0 || value == null) throw "Component value needs a name and value";
		entries.set(name, value);
		return this;
	}

	public function get(name:String):Null<ComponentValue> return entries.get(name);
	public function setNumber(name:String, value:Float):ComponentValues return set(name, Number(value));
	public function setInteger(name:String, value:Int):ComponentValues return set(name, Integer(value));
	public function setBoolean(name:String, value:Bool):ComponentValues return set(name, Boolean(value));
	public function setToken(name:String, value:String):ComponentValues return set(name, Token(value));
	public function setUnset(name:String):ComponentValues return set(name, Unset);
	public function names():Array<String> return [for (name in entries.keys()) name];

	public function number(name:String):Float return switch (required(name)) {
		case Number(value): value;
		default: throw 'Component value "$name" is not numeric';
	};

	public function optionalNumber(name:String):Null<Float> return switch (required(name)) {
		case Unset: null;
		case Number(value): value == 0 ? null : value;
		default: throw 'Component value "$name" is not optional numeric';
	};

	public function optionalToken(name:String):Null<String> return switch (required(name)) {
		case Unset: null;
		case Token(value): value;
		default: throw 'Component value "$name" is not optional text';
	};

	public function integer(name:String):Int return switch (required(name)) {
		case Integer(value): value;
		default: throw 'Component value "$name" is not an integer';
	};

	public function boolean(name:String):Bool return switch (required(name)) {
		case Boolean(value): value;
		default: throw 'Component value "$name" is not boolean';
	};

	public function token(name:String):String return switch (required(name)) {
		case Token(value): value;
		default: throw 'Component value "$name" is not a token';
	};

	public function copy():ComponentValues {
		var result = new ComponentValues();
		for (name in entries.keys()) result.set(name, entries.get(name));
		return result;
	}

	function required(name:String):ComponentValue {
		var value = entries.get(name);
		if (value == null) throw 'Missing component value "$name"';
		return value;
	}
}
