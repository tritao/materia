package machinekit.component;

import machinekit.component.ComponentValue.*;

/** Editable, typed inputs for a named cutting tool produced by a component. */
class ToolSpec {
	public final name:String;
	final inputs:Array<ComponentParameter>;

	public function new(name:String, inputs:Array<ComponentParameter>) {
		if (name == null || name.length == 0 || inputs == null) throw "Incomplete tool specification";
		this.name = name;
		this.inputs = inputs.copy();
		var seen:Map<String, Bool> = [];
		for (input in inputs) {
			if (seen.exists(input.name)) throw 'Duplicate tool input "${input.name}" for "$name"';
			seen.set(input.name, true);
		}
	}

	public function parameters():Array<ComponentParameter> return inputs.copy();

	public function defaults():ComponentValues {
		var result = new ComponentValues();
		for (input in inputs) result.set(input.name, input.defaultValue);
		return result;
	}

	public function resolve(?overrides:ComponentValues):ComponentValues {
		var result = defaults();
		if (overrides == null) return result;
		for (name in overrides.names()) {
			var input:Null<ComponentParameter> = null;
			for (candidate in inputs) if (candidate.name == name) input = candidate;
			if (input == null) throw 'Unknown input "$name" for tool "$this.name"';
			var value = overrides.get(name);
			input.validate(value);
			result.set(name, value);
		}
		return result;
	}

	public static function inputName(toolName:String, parameterName:String):String
		return 'tool_${toolName}_$parameterName';
}
