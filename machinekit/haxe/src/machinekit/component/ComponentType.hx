package machinekit.component;

import machinekit.component.ComponentValue.*;

/** Stable recipe for one kind of single part. */
class ComponentType {
	public final id:String;
	final inputs:Array<ComponentParameter>;
	final build:ComponentValues->MachineComponent;
	final keepDesignation:Bool;

	public function new(id:String, inputs:Array<ComponentParameter>, build:ComponentValues->MachineComponent,
			keepDesignation:Bool = false) {
		if (id == null || id.length == 0 || inputs == null || build == null)
			throw "Incomplete component type";
		this.id = id;
		this.inputs = inputs.copy();
		this.build = build;
		this.keepDesignation = keepDesignation;
		var seen:Map<String, Bool> = [];
		for (input in inputs) {
			if (seen.exists(input.name)) throw 'Duplicate component input "${input.name}"';
			seen.set(input.name, true);
		}
	}

	public function parameters():Array<ComponentParameter> return inputs.copy();
	public function matches(component:MachineComponent):Bool return component.componentType() == this;

	public function defaults():ComponentValues return resolve(null);

	public function resolve(?overrides:ComponentValues):ComponentValues {
		var result = new ComponentValues();
		if (overrides != null)
			for (name in overrides.names())
				if (parameter(name) == null) throw 'Unknown input "$name" for component type "$id"';
		for (input in inputs) {
			var value = overrides == null ? null : overrides.get(input.name);
			if (value == null) value = input.defaultValue;
			input.validate(value);
			result.set(input.name, value);
		}
		return result;
	}

	public function create(?values:ComponentValues):MachineComponent
		return build(resolve(values));

	public function valuesOf(component:MachineComponent):ComponentValues {
		if (!matches(component)) throw 'Component is not type "$id"';
		return component.values();
	}

	public function key(values:ComponentValues):String {
		var parts:Array<String> = [];
		for (input in inputs) {
			var value = values == null ? null : values.get(input.name);
			if (value == null) value = input.defaultValue;
			var text = switch value {
				case Number(number): switch input.type {
					case Length: Dimension.format(number);
					case Angle: Std.string(Math.round(number * 1e9) / 1e9);
					default: Std.string(number);
				};
				case Integer(value): Std.string(value);
				case Boolean(value): value ? "true" : "false";
				case Token(value): value;
				case null: throw 'Missing component input "${input.name}"';
			};
			parts.push('${input.name}:${text.length}:$text');
		}
		return id + "|" + parts.join("|");
	}

	public function partNumber(component:MachineComponent):String
		return keepDesignation ? component.designation : key(valuesOf(component));

	function parameter(name:String):Null<ComponentParameter> {
		for (input in inputs) if (input.name == name) return input;
		return null;
	}
}
