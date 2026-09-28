package machinekit.component;

import machinekit.component.ComponentValue.*;
import materia.project.MaterialLibrary;

/** Stable recipe for one kind of single part. */
class ComponentType {
	public final id:String;
	final inputs:Array<ComponentParameter>;
	final build:ComponentValues->MachineComponent;
	final keepDesignation:Bool;
	final materialInDesignation:Bool;
	final defaultMaterial:String;

	public function new(id:String, inputs:Array<ComponentParameter>, build:ComponentValues->MachineComponent,
			keepDesignation:Bool = false, materialInDesignation:Bool = false) {
		if (id == null || id.length == 0 || inputs == null || build == null)
			throw "Incomplete component type";
		this.id = id;
		this.keepDesignation = keepDesignation;
		this.materialInDesignation = materialInDesignation;
		var seen:Map<String, Bool> = [];
		for (input in inputs) {
			if (seen.exists(input.name)) throw 'Duplicate component input "${input.name}"';
			seen.set(input.name, true);
		}
		var baseline = new ComponentValues();
		for (input in inputs) baseline.set(input.name, input.defaultValue);
		var sourceBuild = build;
		var defaultComponent = sourceBuild(baseline);
		defaultMaterial = defaultComponent.materialSpec();
		this.inputs = inputs.copy();
		if (!seen.exists("material"))
			this.inputs.push(new ComponentParameter("material", Choice(MaterialLibrary.specs()), Token(defaultMaterial)));
		else {
			var declared = switch this.inputs[seenIndex(this.inputs, "material")].defaultValue {
				case Token(spec): spec;
				default: "";
			};
			if (declared != defaultMaterial) throw 'Component type "$id" material default differs from its generator';
		}
		this.build = values -> {
			var component = sourceBuild(values);
			component.setMaterial(values.token("material"));
			return component;
		};
	}

	public function parameters():Array<ComponentParameter> return inputs.copy();
	public function label():String {
		var words = id.split(".").pop().split("-").join(" ");
		return words.substr(0, 1).toUpperCase() + words.substr(1);
	}
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
			if (input.name == "material" && switch value {
				case Token(spec): spec == defaultMaterial;
				default: false;
			}) continue;
			var text = switch value {
				case Number(number): switch input.type {
					case Scalar: Std.string(Math.fround(number * 1e9) / 1e9);
					case Length: Dimension.format(number);
					case Angle: Std.string(Math.fround(number * 1e9) / 1e9);
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
		return keepDesignation && (component.materialSpec() == defaultMaterial || materialInDesignation)
			? component.designation : key(valuesOf(component));

	function parameter(name:String):Null<ComponentParameter> {
		for (input in inputs) if (input.name == name) return input;
		return null;
	}

	static function seenIndex(inputs:Array<ComponentParameter>, name:String):Int {
		for (i in 0...inputs.length) if (inputs[i].name == name) return i;
		return -1;
	}
}
