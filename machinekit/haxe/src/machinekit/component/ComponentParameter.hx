package machinekit.component;

import machinekit.component.ComponentParameterType.*;
import machinekit.component.ComponentValue.*;

/** Description and validation of one editable generator input. */
class ComponentParameter {
	public final name:String;
	public final type:ComponentParameterType;
	public final defaultValue:ComponentValue;
	public final unit:Null<String>;
	public final minimum:Null<Float>;
	public final maximum:Null<Float>;

	public function new(name:String, type:ComponentParameterType, defaultValue:ComponentValue,
			?unit:String, ?minimum:Float, ?maximum:Float) {
		if (name == null || name.length == 0) throw "Component parameter needs a name";
		this.name = name;
		this.type = type;
		this.defaultValue = defaultValue;
		this.unit = unit;
		this.minimum = minimum;
		this.maximum = maximum;
		validate(defaultValue);
	}

	public function validate(value:ComponentValue):Void {
		if (value == null) throw 'Missing component parameter "$name"';
		switch type {
			case Scalar | Length | Angle:
				switch value {
					case Number(v): validateNumber(v);
					default: wrongType();
				}
			case Count:
				switch value {
					case Integer(v): validateNumber(v);
					default: wrongType();
				}
			case Bool:
				switch value {
					case Boolean(_):
					default: wrongType();
				}
			case Text:
				switch value {
					case Token(v): if (v == null) wrongType();
					default: wrongType();
				}
			case Choice(options):
				switch value {
					case Token(v): if (options.indexOf(v) < 0) throw 'Invalid choice "$v" for component parameter "$name"';
					default: wrongType();
				}
			case CatalogDesignation(index):
				switch value {
					case Token(v): if (v == null || index.designations().indexOf(v) < 0) throw 'Unknown catalog designation "$v" for component parameter "$name"';
					default: wrongType();
				}
		}
	}

	function validateNumber(value:Float):Void {
		if (!Math.isFinite(value)) throw 'Non-finite component parameter "$name"';
		if (minimum != null && value < minimum) throw 'Component parameter "$name" is below its minimum';
		if (maximum != null && value > maximum) throw 'Component parameter "$name" is above its maximum';
	}

	function wrongType():Void throw 'Wrong value type for component parameter "$name"';
}
