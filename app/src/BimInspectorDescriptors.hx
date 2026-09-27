package app;

import nativekit.ui.widgets.text.Text;


import cadkit.parametric.Definition;
import cadkit.parametric.Element;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.UnitConversion;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.properties.PropertyOption;

/** Adapts persistent CadKit values into the shared typed property inspector. */
class BimInspectorDescriptors {
	public static function forElement(element:Element, ?edit:BimProjectEdit):Array<PropertyDescriptor> {
		var result:Array<PropertyDescriptor> = [];
		result.push(readOnly("element:" + element.id.value + ":id", "Identity", "ID", element.id.value));
		result.push(readOnly("element:" + element.id.value + ":kind", "Identity", "Core kind", element.kind));
		result.push(nameDescriptor(element, edit));
		for (property in element.properties())
			result.push(elementProperty(element, property, edit));
		return result;
	}

	public static function forDefinition(definition:Definition, ?edit:BimProjectEdit):Array<PropertyDescriptor> {
		var result:Array<PropertyDescriptor> = [];
		result.push(readOnly("definition:" + definition.id.value + ":id", "Identity", "ID", definition.id.value));
		result.push(readOnly("definition:" + definition.id.value + ":recipe", "Identity", "Recipe", definition.recipe));
		for (input in definition.inputs())
			result.push(inputDescriptor(definition, input, edit));
		for (property in definition.properties())
			result.push(definitionProperty(definition, property, edit));
		return result;
	}

	/** The same controls edit resolved values on an individual definition instance. */
	public static function forInstanceInputs(instance:InstanceElement, ?edit:BimProjectEdit):Array<PropertyDescriptor> {
		var result:Array<PropertyDescriptor> = [];
		for (input in instance.document.definition(instance.definitionId).inputs())
			result.push(instanceInputDescriptor(instance, input, edit));
		return result;
	}

	private static function instanceInputDescriptor(instance:InstanceElement, input:DefinitionInput,
			?edit:BimProjectEdit):PropertyDescriptor {
		var options = inputOptions(input, "Instance Inputs");
		return new PropertyDescriptor("instance:" + instance.id.value + ":input:" + input.name,
			input.name, inputType(input), function(_) return switch input.kind {
				case TypedProperty.TypeBoolean: PropertyValue.Bool(instance.resolvedBoolean(input.name));
				case TypedProperty.TypeInteger: PropertyValue.Int(instance.resolvedInteger(input.name));
				case TypedProperty.TypeToken: PropertyValue.Enum(instance.resolvedToken(input.name));
				default: PropertyValue.Float(UnitConversion.fromCanonical(instance.resolved(input.name), input.kind, input.unit));
			}, function(_, value) setInstanceInput(edit, instance, input, value), options);
	}

	private static function inputOptions(input:DefinitionInput, category:String):PropertyDescriptorOptions {
		var options = new PropertyDescriptorOptions();
		options.category = category;
		options.unit = input.isNumeric() ? input.unit : null;
		options.recordHistory = false;
		if (input.allowedValues != null)
			for (allowed in input.allowedValues) options.options.push(new PropertyOption(allowed, allowed));
		return options;
	}

	private static function inputType(input:DefinitionInput):PropertyType return switch input.kind {
		case TypedProperty.TypeBoolean: PropertyType.Bool;
		case TypedProperty.TypeInteger: PropertyType.Int;
		case TypedProperty.TypeToken: PropertyType.Enum;
		default: PropertyType.Float;
	};

	private static function inputDescriptor(definition:Definition, input:DefinitionInput, ?edit:BimProjectEdit):PropertyDescriptor {
		var options = inputOptions(input, "Type Inputs");
		return new PropertyDescriptor("definition:" + definition.id.value + ":input:" + input.name, input.name, inputType(input),
			function(_) return switch input.kind {
				case TypedProperty.TypeBoolean: PropertyValue.Bool(cast definition.input(input.name).defaultValue);
				case TypedProperty.TypeInteger: PropertyValue.Int(cast definition.input(input.name).defaultValue);
				case TypedProperty.TypeToken: PropertyValue.Enum(cast definition.input(input.name).defaultValue);
				default: PropertyValue.Float(UnitConversion.fromCanonical(definition.input(input.name).defaultValue,
					input.kind, input.unit));
			}, function(_, value) setDefinitionInput(edit, definition, input, value), options);
	}

	private static function setInstanceInput(edit:Null<BimProjectEdit>, instance:InstanceElement,
			input:DefinitionInput, value:PropertyValue):Void {
		var before = instance.typedOverrideValue(input.name);
		apply(edit, "Set " + input.name, function() writeInstanceInput(instance, input, value), function() {
			if (before == null) instance.removeOverride(input.name);
			else if (input.isNumeric()) instance.setOverride(input.name,
				UnitConversion.fromCanonical(before, input.kind, input.unit), input.unit);
			else instance.setTypedOverride(input.name, before);
		});
	}

	private static function writeInstanceInput(instance:InstanceElement, input:DefinitionInput,
			value:PropertyValue):Void switch value {
		case PropertyValue.Bool(next): instance.setTypedOverride(input.name, next);
		case PropertyValue.Enum(next): instance.setTypedOverride(input.name, next);
		case PropertyValue.Int(next):
			if (input.kind == TypedProperty.TypeInteger) instance.setTypedOverride(input.name, next);
			else instance.setOverride(input.name, next, input.unit);
		case PropertyValue.Float(next): instance.setOverride(input.name, next, input.unit);
		default: throw "Definition input value has the wrong type";
	}

	private static function setDefinitionInput(edit:Null<BimProjectEdit>, definition:Definition,
			input:DefinitionInput, value:PropertyValue):Void {
		var before = definition.input(input.name).defaultValue;
		apply(edit, "Set " + input.name, function() writeDefinitionInput(definition, input, value), function() {
			if (input.isNumeric()) definition.setDefault(input.name,
				UnitConversion.fromCanonical(before, input.kind, input.unit), input.unit);
			else definition.setTypedDefault(input.name, before);
		});
	}

	private static function writeDefinitionInput(definition:Definition, input:DefinitionInput,
			value:PropertyValue):Void switch value {
		case PropertyValue.Bool(next): definition.setTypedDefault(input.name, next);
		case PropertyValue.Enum(next): definition.setTypedDefault(input.name, next);
		case PropertyValue.Int(next):
			if (input.kind == TypedProperty.TypeInteger) definition.setTypedDefault(input.name, next);
			else definition.setDefault(input.name, next, input.unit);
		case PropertyValue.Float(next): definition.setDefault(input.name, next, input.unit);
		default: throw "Definition input value has the wrong type";
	}

	private static function nameDescriptor(element:Element, ?edit:BimProjectEdit):PropertyDescriptor {
		var options = new PropertyDescriptorOptions();
		options.category = "Identity";
		options.recordHistory = false;
		return new PropertyDescriptor("element:" + element.id.value + ":name", "Name", PropertyType.Text,
			function(_) return PropertyValue.Text(element.name), function(_, value) switch (value) {
				case PropertyValue.Text(next):
					var before = element.name;
					apply(edit, "Rename " + before, function() element.rename(next), function() element.rename(before));
				default: throw "Element name requires text";
			}, options);
	}

	private static function elementProperty(element:Element, property:TypedProperty, ?edit:BimProjectEdit):PropertyDescriptor {
		return propertyDescriptor("element:" + element.id.value, property,
			function(next) {
				var before = element.property(property.name);
				apply(edit, "Set " + property.name, function() element.setProperty(next),
					function() element.setProperty(before));
			});
	}

	private static function definitionProperty(definition:Definition, property:TypedProperty, ?edit:BimProjectEdit):PropertyDescriptor {
		return propertyDescriptor("definition:" + definition.id.value, property,
			function(next) {
				var before = definition.property(property.name);
				apply(edit, "Set " + property.name, function() definition.setProperty(next),
					function() definition.setProperty(before));
			});
	}

	static function apply(edit:Null<BimProjectEdit>, label:String, change:Void->Void,
			undo:Void->Void):Void {
		if (edit == null) change(); else edit(label, change, undo);
	}

	private static function propertyDescriptor(prefix:String, schema:TypedProperty, write:TypedProperty->Void):PropertyDescriptor {
		var options = new PropertyDescriptorOptions();
		options.category = StringTools.startsWith(schema.name, "bim.") ? "BIM" : "Properties";
		options.unit = schema.unit;
		options.recordHistory = false;
		var editorType = propertyType(schema);
		if (editorType == null) {
			options.readOnly = true;
			return new PropertyDescriptor(prefix + ":property:" + schema.name, schema.name, PropertyType.Text,
				function(_) return PropertyValue.Text(display(schema)), function(_, _) {}, options);
		}
		return new PropertyDescriptor(prefix + ":property:" + schema.name, schema.name, editorType,
			function(_) return editableValue(schema), function(_, value) write(edited(schema, value)), options);
	}

	private static function propertyType(property:TypedProperty):Null<PropertyType> {
		return switch property.type {
			case TypedProperty.TypeBoolean: PropertyType.Bool;
			case TypedProperty.TypeInteger: PropertyType.Int;
			case TypedProperty.TypeText, TypedProperty.TypeToken: PropertyType.Text;
			case QuantityKind.Scalar, QuantityKind.Length, QuantityKind.Angle, QuantityKind.Area, QuantityKind.Volume: PropertyType.Float;
			default: null;
		};
	}

	private static function editableValue(property:TypedProperty):PropertyValue {
		return switch property.type {
			case TypedProperty.TypeBoolean: PropertyValue.Bool(cast property.value);
			case TypedProperty.TypeInteger: PropertyValue.Int(cast property.value);
			case TypedProperty.TypeText, TypedProperty.TypeToken: PropertyValue.Text(cast property.value);
			case QuantityKind.Scalar, QuantityKind.Length, QuantityKind.Angle, QuantityKind.Area, QuantityKind.Volume:
				PropertyValue.Float(cast property.value);
			default: PropertyValue.Unavailable;
		};
	}

	private static function edited(schema:TypedProperty, value:PropertyValue):TypedProperty {
		var next:Dynamic;
		switch schema.type {
			case TypedProperty.TypeBoolean:
				switch value {
					case PropertyValue.Bool(result): next = result;
					default: throw "Boolean property requires a boolean";
				}
			case TypedProperty.TypeInteger:
				switch value {
					case PropertyValue.Int(result): next = result;
					default: throw "Integer property requires an integer";
				}
			case TypedProperty.TypeText, TypedProperty.TypeToken:
				switch value {
					case PropertyValue.Text(result): next = result;
					default: throw "Text property requires text";
				}
			case QuantityKind.Scalar, QuantityKind.Length, QuantityKind.Angle, QuantityKind.Area, QuantityKind.Volume:
				switch value {
					case PropertyValue.Float(result): next = result;
					case PropertyValue.Int(result): next = result;
					default: throw "Quantity property requires a number";
				}
			default:
				throw "This persistent property type is read-only";
		}
		return new TypedProperty(schema.name, schema.type, next, schema.unit, schema.tokenDomain, schema.metadata);
	}

	private static function readOnly(id:String, category:String, label:String, value:String):PropertyDescriptor {
		var options = new PropertyDescriptorOptions();
		options.category = category;
		options.readOnly = true;
		options.recordHistory = false;
		return new PropertyDescriptor(id, label, PropertyType.Text, function(_) return PropertyValue.Text(value), function(_, _) {}, options);
	}

	private static function display(property:TypedProperty):String {
		if (property.type == TypedProperty.TypePlacement) {
			var value:cadkit.parametric.Placement = cast property.value;
			var origin = value.location.plane.origin;
			return "(" + origin.x + ", " + origin.y + ", " + origin.z + ")";
		}
		if (property.value == null || Std.isOfType(property.value, String) || Std.isOfType(property.value, Bool)
			|| Std.isOfType(property.value, Int) || Std.isOfType(property.value, Float))
			return Std.string(property.value);
		try {
			return haxe.Json.stringify(property.value);
		} catch (_:Dynamic) {
			return Std.string(property.value);
		}
	}
}
