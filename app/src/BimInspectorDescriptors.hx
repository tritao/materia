package app;

import nativekit.ui.widgets.text.Text;


import cadkit.parametric.Definition;
import cadkit.parametric.Element;
import cadkit.parametric.DefinitionInput;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.UnitConversion;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;

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

	private static function inputDescriptor(definition:Definition, input:DefinitionInput, ?edit:BimProjectEdit):PropertyDescriptor {
		var options = new PropertyDescriptorOptions();
		options.category = "Type Inputs";
		options.unit = input.unit;
		options.recordHistory = false;
		return new PropertyDescriptor("definition:" + definition.id.value + ":input:" + input.name, input.name, PropertyType.Float,
			function(_) return PropertyValue.Float(UnitConversion.fromCanonical(definition.input(input.name).defaultValue,
				input.kind, input.unit)), function(_, value) switch (value) {
				case PropertyValue.Float(next): apply(edit, "Set " + input.name, function() definition.setDefault(input.name, next, input.unit));
				case PropertyValue.Int(next): apply(edit, "Set " + input.name, function() definition.setDefault(input.name, next, input.unit));
				default: throw "Definition input requires a number";
			}, options);
	}

	private static function nameDescriptor(element:Element, ?edit:BimProjectEdit):PropertyDescriptor {
		var options = new PropertyDescriptorOptions();
		options.category = "Identity";
		options.recordHistory = false;
		return new PropertyDescriptor("element:" + element.id.value + ":name", "Name", PropertyType.Text,
			function(_) return PropertyValue.Text(element.name), function(_, value) switch (value) {
				case PropertyValue.Text(next): apply(edit, "Rename " + element.name, function() element.rename(next));
				default: throw "Element name requires text";
			}, options);
	}

	private static function elementProperty(element:Element, property:TypedProperty, ?edit:BimProjectEdit):PropertyDescriptor {
		return propertyDescriptor("element:" + element.id.value, property,
			function(next) apply(edit, "Set " + property.name, function() element.setProperty(next)));
	}

	private static function definitionProperty(definition:Definition, property:TypedProperty, ?edit:BimProjectEdit):PropertyDescriptor {
		return propertyDescriptor("definition:" + definition.id.value, property,
			function(next) apply(edit, "Set " + property.name, function() definition.setProperty(next)));
	}

	static function apply(edit:Null<BimProjectEdit>, label:String, change:Void->Void):Void {
		if (edit == null) change(); else edit(label, change);
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
