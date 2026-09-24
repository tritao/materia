package cadkit.parametric;

import cadkit.parametric.Placement;

/** Schema-independent, persistent value attached to an element or definition. */
class TypedProperty {
	public static inline var TypeBoolean:String = "boolean";
	public static inline var TypeInteger:String = "integer";
	public static inline var TypeText:String = "text";
	public static inline var TypeToken:String = "token";
	public static inline var TypeElementReference:String = "element-reference";
	public static inline var TypeDefinitionReference:String = "definition-reference";
	public static inline var TypeFeatureReference:String = "feature-reference";
	public static inline var TypePlacement:String = "placement";

	public final name:String;
	public final type:String;
	public final unit:Null<String>;
	public final tokenDomain:Null<String>;
	/** Extra JSON-compatible data is retained even when the property's schema is unknown. */
	public final metadata:Dynamic;
	public final value:Dynamic;

	public function new(name:String, type:String, value:Dynamic, ?unit:String, ?tokenDomain:String, ?metadata:Dynamic) {
		if (name == null || StringTools.trim(name) == "")
			throw new ParametricError("property name must not be empty");
		if (type == null || StringTools.trim(type) == "")
			throw new ParametricError("property type must not be empty");
		if (tokenDomain != null && StringTools.trim(tokenDomain) == "")
			throw new ParametricError("property token domain must not be empty");
		this.name = name;
		this.type = type;
		this.unit = unit;
		this.tokenDomain = tokenDomain;
		this.metadata = metadata;
		this.value = validateAndNormalize(type, value, unit, tokenDomain);
	}

	public static function boolean(name:String, value:Bool):TypedProperty
		return new TypedProperty(name, TypeBoolean, value);

	public static function integer(name:String, value:Int):TypedProperty
		return new TypedProperty(name, TypeInteger, value);

	public static function text(name:String, value:String):TypedProperty
		return new TypedProperty(name, TypeText, value);

	/** A token is a string whose allowed values are defined by an optional external domain schema. */
	public static function token(name:String, value:String, ?domain:String):TypedProperty
		return new TypedProperty(name, TypeToken, value, null, domain);

	public static function quantity(name:String, kind:String, value:Float, unit:String):TypedProperty {
		QuantityKind.validate(kind);
		if (kind == QuantityKind.Count)
			throw new ParametricError("count properties use the integer type");
		return new TypedProperty(name, kind, value, unit);
	}

	public static function elementReference(name:String, reference:ElementReference):TypedProperty
		return new TypedProperty(name, TypeElementReference, PersistentReference.element(reference));

	public static function definitionReference(name:String, reference:PersistentReference):TypedProperty
		return new TypedProperty(name, TypeDefinitionReference, reference);

	public static function featureReference(name:String, reference:PersistentReference):TypedProperty
		return new TypedProperty(name, TypeFeatureReference, reference);

	public static function placement(name:String, value:Placement):TypedProperty
		return new TypedProperty(name, TypePlacement, value);

	public function sameValue(other:Null<TypedProperty>):Bool {
		if (other == null || name != other.name || type != other.type || unit != other.unit || tokenDomain != other.tokenDomain
			|| !dynamicEquals(metadata, other.metadata))
			return false;
		if (Std.isOfType(value, PersistentReference))
			return cast(value, PersistentReference).equals(cast other.value);
		if (Std.isOfType(value, Placement))
			return placementEquals(cast value, cast other.value);
		if (value == null || other.value == null || Std.isOfType(value, String) || Std.isOfType(value, Bool) || Std.isOfType(value, Int)
			|| Std.isOfType(value, Float))
			return value == other.value;
		return haxe.Json.stringify(value) == haxe.Json.stringify(other.value);
	}

	private static function dynamicEquals(first:Dynamic, second:Dynamic):Bool {
		if (first == null || second == null || Std.isOfType(first, String) || Std.isOfType(first, Bool) || Std.isOfType(first, Int)
			|| Std.isOfType(first, Float))
			return first == second;
		return haxe.Json.stringify(first) == haxe.Json.stringify(second);
	}

	private static function placementEquals(first:Placement, second:Placement):Bool {
		if (second == null)
			return false;
		var a = first.location.plane;
		var b = second.location.plane;
		return vectorEquals(a.origin, b.origin) && vectorEquals(a.xDirection, b.xDirection) && vectorEquals(a.normal, b.normal);
	}

	private static function vectorEquals(first:cadkit.modeling.Vector, second:cadkit.modeling.Vector):Bool
		return first.x == second.x && first.y == second.y && first.z == second.z;

	private static function validateAndNormalize(type:String, value:Dynamic, unit:Null<String>, tokenDomain:Null<String>):Dynamic {
		return switch type {
			case TypeBoolean:
				if (!Std.isOfType(value, Bool)) throw new ParametricError("boolean property value must be a bool");
				value;
			case TypeInteger:
				if (!Std.isOfType(value, Int) || !Math.isFinite(cast value)) throw new ParametricError("integer property value must be an integer");
				value;
			case TypeText, TypeToken:
				if (!Std.isOfType(value, String)) throw new ParametricError("text property value must be a string");
				value;
			case QuantityKind.Scalar, QuantityKind.Length, QuantityKind.Angle, QuantityKind.Area, QuantityKind.Volume:
				if (!Std.isOfType(value, Int) && !Std.isOfType(value, Float))
					throw new ParametricError("quantity property value must be numeric");
				if (unit == null)
					throw new ParametricError("quantity property needs a unit");
				var numeric:Float = cast value;
				UnitConversion.validateUnit(type, unit);
				if (!Math.isFinite(numeric)) throw new ParametricError("quantity property value must be finite");
				numeric;
			case TypeElementReference:
				validateReference(value, PersistentReference.ElementTarget);
			case TypeDefinitionReference:
				validateReference(value, PersistentReference.DefinitionTarget);
			case TypeFeatureReference:
				validateReference(value, PersistentReference.FeatureTarget);
			case TypePlacement:
				if (!Std.isOfType(value, Placement)) throw new ParametricError("placement property value must be a placement");
				value;
			default:
				// Unknown types remain inspectable and round-trip without a registered domain schema.
				value;
		};
	}

	private static function validateReference(value:Dynamic, expectedType:String):Dynamic {
		if (!Std.isOfType(value, PersistentReference))
			throw new ParametricError("persistent reference property value must be a typed reference");
		var reference:PersistentReference = cast value;
		if (reference.targetType != expectedType)
			throw new ParametricError("persistent reference property has the wrong target type");
		return reference;
	}
}
