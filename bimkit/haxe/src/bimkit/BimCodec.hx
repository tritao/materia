package bimkit;

import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ElementId;
import haxe.Json;

/** Persistence for BIM records keyed by persistent CadKit element IDs. */
class BimCodec {
	public static inline var FORMAT:String = "bimkit.document";
	public static inline var VERSION:Int = 1;

	public static function encode(model:BimDocument):String {
		var walls:Array<Dynamic> = [];
		for (wall in model.allWallRoles())
			walls.push({element: wall.elementId.value, uncutOutput: wall.uncutOutput.id.toInt()});
		var relationships:Array<Dynamic> = [];
		for (wall in model.allWallRoles())
			for (relationship in model.openingsForWall(wall.elementId))
				relationships.push({
					opening: relationship.openingId.value,
					wall: relationship.wallId.value,
					along: relationship.along,
					sill: relationship.sill,
					output: relationship.outputName
				});
		return Json.stringify({
			format: FORMAT,
			version: VERSION,
			cadkit: DocumentCodec.encode(model.cad),
			walls: walls,
			relationships: relationships
		});
	}

	public static function decode(text:String):BimDocument {
		var root:Dynamic = Json.parse(text);
		if (fieldString(root, "format") != FORMAT || fieldInt(root, "version") != VERSION)
			throw new BimError("unsupported BimKit document format");
		var cad:cadkit.parametric.Document;
		try {
			cad = DocumentCodec.decode(fieldString(root, "cadkit"));
		} catch (error:Dynamic) {
			throw new BimError("CadKit decode failed: " + Std.string(Reflect.field(error, "message")));
		}
		var model = new BimDocument(cad);
		try {
			var walls:Array<Dynamic> = cast required(root, "walls");
			for (record in walls) {
				var elementId = new ElementId(fieldString(record, "element"));
				cad.element(elementId);
				var feature = cad.featureById(fieldInt(record, "uncutOutput"));
				if (feature == null)
					throw new BimError("unresolved wall output");
				model.installWall(elementId, feature);
			}
			var relationships:Array<Dynamic> = cast required(root, "relationships");
			for (record in relationships) {
				var opening = new ElementId(fieldString(record, "opening"));
				var wall = new ElementId(fieldString(record, "wall"));
				cad.element(opening);
				model.wallRole(wall);
				model.installRelationship(new HostRelationship(opening, wall, fieldNumber(record, "along"), fieldNumber(record, "sill"),
					fieldString(record, "output")));
			}
			return model;
		} catch (error:Dynamic) {
			model.close();
			throw error;
		}
	}

	private static function required(value:Dynamic, name:String):Dynamic {
		var result = Reflect.field(value, name);
		if (result == null)
			throw new BimError("missing BimKit field: " + name);
		return result;
	}

	private static function fieldString(value:Dynamic, name:String):String {
		var result:Dynamic = required(value, name);
		if (!Std.isOfType(result, String))
			throw new BimError("BimKit field is not a string: " + name);
		return cast result;
	}

	private static function fieldNumber(value:Dynamic, name:String):Float {
		var result:Dynamic = required(value, name);
		if (!Std.isOfType(result, Float) && !Std.isOfType(result, Int))
			throw new BimError("BimKit field is not numeric: " + name);
		var number:Float = cast result;
		if (!Math.isFinite(number))
			throw new BimError("BimKit field is not finite: " + name);
		return number;
	}

	private static function fieldInt(value:Dynamic, name:String):Int {
		var number = fieldNumber(value, name);
		if (number != Std.int(number))
			throw new BimError("BimKit field is not an integer: " + name);
		return Std.int(number);
	}
}
