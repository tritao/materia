package bimkit;

import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ElementId;
import cadkit.parametric.DocumentId;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Placement;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import haxe.Json;

/** Persistence for BIM records keyed by persistent CadKit element IDs. */
class BimCodec {
	public static inline var FORMAT:String = "bimkit.document";
	public static inline var VERSION:Int = 2;

	public static function encode(model:BimDocument):String {
		var walls:Array<Dynamic> = [];
		for (wall in model.allWallRoles())
			walls.push({element: wall.elementId.value, uncutOutput: wall.uncutOutput.id.toInt()});
		var relationships:Array<Dynamic> = [];
		for (relationship in model.allRelationships())
			relationships.push({
				opening: relationship.openingId.value,
				wall: relationship.wallId.value,
				along: relationship.along,
				sill: relationship.sill,
				output: relationship.outputName,
				unhostPlacement: encodePlacement(relationship.unhostPlacement),
				unhostParent: relationship.unhostParent == null ? null : {
					document: relationship.unhostParent.documentId.value,
					element: relationship.unhostParent.elementId.value
				},
				unhostDepth: relationship.unhostDepth
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
		BimWindowDefinition.registerEvaluator();
		var cad:cadkit.parametric.Document;
		try {
			cad = DocumentCodec.decode(fieldString(root, "cadkit"), false, false);
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
				var rawParent:Dynamic = Reflect.field(record, "unhostParent");
				var rawDepth:Dynamic = Reflect.field(record, "unhostDepth");
				model.installRelationship(new HostRelationship(opening, wall, fieldNumber(record, "along"), fieldNumber(record, "sill"),
					fieldString(record, "output"), decodePlacement(required(record, "unhostPlacement")),
					rawParent == null ? null : new ElementReference(new DocumentId(fieldString(rawParent, "document")),
						new ElementId(fieldString(rawParent, "element"))),
					rawDepth == null ? null : fieldNumber(record, "unhostDepth")));
			}
			cad.recompute();
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

	private static function encodeVector(value:Vector):Dynamic
		return {x: value.x, y: value.y, z: value.z};

	private static function decodeVector(value:Dynamic):Vector
		return new Vector(fieldNumber(value, "x"), fieldNumber(value, "y"), fieldNumber(value, "z"));

	private static function encodePlacement(value:Placement):Dynamic {
		var plane = value.location.plane;
		return {origin: encodeVector(plane.origin), xDirection: encodeVector(plane.xDirection), normal: encodeVector(plane.normal)};
	}

	private static function decodePlacement(value:Dynamic):Placement
		return new Placement(new Plane(decodeVector(required(value, "origin")), decodeVector(required(value, "xDirection")),
			decodeVector(required(value, "normal"))));

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
