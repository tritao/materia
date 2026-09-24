package bimkit;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.DocumentId;
import cadkit.parametric.ElementId;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Placement;
import haxe.Json;

/** Compatibility boundary for old BimKit files; current files are CadKit documents. */
class BimCodec {
	public static inline var LEGACY_FORMAT:String = "bimkit.document";
	public static inline var LEGACY_VERSION:Int = 2;
	public static inline var FORMAT:String = LEGACY_FORMAT;
	public static inline var VERSION:Int = LEGACY_VERSION;

	/** Current BIM documents use the single CadKit document serializer. */
	public static function encode(model:BimDocument):String
		return DocumentCodec.encode(model.cad);

	/** Read a current CadKit document or import the legacy BIM wrapper format v2. */
	public static function decode(text:String):BimDocument {
		var root:Dynamic;
		try root = Json.parse(text) catch (error:Dynamic) throw new BimError("invalid BIM document JSON: " + errorText(error));
		var format = optionalString(root, "format");
		if (format == DocumentCodec.FORMAT)
			return decodeCurrent(text);
		if (format != LEGACY_FORMAT || fieldInt(root, "version") != LEGACY_VERSION)
			throw new BimError("unsupported BimKit document format");
		return importLegacyV2(root);
	}

	private static function decodeCurrent(text:String):BimDocument {
		var document:Null<Document> = null;
		var model:Null<BimDocument> = null;
		try {
			document = DocumentCodec.decode(text, false, false);
			model = new BimDocument(document);
			document.recompute();
			return model;
		} catch (error:Dynamic) {
			if (model != null)
				model.close();
			else if (document != null)
				document.close();
			throw new BimError("CadKit BIM document decode failed: " + errorText(error));
		}
	}

	private static function importLegacyV2(root:Dynamic):BimDocument {
		var document:Document;
		try {
			document = DocumentCodec.decode(fieldString(root, "cadkit"), false, false);
		} catch (error:Dynamic) {
			throw new BimError("legacy CadKit document decode failed: " + errorText(error));
		}
		var model = new BimDocument(document);
		try {
			var walls:Array<Dynamic> = cast required(root, "walls");
			for (record in walls) {
				var elementId = new ElementId(fieldString(record, "element"));
				var feature = document.featureById(fieldInt(record, "uncutOutput"));
				if (feature == null)
					throw new BimError("unresolved wall output");
				model.installWall(elementId, feature);
			}
			var relationships:Array<Dynamic> = cast required(root, "relationships");
			for (record in relationships) {
				var opening = new ElementId(fieldString(record, "opening"));
				var wall = new ElementId(fieldString(record, "wall"));
				var rawParent:Dynamic = Reflect.field(record, "unhostParent");
				var rawDepth:Dynamic = Reflect.field(record, "unhostDepth");
				model.installRelationship(new HostRelationship(opening, wall, fieldNumber(record, "along"), fieldNumber(record, "sill"),
					fieldString(record, "output"), decodePlacement(required(record, "unhostPlacement")),
					rawParent == null ? null : new ElementReference(new DocumentId(fieldString(rawParent, "document")),
						new ElementId(fieldString(rawParent, "element"))),
					rawDepth == null ? null : fieldNumber(record, "unhostDepth")));
			}
		document.clearHistory();
		document.recompute();
		return model;
		} catch (error:Dynamic) {
			model.close();
			if (Std.isOfType(error, BimError))
				throw error;
			throw new BimError("legacy BIM migration failed: " + errorText(error));
		}
	}

	private static function required(value:Dynamic, name:String):Dynamic {
		if (!Reflect.hasField(value, name))
			throw new BimError("missing BimKit field: " + name);
		return Reflect.field(value, name);
	}

	private static function optionalString(value:Dynamic, name:String):Null<String> {
		var result:Dynamic = Reflect.field(value, name);
		return Std.isOfType(result, String) ? cast result : null;
	}

	private static function decodeVector(value:Dynamic):Vector
		return new Vector(fieldNumber(value, "x"), fieldNumber(value, "y"), fieldNumber(value, "z"));

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

	private static function errorText(error:Dynamic):String {
		var detail:Dynamic = Reflect.field(error, "message");
		return detail == null ? Std.string(error) : Std.string(detail);
	}
}
