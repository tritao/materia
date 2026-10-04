package bimkit;

import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;

/** BIM documents share CadKit's current document schema. */
class BimCodec {
	public static inline var FORMAT:String = DocumentCodec.FORMAT;
	public static inline var VERSION:Int = DocumentCodec.VERSION;

	public static function encode(model:BimDocument):String return DocumentCodec.encode(model.cad);
	public static function decode(text:String):BimDocument {
		var root:Dynamic = haxe.Json.parse(text);
		var version:Dynamic = Reflect.field(root, "version");
		if (version != VERSION) throw new BimError('schema v$version is unsupported; expected v$VERSION');
		return decodeCurrent(text);
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

	private static function errorText(error:Dynamic):String {
		var detail:Dynamic = Reflect.field(error, "message");
		return detail == null ? Std.string(error) : Std.string(detail);
	}
}
