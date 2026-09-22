import cadkit.parametric.Document;
import cadkit.parametric.features.BoxFeature;

class ElementSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value) throw message;
	}

	public static function run():Void {
		var document = new Document();
		var box = document.add(new BoxFeature(10, 20, 30));
		var first = document.createElement("First wall", box);
		var second = document.createElement("Second wall", box);
		check(document.elementCount() == 2, "element registry count");
		check(first.id.value != second.id.value, "new elements receive distinct identities");
		check(document.element(first.id) == first && first.output == box, "element lookup preserves output references");

		var otherDocument = new Document();
		var otherBox = otherDocument.add(new BoxFeature(1, 1, 1));
		var failed = false;
		try document.createElement("Foreign", otherBox) catch (error:Dynamic) failed = true;
		check(failed, "cross-document element outputs are rejected");
		otherDocument.close();
		document.close();
	}
}
