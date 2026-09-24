import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ElementReference;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.Relationship;
import cadkit.parametric.TypedProperty;

class RelationshipSmoke {
	static function check(value:Bool, message:String):Void {
		if (!value)
			throw message;
	}

	static function property(relationship:Relationship, name:String):TypedProperty {
		var result = relationship.property(name);
		if (result == null)
			throw "missing relationship property: " + name;
		return result;
	}

	public static function run():Void {
		var document = new Document();
		var wall = document.createObject("Wall 03");
		var window = document.createObject("Window 01");
		var relationship = document.createRelationship("bim.host", new ElementReference(document.id, window.id),
			new ElementReference(document.id, wall.id));
		relationship.setProperty(TypedProperty.quantity("along", QuantityKind.Length, 1800, "mm"));
		relationship.setProperty(TypedProperty.quantity("sill", QuantityKind.Length, 900, "mm"));
		relationship.setProperty(TypedProperty.token("output", "opening", "bim.host-output"));

		var transaction = document.beginTransaction();
		relationship.setProperty(TypedProperty.quantity("along", QuantityKind.Length, 1900, "mm"));
		transaction.commit();
		check(property(relationship, "along").value == 1900, "relationship property edit commits");
		check(document.undo() && property(relationship, "along").value == 1800, "relationship property edit undo");
		check(document.redo() && property(relationship, "along").value == 1900, "relationship property edit redo");

		var failed = false;
		try document.removeElement(wall.id) catch (error:Dynamic) failed = true;
		check(failed && document.relationshipCount() == 1, "endpoint deletion preserves the unresolved-reference guard");

		var text = DocumentCodec.encode(document);
		var reopened = DocumentCodec.decode(text, false, false);
		check(reopened.relationshipCount() == 1, "relationships save and reopen with the document");
		var loaded = reopened.allRelationships()[0];
		check(loaded.id.value == relationship.id.value && loaded.typeName == "bim.host"
			&& loaded.source.elementId.value == window.id.value && loaded.target.elementId.value == wall.id.value,
			"relationship identity, type, and endpoints round-trip");
		check(property(loaded, "sill").unit == "mm" && property(loaded, "output").value == "opening",
			"relationship properties round-trip");
		reopened.close();

		var clone = DocumentCodec.decode(text, true, false);
		var clonedRelationship = clone.allRelationships()[0];
		check(clonedRelationship.source.documentId.value == clone.id.value && clonedRelationship.target.documentId.value == clone.id.value,
			"cloning remaps relationship endpoints");
		clone.close();

		transaction = document.beginTransaction();
		relationship.remove();
		document.removeElement(wall.id);
		transaction.commit();
		check(document.relationshipCount() == 0 && document.findElement(wall.id) == null,
			"owning transaction can remove a relationship and endpoint together");
		check(document.undo() && document.relationshipCount() == 1 && document.findElement(wall.id) != null,
			"relationship and endpoint deletion undo restores identity");
		check(document.redo() && document.relationshipCount() == 0 && document.findElement(wall.id) == null,
			"relationship and endpoint deletion redo remains ordered");
		document.close();

		var hooks = new Document();
		var calls:Array<String> = [];
		var beforeOne = hooks.addBeforeRecomputeHook(function() calls.push("before-one"));
		hooks.addBeforeRecomputeHook(function() calls.push("before-two"));
		var afterOne = hooks.addAfterRecomputeHook(function() calls.push("after-one"));
		hooks.addAfterRecomputeHook(function() calls.push("after-two"));
		hooks.beforeRecompute = function() calls.push("legacy-before");
		hooks.afterRecompute = function() calls.push("legacy-after");
		hooks.recompute();
		check(calls.join(",") == "legacy-before,before-one,before-two,legacy-after,after-one,after-two",
			"legacy callbacks compose with ordered before and after hooks");
		hooks.removeBeforeRecomputeHook(beforeOne);
		hooks.removeAfterRecomputeHook(afterOne);
		calls.resize(0);
		hooks.recompute();
		check(calls.join(",") == "legacy-before,before-two,legacy-after,after-two", "hook handles remove one subscriber independently");
		hooks.close();
	}
}
