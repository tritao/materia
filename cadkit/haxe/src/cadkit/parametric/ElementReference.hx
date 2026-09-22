package cadkit.parametric;

import cadkit.parametric.Document;
import cadkit.parametric.DocumentId;
import cadkit.parametric.ElementId;

/** Reserved cross-document identity pair. External document loading is intentionally absent. */
class ElementReference {
	public static inline var Resolved:String = "resolved";
	public static inline var UnresolvedDocument:String = "unresolved-document";
	public static inline var UnresolvedElement:String = "unresolved-element";
	public static inline var IncompatibleKind:String = "incompatible-kind";

	public final documentId:DocumentId;
	public final elementId:ElementId;

	public function new(documentId:DocumentId, elementId:ElementId) {
		this.documentId = documentId;
		this.elementId = elementId;
	}

	public function state(document:Document, ?expectedKind:String):String {
		var result:String;
		if (document.id.value != documentId.value)
			result = "unresolved-document";
		else if (document.findElement(elementId) == null)
			result = "unresolved-element";
		else if (expectedKind != null && document.element(elementId).kind != expectedKind)
			result = "incompatible-kind";
		else
			result = "resolved";
		return result;
	}

}
