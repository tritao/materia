package cadkit.parametric;

/** A typed persistent reference. The document ID keeps cross-document references explicit. */
class PersistentReference {
	public static inline var ElementTarget:String = "element";
	public static inline var DefinitionTarget:String = "definition";
	public static inline var FeatureTarget:String = "feature";

	public final documentId:String;
	public final targetType:String;
	public final targetId:String;

	public function new(documentId:String, targetType:String, targetId:String) {
		if (documentId == null || StringTools.trim(documentId) == "")
			throw new ParametricError("persistent reference needs a document ID");
		if (targetType != ElementTarget && targetType != DefinitionTarget && targetType != FeatureTarget)
			throw new ParametricError("unsupported persistent reference target: " + targetType);
		if (targetId == null || StringTools.trim(targetId) == "")
			throw new ParametricError("persistent reference needs a target ID");
		if (targetType == FeatureTarget) {
			var featureId = Std.parseInt(targetId);
			if (featureId == null || featureId < 1 || Std.string(featureId) != targetId)
				throw new ParametricError("feature reference needs a positive canonical feature ID");
		}
		this.documentId = documentId;
		this.targetType = targetType;
		this.targetId = targetId;
	}

	public static function element(reference:ElementReference):PersistentReference
		return new PersistentReference(reference.documentId.value, ElementTarget, reference.elementId.value);

	public static function definition(document:Document, id:DefinitionId):PersistentReference
		return new PersistentReference(document.id.value, DefinitionTarget, id.value);

	public static function feature(document:Document, id:FeatureId):PersistentReference
		return new PersistentReference(document.id.value, FeatureTarget, Std.string(id.toInt()));

	public function equals(other:PersistentReference):Bool
		return other != null && documentId == other.documentId && targetType == other.targetType && targetId == other.targetId;
}
