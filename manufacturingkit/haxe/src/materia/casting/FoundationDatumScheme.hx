package materia.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.casting.InterfaceRequirement.InterfaceRequirements;

enum DatumReference {
	BodyFace(faceId:String);
	FeatureFace(featureId:String, faceId:String);
	InsertInterface(insertId:String, interfaceId:String);
	InsertHole(insertId:String, holeId:String);
}

typedef FoundationDatum = {
	var id:String;
	var reference:DatumReference;
	/** In foundation coordinates: +Z is the face normal or bore axis; +X is
	 * the reference direction of a line interface. The source declares this frame.
	 */
	var frame:AssemblyFrame;
}

typedef FoundationDatumScheme = {
	var id:String;
	var foundationId:String;
	var primary:FoundationDatum;
	var secondary:FoundationDatum;
	var tertiary:FoundationDatum;
}

/** Source-provided reference inventory, not a second geometry representation. */
typedef FoundationDatumSources = {
	var bodyFaces:Array<String>;
	var featureFaces:Map<String,Array<String>>;
	var insertInterfaces:Map<String,Array<String>>;
	var insertHoles:Map<String,Array<String>>;
	var unsupportedFeatures:Map<String,String>;
}

typedef FoundationDatumInsert = {
	var id:String;
	var pose:AssemblyFrame;
	var holes:Array<InsertHole>;
	var interfaces:Array<InterfaceRequirement>;
}

typedef UnresolvedFoundationDatum = {
	var datumId:String;
	var featureId:String;
	var reason:String;
}

class FoundationDatums {
	public static function snapshot(raw:Dynamic):FoundationDatumScheme {
		CastingFeatureCodec.fields(raw, ["id", "foundationId", "primary", "secondary", "tertiary"]);
		var id = CastingRecordCodec.string(raw, "id");
		var foundation = CastingRecordCodec.string(raw, "foundationId");
		CastingFeatureCodec.identifier(id);
		CastingFeatureCodec.identifier(foundation);
		var result:FoundationDatumScheme = {id: id, foundationId: foundation,
			primary: readDatum(Reflect.field(raw, "primary")), secondary: readDatum(Reflect.field(raw, "secondary")),
			tertiary: readDatum(Reflect.field(raw, "tertiary"))};
		var ids:Array<String> = [];
		for (datum in ordered(result)) {
			if (ids.indexOf(datum.id) >= 0) throw "Duplicate foundation datum: " + datum.id;
			ids.push(datum.id);
		}
		return result;
	}

	public static function record(scheme:FoundationDatumScheme):Dynamic {
		var s = snapshot(scheme);
		return {id: s.id, foundationId: s.foundationId,
			primary: datumRecord(s.primary), secondary: datumRecord(s.secondary), tertiary: datumRecord(s.tertiary)};
	}

	/** Datum order is semantic and never sorted alphabetically. */
	public static function ordered(s:FoundationDatumScheme):Array<FoundationDatum>
		return [s.primary, s.secondary, s.tertiary];

	/** Resolve IDs against the source's exact supported boundary records. This does
	 * not qualify the availability or accuracy of a physical datum contact patch.
	 * Non-prismatic feature boundaries stay explicitly unresolved, never boxified.
	 */
	public static function sourceInventory(envelope:Array<CastingPrism>, features:Array<CastingFeature>,
			inserts:Array<FoundationDatumInsert>):FoundationDatumSources {
		var result:FoundationDatumSources = {bodyFaces: [], featureFaces: [], insertInterfaces: [],
			insertHoles: [], unsupportedFeatures: []};
		for (face in CastingCavities.boundary(envelope))
			if (result.bodyFaces.indexOf(face.id) < 0) result.bodyFaces.push(face.id);
		for (cavity in CastingCavities.derive(envelope, features)) switch cavity.geometry {
			case Prisms(pieces):
				var ids:Array<String> = [];
				for (face in CastingCavities.boundary(pieces)) if (ids.indexOf(face.id) < 0) ids.push(face.id);
				ids.sort(Reflect.compare);
				result.featureFaces.set(cavity.featureId, ids);
			case SolidIntersection(_, _, _):
				result.unsupportedFeatures.set(cavity.featureId, "Exact non-prismatic feature-face references require native resolution");
			case UnsupportedGeometry(reason): result.unsupportedFeatures.set(cavity.featureId, reason);
		}
		for (insert in inserts) {
			CastingFeatureCodec.identifier(insert.id);
			CastingRecordCodec.frame(insert.pose);
			if (result.insertHoles.exists(insert.id)) throw "Duplicate datum source insert: " + insert.id;
			result.insertHoles.set(insert.id, [for (hole in InsertHole.InsertHoleCodec.snapshot(insert.holes)) hole.id]);
			result.insertInterfaces.set(insert.id, [for (requirement in InterfaceRequirements.snapshot(insert.interfaces)) requirement.id]);
		}
		result.bodyFaces.sort(Reflect.compare);
		return result;
	}

	public static function validateReferences(scheme:FoundationDatumScheme, sources:FoundationDatumSources,
			requirements:Array<InterfaceRequirement>):Array<UnresolvedFoundationDatum> {
		var s = snapshot(scheme);
		var unresolved:Array<UnresolvedFoundationDatum> = [];
		for (d in ordered(s)) {
			var found = switch d.reference {
				case BodyFace(face): sources.bodyFaces.indexOf(face) >= 0;
				case FeatureFace(feature, face):
					if (sources.unsupportedFeatures.exists(feature)) {
						unresolved.push({datumId: d.id, featureId: feature, reason: sources.unsupportedFeatures.get(feature)});
						true;
					} else contains(sources.featureFaces, feature, face);
				case InsertInterface(insert, face): contains(sources.insertInterfaces, insert, face);
				case InsertHole(insert, hole): contains(sources.insertHoles, insert, hole);
			};
			if (!found) throw "Unknown foundation datum reference: " + d.id;
		}
		validateRequirementReferences(s, requirements);
		return unresolved;
	}

	public static function validateSource(scheme:FoundationDatumScheme, envelope:Array<CastingPrism>,
			features:Array<CastingFeature>, inserts:Array<FoundationDatumInsert>,
			interfaces:Array<InterfaceRequirement>):Array<UnresolvedFoundationDatum> {
		var inventory = sourceInventory(envelope, features, inserts);
		var result = validateReferences(scheme, inventory, interfaces);
		// Interface IDs belong to their owning insert, not one flattened ID space.
		for (insert in inserts) validateRequirementReferences(scheme, insert.interfaces);
		FoundationDatumGeometry.validate(scheme,envelope,features,inserts);
		return result;
	}

	static function validateRequirementReferences(scheme:FoundationDatumScheme, requirements:Array<InterfaceRequirement>):Void {
		var ids = [for (datum in ordered(scheme)) datum.id];
		for (requirement in InterfaceRequirements.snapshot(requirements))
			if (ids.indexOf(requirement.datumReference) < 0)
				throw "Interface refers to an undeclared foundation datum: " + requirement.id;
	}

	static function contains(inventory:Map<String,Array<String>>, owner:String, feature:String):Bool {
		var values = inventory.get(owner);
		return values != null && values.indexOf(feature) >= 0;
	}

	static function readDatum(raw:Dynamic):FoundationDatum {
		CastingFeatureCodec.fields(raw, ["id", "reference", "frame"]);
		var id = CastingRecordCodec.string(raw, "id");
		CastingFeatureCodec.identifier(id);
		return {id: id, reference: readReference(Reflect.field(raw, "reference")),
			frame: CastingRecordCodec.frame(Reflect.field(raw, "frame"))};
	}

	static function datumRecord(d:FoundationDatum):Dynamic
		return {id: d.id, reference: referenceRecord(d.reference), frame: CastingRecordCodec.frame(d.frame)};

	static function readReference(raw:Dynamic):DatumReference {
		if (Std.isOfType(raw, DatumReference)) raw = referenceRecord(cast raw);
		var result = switch CastingRecordCodec.string(raw, "kind") {
			case "body-face":
				CastingFeatureCodec.fields(raw, ["kind", "faceId"]);
				BodyFace(CastingRecordCodec.string(raw, "faceId"));
			case "feature-face":
				CastingFeatureCodec.fields(raw, ["kind", "featureId", "faceId"]);
				FeatureFace(CastingRecordCodec.string(raw, "featureId"), CastingRecordCodec.string(raw, "faceId"));
			case "insert-interface":
				CastingFeatureCodec.fields(raw, ["kind", "insertId", "interfaceId"]);
				InsertInterface(CastingRecordCodec.string(raw, "insertId"), CastingRecordCodec.string(raw, "interfaceId"));
			case "insert-hole":
				CastingFeatureCodec.fields(raw, ["kind", "insertId", "holeId"]);
				InsertHole(CastingRecordCodec.string(raw, "insertId"), CastingRecordCodec.string(raw, "holeId"));
			default: throw "Unknown foundation datum reference kind";
		};
		referenceRecord(result);
		return result;
	}

	public static function referenceRecord(reference:DatumReference):Dynamic return switch reference {
		case BodyFace(face):
			CastingFeatureCodec.identifier(face);
			{kind: "body-face", faceId: face};
		case FeatureFace(feature, face):
			for (id in [feature, face]) CastingFeatureCodec.identifier(id);
			{kind: "feature-face", featureId: feature, faceId: face};
		case InsertInterface(insert, face):
			for (id in [insert, face]) CastingFeatureCodec.identifier(id);
			{kind: "insert-interface", insertId: insert, interfaceId: face};
		case InsertHole(insert, hole):
			for (id in [insert, hole]) CastingFeatureCodec.identifier(id);
			{kind: "insert-hole", insertId: insert, holeId: hole};
	};
}
