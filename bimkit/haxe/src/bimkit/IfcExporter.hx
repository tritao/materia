package bimkit;

import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Definition;
import cadkit.parametric.Document;
import cadkit.parametric.DocumentId;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Feature;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.Placement;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.Relationship;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.UnitConversion;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.LevelBoxFeature;
import sys.io.File;

/** IFC4 STEP export for the BIM object, type, spatial, containment, and hosting subset. */
class IfcExporter {
	public static function encode(document:Document, ?timestamp:String):String
		return new Ifc4Document(document).encode(timestamp);

	public static function save(document:Document, path:String, ?timestamp:String):Void {
		if (path == null || StringTools.trim(path) == "")
			throw new BimError("IFC export path must not be empty");
		File.saveContent(path, encode(document, timestamp));
	}
}

private class Ifc4Document {
	static inline var LengthScale:Float = 0.001;
	static inline var AreaScale:Float = 0.000001;
	static inline var VolumeScale:Float = 0.000000001;
	static inline var OpeningOverlap:Float = 1.0;
	static inline var Alphabet:String = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz_$";

	final document:Document;
	final step:IfcStepWriter;
	final productIds:Map<String, Int>;
	final typeIds:Map<String, Int>;
	final classByElement:Map<String, String>;
	final psetOwners:Array<IfcPsetOwner>;
	final openingPlacements:Map<String, Int>;
	final fillingPlacements:Map<String, IfcFillingPlacement>;
	final productPlacements:Map<String, Int>;
	final usedGuids:Map<String, Bool>;
	var ownerHistory:Int;
	var representationContext:Int;

	public function new(document:Document) {
		if (document == null)
			throw new BimError("IFC export needs a CadKit document");
		this.document = document;
		step = new IfcStepWriter();
		productIds = new Map();
		typeIds = new Map();
		classByElement = new Map();
		psetOwners = [];
		openingPlacements = new Map();
		fillingPlacements = new Map();
		productPlacements = new Map();
		usedGuids = new Map();
		ownerHistory = 0;
		representationContext = 0;
	}

	public function encode(timestamp:Null<String>):String {
		var projects:Array<Element> = [];
		for (element in document.allElements())
			if (elementClass(element) == BimSchema.Project)
				projects.push(element);
		if (projects.length != 1)
			throw new BimError("IFC4 export requires exactly one BIM Project");

		createOwnerHistory();
		createProjectUnitsAndContext();
		createProject(projects[0]);
		createTypes();
		prepareHostedPlacements();
		createProducts();
		createRelationships();
		createPropertySets();
		var fileDate = timestamp == null ? "1970-01-01T00:00:00" : timestamp;
		if (!~/^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}$/.match(fileDate))
			throw new BimError("IFC timestamp must use YYYY-MM-DDTHH:MM:SS");
		return step.render(fileDate);
	}

	function createOwnerHistory():Void {
		var organization = step.add("IFCORGANIZATION", [
			"$", IfcStepWriter.string("CadKit"), IfcStepWriter.string("CadKit BIM export"), "$", "$"
		]);
		var person = step.add("IFCPERSON", ["$", "$", IfcStepWriter.string("CadKit"), "$", "$", "$", "$", "$"]);
		var user = step.add("IFCPERSONANDORGANIZATION", ["#" + person, "#" + organization, "$"]);
		var application = step.add("IFCAPPLICATION", [
			"#" + organization, IfcStepWriter.string("1"), IfcStepWriter.string("CadKit"), IfcStepWriter.string("CadKit")
		]);
		ownerHistory = step.add("IFCOWNERHISTORY", [
			"#" + user, "#" + application, "$", ".ADDED.", "$", "$", "$",
			Std.string(Std.int(Date.now().getTime() / 1000))
		]);
	}

	function createProjectUnitsAndContext():Void {
		var length = step.add("IFCSIUNIT", ["$", ".LENGTHUNIT.", "$", ".METRE."]);
		var area = step.add("IFCSIUNIT", ["$", ".AREAUNIT.", "$", ".SQUARE_METRE."]);
		var volume = step.add("IFCSIUNIT", ["$", ".VOLUMEUNIT.", "$", ".CUBIC_METRE."]);
		var angle = step.add("IFCSIUNIT", ["$", ".PLANEANGLEUNIT.", "$", ".RADIAN."]);
		var units = step.add("IFCUNITASSIGNMENT", ["(#" + length + ",#" + area + ",#" + volume + ",#" + angle + ")"]);
		var world = addAxisPlacement(Placement.identity().location.plane);
		representationContext = step.add("IFCGEOMETRICREPRESENTATIONCONTEXT", [
			"$", IfcStepWriter.string("Model"), "3", "1.E-5", "#" + world, "$"
		]);
		projectUnits = units;
	}

	var projectUnits:Int;

	function createProject(project:Element):Void {
		var properties = project.properties();
		var globalId = rootGlobalId("element:" + project.id.value, properties);
		var name = IfcStepWriter.string(project.name);
		var entity = step.add("IFCPROJECT", [
			IfcStepWriter.string(globalId), "#" + ownerHistory, name, "$", "$", name, "$",
			"(#" + representationContext + ")", "#" + projectUnits
		]);
		productIds.set(project.id.value, entity);
		classByElement.set(project.id.value, BimSchema.Project);
		addPsetOwner("element:" + project.id.value, entity, properties);
	}

	function createTypes():Void {
		var definitions = document.allDefinitions();
		definitions.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		for (definition in definitions) {
			var classification = definitionClass(definition);
			if (classification != BimSchema.WindowType && classification != BimSchema.DoorType)
				continue;
			var globalId = rootGlobalId("definition:" + definition.id.value, definition.properties());
			var propertySets = createPropertySetEntities("definition:" + definition.id.value, definition.properties(), false);
			var common = [
				IfcStepWriter.string(globalId), "#" + ownerHistory, IfcStepWriter.string(definition.name),
				"$", "$", propertySets.length == 0 ? "$" : idSet(propertySets), "$", "$", "$"
			];
			var entity:Int;
			if (classification == BimSchema.WindowType) {
				entity = step.add("IFCWINDOWTYPE", common.concat([
					".WINDOW.", ".NOTDEFINED.", ".T.", "$"
				]));
			} else {
				entity = step.add("IFCDOORTYPE", common.concat([
					".DOOR.", ".NOTDEFINED.", ".T.", "$"
				]));
			}
			typeIds.set(definition.id.value, entity);
		}
	}

	function createProducts():Void {
		var elements = document.allElements();
		elements.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		for (element in elements) {
			var classification = elementClass(element);
			if (classification == null || classification == BimSchema.Project)
				continue;
			var ifcType = elementEntityType(classification);
			if (ifcType == null)
				continue;
			var globalId = rootGlobalId("element:" + element.id.value, element.properties());
			var filling = fillingPlacements.get(element.id.value);
			var placement = filling == null ? productPlacements.get(element.id.value) : filling.placement;
			if (placement == null)
				throw new BimError("IFC product placement is missing: " + element.id.value);
			var dimensions = elementDimensions(element, classification);
			var representation = dimensions == null ? "$" : "#" + addBoxRepresentation(dimensions.width, dimensions.depth, dimensions.height);
			var args:Array<String> = [
				IfcStepWriter.string(globalId), "#" + ownerHistory, IfcStepWriter.string(element.name), "$", "$",
				"#" + placement, representation
			];
			switch classification {
				case BimSchema.Site:
					args = args.concat([IfcStepWriter.string(element.name), ".ELEMENT.", "$", "$", "$", "$", "$"]);
				case BimSchema.Building:
					args = args.concat([IfcStepWriter.string(element.name), ".ELEMENT.", "$", "$", "$"]);
				case BimSchema.Storey:
					var elevation = storeyElevation(element);
					args = args.concat([IfcStepWriter.string(element.name), ".ELEMENT.",
						elevation == null ? "$" : real(toMeters(cast elevation, "mm"))]);
				case BimSchema.Space:
					args = args.concat([IfcStepWriter.string(element.name), ".ELEMENT.", ".SPACE.", "$"]);
				case BimSchema.Wall:
					args = args.concat(["$", ".STANDARD."]);
				case BimSchema.Slab:
					args = args.concat(["$", ".FLOOR."]);
				case BimSchema.Window:
					args = args.concat(["$", realOrNull(instanceDimension(element, "height")),
						realOrNull(instanceDimension(element, "width")), ".WINDOW.", ".NOTDEFINED.", "$"]);
				case BimSchema.Door:
					args = args.concat(["$", realOrNull(instanceDimension(element, "height")),
						realOrNull(instanceDimension(element, "width")), ".DOOR.", ".NOTDEFINED.", "$"]);
				default:
					continue;
			}
			var entity = step.add(ifcType, args);
			productIds.set(element.id.value, entity);
			classByElement.set(element.id.value, classification);
			addPsetOwner("element:" + element.id.value, entity, element.properties());
		}
	}

	function createRelationships():Void {
		var aggregateChildren = new Map<String, Array<Int>>();
		var aggregateKeys = new Map<String, String>();
		var containedElements = new Map<String, Array<Int>>();
		var containmentKeys = new Map<String, String>();
		var spatialParents = new Map<String, String>();
		for (relationship in document.allRelationships()) {
			if (relationship.typeName != BimSchema.Aggregates && relationship.typeName != BimSchema.Contains)
				continue;
			validateLocalEndpoints(relationship);
			var sourceKey = relationship.source.elementId.value;
			var targetKey = relationship.target.elementId.value;
			var sourceClass = classByElement.get(sourceKey);
			var targetClass = classByElement.get(targetKey);
			if (sourceClass == null || targetClass == null)
				throw new BimError("IFC export found an unresolved BIM spatial relationship: " + relationship.id.value);
			if (spatialParents.exists(targetKey))
				throw new BimError("IFC export found multiple spatial parents for element: " + targetKey);
			spatialParents.set(targetKey, sourceKey);
			if (relationship.typeName == BimSchema.Aggregates || targetClass == BimSchema.Space) {
				var validAggregate = (sourceClass == BimSchema.Project && targetClass == BimSchema.Site)
					|| (sourceClass == BimSchema.Site && targetClass == BimSchema.Building)
					|| (sourceClass == BimSchema.Building && targetClass == BimSchema.Storey)
					|| (sourceClass == BimSchema.Storey && targetClass == BimSchema.Space);
				if (!validAggregate)
					throw new BimError("IFC export found an invalid spatial aggregate: " + sourceClass + " -> " + targetClass);
				var children = aggregateChildren.get(sourceKey);
				if (children == null) {
					children = [];
					aggregateChildren.set(sourceKey, children);
					aggregateKeys.set(sourceKey, "relationship:aggregates:" + sourceKey);
				}
				children.push(cast productIds.get(targetKey));
			} else {
				if (sourceClass != BimSchema.Storey || (targetClass != BimSchema.Wall && targetClass != BimSchema.Window
					&& targetClass != BimSchema.Door && targetClass != BimSchema.Slab))
					throw new BimError("IFC spatial containment requires a Storey and a physical BIM element: " + relationship.id.value);
				var children = containedElements.get(sourceKey);
				if (children == null) {
					children = [];
					containedElements.set(sourceKey, children);
					containmentKeys.set(sourceKey, "relationship:contains:" + sourceKey);
				}
				children.push(cast productIds.get(targetKey));
			}
		}
		for (sourceKey in sortedKeys(aggregateChildren)) {
			var related = aggregateChildren.get(sourceKey);
			related.sort(function(a, b) return a - b);
			step.add("IFCRELAGGREGATES", [
				IfcStepWriter.string(derivedGlobalId(aggregateKeys.get(sourceKey))), "#" + ownerHistory, "$", "$",
				"#" + productIds.get(sourceKey), idSet(related)
			]);
		}
		for (sourceKey in sortedKeys(containedElements)) {
			var related = containedElements.get(sourceKey);
			related.sort(function(a, b) return a - b);
			step.add("IFCRELCONTAINEDINSPATIALSTRUCTURE", [
				IfcStepWriter.string(derivedGlobalId(containmentKeys.get(sourceKey))), "#" + ownerHistory, "$", "$",
				idSet(related), "#" + productIds.get(sourceKey)
			]);
		}

		var relationships = document.allRelationships();
		relationships.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		for (relationship in relationships)
			if (relationship.typeName == BimSchema.Hosts)
				createOpeningRelations(relationship);

		for (element in document.allElements()) {
			var instance:InstanceElement = Std.isOfType(element, InstanceElement) ? cast element : null;
			if (instance == null || !productIds.exists(element.id.value))
				continue;
			var definition = document.definition(instance.definitionId);
			var classification = definitionClass(definition);
			if (classification != BimSchema.WindowType && classification != BimSchema.DoorType)
				throw new BimError("BIM opening instance does not reference a supported IFC type: " + element.id.value);
			var typeEntity = typeIds.get(definition.id.value);
			if (typeEntity == null)
				throw new BimError("IFC type is missing for BIM instance: " + element.id.value);
			step.add("IFCRELDEFINESBYTYPE", [
				IfcStepWriter.string(derivedGlobalId("relationship:type:" + element.id.value)), "#" + ownerHistory, "$", "$",
				"(#" + productIds.get(element.id.value) + ")", "#" + typeEntity
			]);
		}
	}

	function prepareHostedPlacements():Void {
		var relationships = document.allRelationships();
		relationships.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		var hostedOpenings = new Map<String, Bool>();
		for (relationship in relationships) {
			if (relationship.typeName != BimSchema.Hosts)
				continue;
			var opening = document.element(relationship.source.elementId);
			if (hostedOpenings.exists(opening.id.value))
				throw new BimError("IFC opening has more than one host: " + opening.id.value);
			hostedOpenings.set(opening.id.value, true);
		}
		for (element in document.allElements()) {
			var classification = elementClass(element);
			if (classification == null || classification == BimSchema.Project || elementEntityType(classification) == null
				|| hostedOpenings.exists(element.id.value))
				continue;
			productPlacements.set(element.id.value, addLocalPlacement(productPlacementPlane(element, classification)));
		}
		for (relationship in relationships) {
			if (relationship.typeName != BimSchema.Hosts)
				continue;
			validateLocalEndpoints(relationship);
			var opening = document.element(relationship.source.elementId);
			var wall = document.element(relationship.target.elementId);
			var openingClass = elementClass(opening);
			if ((openingClass != BimSchema.Window && openingClass != BimSchema.Door) || elementClass(wall) != BimSchema.Wall)
				throw new BimError("IFC host relationship must connect a Window or Door to a Wall: " + relationship.id.value);
			var wallPlacement = productPlacements.get(wall.id.value);
			if (wallPlacement == null)
				throw new BimError("IFC host wall has no product placement: " + wall.id.value);
			var localOpening = new Plane(new Vector(lengthProperty(relationship, "along"), -OpeningOverlap,
				lengthProperty(relationship, "sill")), Vector.X(), Vector.Z());
			var openingPlacement = addLocalPlacementRelative(wallPlacement, localOpening);
			openingPlacements.set(relationship.id.value, openingPlacement);
			var localFilling = new Plane(new Vector(0, OpeningOverlap, 0), Vector.X(), Vector.Z());
			var fillingPlacement = addLocalPlacementRelative(openingPlacement, localFilling);
			fillingPlacements.set(opening.id.value, new IfcFillingPlacement(fillingPlacement));
		}
	}

	function createOpeningRelations(relationship:Relationship):Void {
		validateLocalEndpoints(relationship);
		var openingKey = relationship.source.elementId.value;
		var wallKey = relationship.target.elementId.value;
		if (classByElement.get(openingKey) != BimSchema.Window && classByElement.get(openingKey) != BimSchema.Door)
			throw new BimError("IFC host source must be a Window or Door: " + relationship.id.value);
		if (classByElement.get(wallKey) != BimSchema.Wall)
			throw new BimError("IFC host target must be a Wall: " + relationship.id.value);
		var instance:InstanceElement = cast document.element(relationship.source.elementId);
		var wall = document.element(relationship.target.elementId);
		var along = lengthProperty(relationship, "along");
		var sill = lengthProperty(relationship, "sill");
		var openingPlacement = openingPlacements.get(relationship.id.value);
		if (openingPlacement == null)
			throw new BimError("IFC host opening placement is missing: " + relationship.id.value);
		var wallDimensions = elementDimensions(wall, BimSchema.Wall);
		var openingDepth = wallDimensions == null
			? instanceDimension(instance, "depth") + OpeningOverlap * 2
			: wallDimensions.depth + OpeningOverlap * 2;
		var width = instanceDimension(instance, "width");
		var height = instanceDimension(instance, "height");
		var shape = addBoxRepresentation(width, openingDepth, height);
		var openingGuid = derivedGlobalId("opening:" + relationship.id.value);
		var openingEntity = step.add("IFCOPENINGELEMENT", [
			IfcStepWriter.string(openingGuid), "#" + ownerHistory, IfcStepWriter.string(instance.name + " Opening"), "$", "$",
			"#" + openingPlacement, "#" + shape, "$", ".OPENING."
		]);
		psetOwners.push(new IfcPsetOwner("opening:" + relationship.id.value, openingEntity, [
			TypedProperty.quantity("along", QuantityKind.Length, along, "mm"),
			TypedProperty.quantity("sill", QuantityKind.Length, sill, "mm"),
			TypedProperty.token("output", tokenProperty(relationship, "output"), BimSchema.HostOutput)
		]));
		step.add("IFCRELVOIDSELEMENT", [
			IfcStepWriter.string(derivedGlobalId("relationship:voids:" + relationship.id.value)), "#" + ownerHistory,
			"$", "$", "#" + productIds.get(wallKey), "#" + openingEntity
		]);
		step.add("IFCRELFILLSELEMENT", [
			IfcStepWriter.string(derivedGlobalId("relationship:fills:" + relationship.id.value)), "#" + ownerHistory,
			"$", "$", "#" + openingEntity, "#" + productIds.get(openingKey)
		]);
	}

	function createPropertySets():Void {
		for (owner in psetOwners)
			createPropertySetEntities(owner.key, owner.properties, true, owner.entityId);
	}

	function createPropertySetEntities(ownerKey:String, properties:Array<TypedProperty>, relateByProperties:Bool,
		?ownerEntityId:Int):Array<Int> {
		var result:Array<Int> = [];
		var groups = groupProperties(properties);
		for (propertySetName in sortedKeys(groups)) {
			var values = groups.get(propertySetName);
			var propertyIds:Array<Int> = [];
			var usedNames = new Map<String, Bool>();
			for (property in values) {
				var propertyName = propertyOutputName(property);
				if (usedNames.exists(propertyName))
					propertyName = StringTools.replace(property.name, ".", "_");
				usedNames.set(propertyName, true);
				var nominal = propertyNominalValue(property);
				if (nominal == null)
					continue;
				propertyIds.push(step.add("IFCPROPERTYSINGLEVALUE", [
					IfcStepWriter.string(propertyName), "$", nominal, "$"
				]));
			}
			if (propertyIds.length == 0)
				continue;
			var pset = step.add("IFCPROPERTYSET", [
				IfcStepWriter.string(derivedGlobalId("pset:" + ownerKey + ":" + propertySetName)), "#" + ownerHistory,
				IfcStepWriter.string(propertySetName), "$", idSet(propertyIds)
			]);
			result.push(pset);
			if (relateByProperties) {
				if (ownerEntityId == null)
					throw new BimError("IFC occurrence property set has no owning entity");
				step.add("IFCRELDEFINESBYPROPERTIES", [
					IfcStepWriter.string(derivedGlobalId("relationship:pset:" + ownerKey + ":" + propertySetName)),
					"#" + ownerHistory, "$", "$", "(#" + ownerEntityId + ")", "#" + pset
				]);
			}
		}
		return result;
	}

	function addPsetOwner(key:String, entityId:Int, properties:Array<TypedProperty>):Void
		psetOwners.push(new IfcPsetOwner(key, entityId, properties));

	function groupProperties(properties:Array<TypedProperty>):Map<String, Array<TypedProperty>> {
		var result = new Map<String, Array<TypedProperty>>();
		for (property in properties) {
			if (isInternalProperty(property.name))
				continue;
			var propertySetName = propertySetFor(property);
			var values = result.get(propertySetName);
			if (values == null) {
				values = [];
				result.set(propertySetName, values);
			}
			values.push(property);
		}
		for (values in result)
			values.sort(function(a, b) return Reflect.compare(a.name, b.name));
		return result;
	}

	function propertySetFor(property:TypedProperty):String {
		var fromMetadata = property.metadata == null ? null : Reflect.field(property.metadata, "propertySet");
		if (Std.isOfType(fromMetadata, String) && StringTools.trim(cast fromMetadata) != "")
			return cast fromMetadata;
		if (StringTools.startsWith(property.name, "ifc.Pset_")) {
			var separator = property.name.indexOf(".", 4);
			if (separator >= 0)
				return property.name.substring(4, separator);
		}
		return "CadKitProperties";
	}

	function propertyOutputName(property:TypedProperty):String {
		var setName = propertySetFor(property);
		var metadataSet = property.metadata == null ? null : Reflect.field(property.metadata, "propertySet");
		if (metadataSet == setName)
			return afterLastDot(property.name);
		if (StringTools.startsWith(property.name, "ifc." + setName + "."))
			return afterLastDot(property.name);
		return property.name;
	}

	function propertyNominalValue(property:TypedProperty):Null<String> {
		var value:Dynamic = property.value;
		var metadata = property.metadata;
		var sourceType:Dynamic = metadata == null ? null : Reflect.field(metadata, "sourceType");
		if (value != null && Reflect.isObject(value) && Reflect.hasField(value, "wrappedValue"))
			value = Reflect.field(value, "wrappedValue");
		if (Std.isOfType(sourceType, String) && ~/^Ifc[A-Za-z0-9]+$/.match(cast sourceType))
			return cast(sourceType, String).toUpperCase() + "(" + rawStepValue(value) + ")";
		return switch property.type {
			case TypedProperty.TypeBoolean:
				"IFCBOOLEAN(" + (value == true ? ".T." : ".F.") + ")";
			case TypedProperty.TypeInteger:
				"IFCINTEGER(" + Std.string(value) + ")";
			case TypedProperty.TypeText:
				"IFCTEXT(" + IfcStepWriter.string(Std.string(value)) + ")";
			case TypedProperty.TypeToken:
				"IFCLABEL(" + IfcStepWriter.string(Std.string(value)) + ")";
			case QuantityKind.Scalar:
				"IFCREAL(" + real(cast value) + ")";
			case QuantityKind.Length:
				"IFCLENGTHMEASURE(" + real(toMeters(cast value, property.unit)) + ")";
			case QuantityKind.Angle:
				"IFCPLANEANGLEMEASURE(" + real(cast value) + ")";
			case QuantityKind.Area:
				"IFCAREAMEASURE(" + real(toSquareMeters(cast value, property.unit)) + ")";
			case QuantityKind.Volume:
				"IFCVOLUMEMEASURE(" + real(toCubicMeters(cast value, property.unit)) + ")";
			case TypedProperty.TypeElementReference, TypedProperty.TypeDefinitionReference, TypedProperty.TypeFeatureReference:
				var reference:PersistentReference = cast value;
				"IFCIDENTIFIER(" + IfcStepWriter.string(reference.targetType + ":" + reference.documentId + ":" + reference.targetId) + ")";
			case TypedProperty.TypePlacement:
				null;
			default:
				"IFCTEXT(" + IfcStepWriter.string(haxe.Json.stringify(value)) + ")";
		};
	}

	function rawStepValue(value:Dynamic):String {
		if (value == null)
			return "$";
		if (Std.isOfType(value, Bool))
			return value == true ? ".T." : ".F.";
		if (Std.isOfType(value, Int) || Std.isOfType(value, Float))
			return real(cast value);
		if (Std.isOfType(value, String))
			return IfcStepWriter.string(cast value);
		return IfcStepWriter.string(haxe.Json.stringify(value));
	}

	function isInternalProperty(name:String):Bool {
		return name == "bim.class" || name == "bim.authoredFeature" || name == "bim.element-class"
			|| name == "bim.definition-class" || name == BimSchema.GlobalId || name == BimSchema.BaseLevel
			|| name == BimSchema.TopLevel;
	}

	function elementClass(element:Element):Null<String> {
		var property = element.property("bim.class");
		if (property == null)
			return null;
		if (property.type != TypedProperty.TypeToken || property.tokenDomain != BimSchema.ElementClass)
			throw new BimError("BIM element class property is invalid: " + element.id.value);
		return cast property.value;
	}

	function definitionClass(definition:Definition):Null<String> {
		var property = definition.property("bim.class");
		if (property == null)
			return null;
		if (property.type != TypedProperty.TypeToken || property.tokenDomain != BimSchema.DefinitionClass)
			throw new BimError("BIM definition class property is invalid: " + definition.id.value);
		return cast property.value;
	}

	function elementEntityType(classification:String):Null<String> {
		return switch classification {
			case BimSchema.Site: "IFCSITE";
			case BimSchema.Building: "IFCBUILDING";
			case BimSchema.Storey: "IFCBUILDINGSTOREY";
			case BimSchema.Space: "IFCSPACE";
			case BimSchema.Wall: "IFCWALL";
			case BimSchema.Slab: "IFCSLAB";
			case BimSchema.Window: "IFCWINDOW";
			case BimSchema.Door: "IFCDOOR";
			default: null;
		};
	}

	function elementDimensions(element:Element, classification:String):Null<IfcBoxDimensions> {
		if (classification == BimSchema.Wall) {
			var property = element.property("bim.authoredFeature");
			if (property == null || property.type != TypedProperty.TypeFeatureReference)
				throw new BimError("IFC Wall is missing its authored feature reference: " + element.id.value);
			var reference:PersistentReference = cast property.value;
			if (reference.documentId != document.id.value || reference.targetType != PersistentReference.FeatureTarget)
				throw new BimError("IFC Wall has an invalid authored feature reference: " + element.id.value);
			var featureId = Std.parseInt(reference.targetId);
			var feature = featureId == null ? null : document.featureById(featureId);
			return feature == null ? null : featureDimensions(feature);
		}
		if (classification == BimSchema.Slab)
			return element.output == null ? null : featureDimensions(element.output);
		if (classification == BimSchema.Window || classification == BimSchema.Door)
			return new IfcBoxDimensions(instanceDimension(element, "width"), instanceDimension(element, "depth"),
				instanceDimension(element, "height"));
		return null;
	}

	function featureDimensions(feature:Feature):Null<IfcBoxDimensions> {
		if (Std.isOfType(feature, BoxFeature)) {
			var box:BoxFeature = cast feature;
			return new IfcBoxDimensions(box.width.value, box.depth.value, box.height.value);
		}
		if (Std.isOfType(feature, LevelBoxFeature)) {
			var box:LevelBoxFeature = cast feature;
			return new IfcBoxDimensions(box.width.value, box.depth.value, box.height(document));
		}
		return null;
	}

	function instanceDimension(element:Element, name:String):Float {
		if (!Std.isOfType(element, InstanceElement))
			throw new BimError("IFC " + name + " requires a typed BIM instance: " + element.id.value);
		var instance:InstanceElement = cast element;
		try {
			return instance.resolved(name);
		} catch (_:Dynamic) {
			throw new BimError("IFC opening type is missing its " + name + " input: " + element.name);
		}
	}

	function lengthProperty(relationship:Relationship, name:String):Float {
		var property = relationship.property(name);
		if (property == null || property.type != QuantityKind.Length || property.unit != "mm")
			throw new BimError("IFC host relationship has an invalid " + name + " property: " + relationship.id.value);
		return cast property.value;
	}

	function tokenProperty(relationship:Relationship, name:String):String {
		var property = relationship.property(name);
		if (property == null || property.type != TypedProperty.TypeToken)
			throw new BimError("IFC host relationship has an invalid " + name + " property: " + relationship.id.value);
		return cast property.value;
	}

	function validateLocalEndpoints(relationship:Relationship):Void {
		if (relationship.source.documentId.value != document.id.value || relationship.target.documentId.value != document.id.value)
			throw new BimError("IFC BIM relationships must stay within one document: " + relationship.id.value);
	}

	function addLocalPlacement(plane:Plane):Int {
		return step.add("IFCLOCALPLACEMENT", ["$", "#" + addAxisPlacement(plane)]);
	}

	function addLocalPlacementRelative(parent:Int, plane:Plane):Int {
		return step.add("IFCLOCALPLACEMENT", ["#" + parent, "#" + addAxisPlacement(plane)]);
	}

	function productPlacementPlane(element:Element, classification:String):Plane {
		var plane = document.worldPlacement(element).location.plane;
		if (classification != BimSchema.Storey)
			return plane;
		var elevation = storeyElevation(element);
		if (elevation == null)
			return plane;
		return new Plane(new Vector(plane.origin.x, plane.origin.y, elevation), plane.xDirection, plane.normal);
	}

	function storeyElevation(element:Element):Null<Float> {
		var property = element.property(BimSchema.BaseLevel);
		if (property == null)
			return null;
		if (property.type != TypedProperty.TypeElementReference)
			throw new BimError("IFC Storey base Level reference is invalid: " + element.id.value);
		var reference:PersistentReference = cast property.value;
		if (reference.targetType != PersistentReference.ElementTarget || reference.documentId != document.id.value)
			throw new BimError("IFC Storey base Level reference belongs to another document: " + element.id.value);
		return document.levelElevation(new ElementReference(new DocumentId(reference.documentId),
			new ElementId(reference.targetId)));
	}

	function addAxisPlacement(plane:Plane):Int {
		var point = step.add("IFCCARTESIANPOINT", ["(" + real(toMeters(plane.origin.x, "mm")) + ","
			+ real(toMeters(plane.origin.y, "mm")) + "," + real(toMeters(plane.origin.z, "mm")) + ")"]);
		var axis = addDirection(plane.normal);
		var reference = addDirection(plane.xDirection);
		return step.add("IFCAXIS2PLACEMENT3D", ["#" + point, "#" + axis, "#" + reference]);
	}

	function addDirection(vector:Vector):Int
		return step.add("IFCDIRECTION", ["(" + real(vector.x) + "," + real(vector.y) + "," + real(vector.z) + ")"]);

	function addBoxRepresentation(width:Float, depth:Float, height:Float):Int {
		if (!Math.isFinite(width) || !Math.isFinite(depth) || !Math.isFinite(height) || width <= 0 || depth <= 0 || height <= 0)
			throw new BimError("IFC swept box dimensions must be finite and positive");
		var origin = step.add("IFCCARTESIANPOINT", ["(0.,0.)"]);
		var profilePlacement = step.add("IFCAXIS2PLACEMENT2D", ["#" + origin, "$"]);
		var profile = step.add("IFCRECTANGLEPROFILEDEF", [
			".AREA.", "$", "#" + profilePlacement, real(toMeters(width, "mm")), real(toMeters(depth, "mm"))
		]);
		var up = step.add("IFCDIRECTION", ["(0.,0.,1.)"]);
		var solid = step.add("IFCEXTRUDEDAREASOLID", ["#" + profile, "$", "#" + up, real(toMeters(height, "mm"))]);
		var representation = step.add("IFCSHAPEREPRESENTATION", [
			"#" + representationContext, IfcStepWriter.string("Body"), IfcStepWriter.string("SweptSolid"), "(#" + solid + ")"
		]);
		return step.add("IFCPRODUCTDEFINITIONSHAPE", ["$", "$", "(#" + representation + ")"]);
	}

	function rootGlobalId(key:String, properties:Array<TypedProperty>):String {
		for (property in properties)
			if (property.name == BimSchema.GlobalId) {
				if ((property.type != TypedProperty.TypeText && property.type != TypedProperty.TypeToken)
					|| !validIfcGuid(Std.string(property.value)))
					throw new BimError("bim.globalId must be a valid 22-character IFC GUID");
				return registerGuid(Std.string(property.value));
			}
		return derivedGlobalId(key);
	}

	function derivedGlobalId(key:String):String
		return registerGuid(ifcGuid(document.id.value + "|" + key));

	function registerGuid(value:String):String {
		if (!validIfcGuid(value))
			throw new BimError("IFC GlobalId is not a valid compressed GUID: " + value);
		if (usedGuids.exists(value))
			throw new BimError("duplicate IFC GlobalId: " + value);
		usedGuids.set(value, true);
		return value;
	}

	static function ifcGuid(key:String):String {
		// Four independent 32-bit polynomial hashes provide 128 stable bits from
		// the persistent document and object identities without a target-specific crypto API.
		var hashes = [hashString(key, 5381, 33), hashString(key, 2166136261.0, 65599),
			hashString(key, 1013904223.0, 131), hashString(key, 2246822519.0, 65587)];
		var result = new StringBuf();
		for (character in 0...22) {
			var value = 0;
			for (bit in 0...6) {
				var sourceBit = character * 6 + bit - 4;
				value = value << 1;
				if (sourceBit >= 0 && sourceBit < 128) {
					var lane = Std.int(sourceBit / 32);
					var laneBit = sourceBit % 32;
					var bitValue = Math.floor(hashes[lane] / Math.pow(2, 31 - laneBit)) % 2;
					value = value | Std.int(bitValue);
				}
			}
			result.add(Alphabet.charAt(value));
		}
		return result.toString();
	}

	static function hashString(value:String, seed:Float, multiplier:Float):Float {
		var hash = seed;
		for (index in 0...value.length)
			hash = (hash * multiplier + value.charCodeAt(index) + index) % 4294967296.0;
		return hash;
	}

	static function validIfcGuid(value:String):Bool
		return value != null && ~/^[0-3][0-9A-Za-z_$]{21}$/.match(value);

	static function toMeters(value:Float, unit:Null<String>):Float {
		if (unit == null)
			throw new BimError("IFC length value is missing its unit");
		return UnitConversion.toCanonical(value, QuantityKind.Length, unit) * LengthScale;
	}

	static function toSquareMeters(value:Float, unit:Null<String>):Float {
		if (unit == null)
			throw new BimError("IFC area value is missing its unit");
		return UnitConversion.toCanonical(value, QuantityKind.Area, unit) * AreaScale;
	}

	static function toCubicMeters(value:Float, unit:Null<String>):Float {
		if (unit == null)
			throw new BimError("IFC volume value is missing its unit");
		return UnitConversion.toCanonical(value, QuantityKind.Volume, unit) * VolumeScale;
	}

	static function realOrNull(value:Float):String
		return real(toMeters(value, "mm"));

	static function real(value:Float):String {
		if (!Math.isFinite(value))
			throw new BimError("IFC numeric value must be finite");
		var result = Std.string(value);
		if (result.indexOf(".") < 0 && result.indexOf("e") < 0 && result.indexOf("E") < 0)
			result += ".";
		return StringTools.replace(StringTools.replace(result, "e", "E"), "e+", "E+");
	}

	static function idSet(values:Array<Int>):String {
		var unique:Array<String> = [];
		var seen = new Map<Int, Bool>();
		for (value in values)
			if (!seen.exists(value)) {
				seen.set(value, true);
				unique.push("#" + value);
			}
		return "(" + unique.join(",") + ")";
	}

	static function sortedKeys<T>(map:Map<String, T>):Array<String> {
		var keys = [for (key in map.keys()) key];
		keys.sort(Reflect.compare);
		return keys;
	}

	static function afterLastDot(value:String):String {
		var index = value.lastIndexOf(".");
		return index < 0 ? value : value.substr(index + 1);
	}

}

private class IfcStepWriter {
	final entities:Array<String>;
	var nextId:Int;

	public function new() {
		entities = [];
		nextId = 1;
	}

	public function add(type:String, arguments:Array<String>):Int {
		var id = nextId++;
		entities.push("#" + id + "=" + type + "(" + arguments.join(",") + ");");
		return id;
	}

	public function render(timestamp:String):String {
		var output = new StringBuf();
		output.add("ISO-10303-21;\nHEADER;\n");
		output.add("FILE_DESCRIPTION(('ViewDefinition [ReferenceView_V1.2]'),'2;1');\n");
		output.add("FILE_NAME('cadkit.ifc'," + string(timestamp) + ",('CadKit'),('CadKit'),'CadKit BIM exporter','CadKit','');\n");
		output.add("FILE_SCHEMA(('IFC4'));\nENDSEC;\nDATA;\n");
		for (entity in entities) {
			output.add(entity);
			output.add("\n");
		}
		output.add("ENDSEC;\nEND-ISO-10303-21;\n");
		return output.toString();
	}

	public static function string(value:String):String {
		var output = new StringBuf();
		output.add("'");
		var unicode = new StringBuf();
		function flushUnicode():Void {
			var contents = unicode.toString();
			if (contents.length > 0) {
				output.add("\\X2\\");
				output.add(contents);
				output.add("\\X0\\");
				unicode = new StringBuf();
			}
		}
		for (index in 0...value.length) {
			var code = value.charCodeAt(index);
			if (code >= 32 && code <= 126) {
				flushUnicode();
				var character = value.charAt(index);
				if (character == "'")
					output.add("''");
				else if (character == "\\")
					output.add("\\\\");
				else
					output.add(character);
			} else if (code == 10 || code == 13 || code == 9) {
				flushUnicode();
				output.add(code == 9 ? "\\X09\\" : code == 10 ? "\\X0A\\" : "\\X0D\\");
			} else {
				var hex = StringTools.hex(code, 4);
				unicode.add(hex.toUpperCase());
			}
		}
		flushUnicode();
		output.add("'");
		return output.toString();
	}
}

private class IfcBoxDimensions {
	public final width:Float;
	public final depth:Float;
	public final height:Float;

	public function new(width:Float, depth:Float, height:Float) {
		this.width = width;
		this.depth = depth;
		this.height = height;
	}
}

private class IfcPsetOwner {
	public final key:String;
	public final entityId:Int;
	public final properties:Array<TypedProperty>;

	public function new(key:String, entityId:Int, properties:Array<TypedProperty>) {
		this.key = key;
		this.entityId = entityId;
		this.properties = properties;
	}
}

private class IfcFillingPlacement {
	public final placement:Int;

	public function new(placement:Int) {
		this.placement = placement;
	}
}
