package bimkit;

import bimkit.BimWindowDefinition;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.ElementKind;
import cadkit.parametric.Definition;
import cadkit.parametric.DocumentId;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Feature;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.PersistentReference;
import cadkit.parametric.Placement;
import cadkit.parametric.PlacementChange;
import cadkit.parametric.QuantityKind;
import cadkit.parametric.Relationship;
import cadkit.parametric.RelationshipId;
import cadkit.parametric.TypedProperty;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.LevelBoxFeature;
import cadkit.parametric.features.DefinitionOutputFeature;
import cadkit.parametric.features.ToolCollectionFeature;

/** BIM roles and hosting relationships layered over an ordinary CadKit document. */
class BimDocument {
	public final cad:Document;

	private var beforeHookId:Int;
	private var afterHookId:Int;

	public function new(?cad:Document) {
		BimWindowDefinition.registerEvaluator();
		this.cad = cad == null ? new Document(null, false) : cad;
		this.cad.setImplicitOutputEnabled(false);
		beforeHookId = this.cad.addBeforeRecomputeHook(validateAllOpenings);
		afterHookId = this.cad.addAfterRecomputeHook(synchronizeLevelPlacements);
		synchronizeHostPlacementAuthority();
	}

	public function createWindowDefinition(name:String, width:Float, height:Float, frameThickness:Float, depth:Float):Definition
		return BimWindowDefinition.create(cad, name, width, height, frameThickness, depth);

	public function createWindow(name:String, definition:Definition):InstanceElement {
		if (definition.document != cad)
			throw new BimError("window definition belongs to another document");
		var transaction = cad.beginTransaction();
		try {
			var result = cad.createInstance(name, definition);
			result.setProperty(TypedProperty.token("bim.class", BimSchema.Window, BimSchema.ElementClass));
			result.setProperty(TypedProperty.definitionReference("bim.type", PersistentReference.definition(cad, definition.id)));
			transaction.commit();
			return result;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	/** Create a BIM project as a generic persistent CadKit object. */
	public function createProject(name:String):Element
		return createClassifiedObject(name, BimSchema.Project);

	public function createSite(name:String, projectId:ElementId):Element
		return createAggregatedObject(name, BimSchema.Site, projectId, BimSchema.Project);

	public function createBuilding(name:String, siteId:ElementId):Element
		return createAggregatedObject(name, BimSchema.Building, siteId, BimSchema.Site);

	public function createStorey(name:String, buildingId:ElementId, ?baseLevel:ElementReference,
		?topLevel:ElementReference):Element {
		validateStoreyLevels(baseLevel, topLevel);
		return createAggregatedObject(name, BimSchema.Storey, buildingId, BimSchema.Building, function(storey) {
			if (baseLevel != null)
				storey.setProperty(TypedProperty.elementReference(BimSchema.BaseLevel, baseLevel));
			if (topLevel != null)
				storey.setProperty(TypedProperty.elementReference(BimSchema.TopLevel, topLevel));
		});
	}

	public function createSpace(name:String, storeyId:ElementId):Element
		return createContainedObject(name, BimSchema.Space, storeyId);

	/** Remove an empty spatial container and detach its one incoming aggregate edge. */
	public function removeSpatialContainer(elementId:ElementId):Void {
		var element = cad.element(elementId);
		var classification = bimClass(element);
		if (classification != BimSchema.Project && classification != BimSchema.Site && classification != BimSchema.Building
			&& classification != BimSchema.Storey)
			throw new BimError("element is not a BIM spatial container: " + elementId.value);
		for (relationship in cad.relationshipsForElement(elementId))
			if ((relationship.typeName == BimSchema.Aggregates || relationship.typeName == BimSchema.Contains)
				&& relationship.source.documentId.value == cad.id.value && relationship.source.elementId.value == elementId.value)
				throw new BimError("spatial container still has children: " + elementId.value);
		var transaction = cad.beginTransaction();
		try {
			removeIncomingSpatialEdges(elementId);
			cad.removeElement(elementId);
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function removeSpace(elementId:ElementId):Void {
		if (bimClass(cad.element(elementId)) != BimSchema.Space)
			throw new BimError("element is not a Space: " + elementId.value);
		var transaction = cad.beginTransaction();
		try {
			removeContainmentFor(elementId);
			cad.removeElement(elementId);
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	private function createClassifiedObject(name:String, classification:String):Element {
		var transaction = cad.beginTransaction();
		try {
			var result = cad.createObject(name);
			result.setProperty(TypedProperty.token("bim.class", classification, BimSchema.ElementClass));
			transaction.commit();
			return result;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	private function createAggregatedObject(name:String, classification:String, parentId:ElementId, parentClass:String,
		?configure:Element->Void):Element {
		var parent = cad.element(parentId);
		if (bimClass(parent) != parentClass)
			throw new BimError("expected " + parentClass + " aggregate parent: " + parentId.value);
		var transaction = cad.beginTransaction();
		try {
			var result = cad.createObject(name);
			result.setProperty(TypedProperty.token("bim.class", classification, BimSchema.ElementClass));
			if (configure != null)
				configure(result);
			cad.createRelationship(BimSchema.Aggregates, new ElementReference(cad.id, parentId), new ElementReference(cad.id, result.id));
			transaction.commit();
			return result;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	private function createContainedObject(name:String, classification:String, storeyId:ElementId):Element {
		var storey = cad.element(storeyId);
		if (bimClass(storey) != BimSchema.Storey)
			throw new BimError("expected a Storey container: " + storeyId.value);
		var transaction = cad.beginTransaction();
		try {
			var result = cad.createObject(name);
			result.setProperty(TypedProperty.token("bim.class", classification, BimSchema.ElementClass));
			cad.createRelationship(BimSchema.Contains, new ElementReference(cad.id, storeyId), new ElementReference(cad.id, result.id));
			transaction.commit();
			return result;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	/** Add an existing BIM object directly to a Storey's spatial contents. */
	public function addToStorey(storeyId:ElementId, elementId:ElementId):Relationship {
		var storey = cad.element(storeyId);
		var child = cad.element(elementId);
		if (bimClass(storey) != BimSchema.Storey || !isContainable(bimClass(child)))
			throw new BimError("bim.contains requires a Storey and a spatial BIM element");
		if (storeyId.value == elementId.value)
			throw new BimError("a Storey cannot contain itself");
		ensureNoSpatialParent(elementId, BimSchema.Contains);
		var transaction = cad.beginTransaction();
		try {
			var result = cad.createRelationship(BimSchema.Contains, new ElementReference(cad.id, storeyId), new ElementReference(cad.id, elementId));
			transaction.commit();
			return result;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function removeFromStorey(storeyId:ElementId, elementId:ElementId):Void {
		var found:Null<Relationship> = null;
		for (relationship in cad.relationshipsForElement(elementId))
			if (relationship.typeName == BimSchema.Contains && relationship.source.documentId.value == cad.id.value
				&& relationship.source.elementId.value == storeyId.value && relationship.target.documentId.value == cad.id.value
				&& relationship.target.elementId.value == elementId.value) {
				found = relationship;
				break;
			}
		if (found == null)
			throw new BimError("element is not contained by Storey " + storeyId.value + ": " + elementId.value);
		var transaction = cad.beginTransaction();
		try {
			cad.removeRelationship(found.id);
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	/** Add an existing child to the spatial aggregate hierarchy. */
	public function aggregate(parentId:ElementId, childId:ElementId):Relationship {
		var parent = cad.element(parentId);
		var child = cad.element(childId);
		if (!canAggregate(bimClass(parent), bimClass(child)))
			throw new BimError("invalid BIM spatial aggregation: " + bimClass(parent) + " -> " + bimClass(child));
		ensureNoSpatialParent(childId, BimSchema.Aggregates);
		var transaction = cad.beginTransaction();
		try {
			var result = cad.createRelationship(BimSchema.Aggregates, new ElementReference(cad.id, parentId), new ElementReference(cad.id, childId));
			transaction.commit();
			return result;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function setStoreyLevels(storeyId:ElementId, ?baseLevel:ElementReference, ?topLevel:ElementReference):Void {
		var storey = cad.element(storeyId);
		if (bimClass(storey) != BimSchema.Storey)
			throw new BimError("element is not a Storey: " + storeyId.value);
		validateStoreyLevels(baseLevel, topLevel);
		var transaction = cad.beginTransaction();
		try {
			if (baseLevel == null)
				storey.removeProperty(BimSchema.BaseLevel);
			else
				storey.setProperty(TypedProperty.elementReference(BimSchema.BaseLevel, baseLevel));
			if (topLevel == null)
				storey.removeProperty(BimSchema.TopLevel);
			else
				storey.setProperty(TypedProperty.elementReference(BimSchema.TopLevel, topLevel));
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function storeyBaseLevel(storeyId:ElementId):Null<ElementReference>
		return storeyLevel(storeyId, BimSchema.BaseLevel);

	public function storeyTopLevel(storeyId:ElementId):Null<ElementReference>
		return storeyLevel(storeyId, BimSchema.TopLevel);

	private function storeyLevel(storeyId:ElementId, propertyName:String):Null<ElementReference> {
		var storey = cad.element(storeyId);
		if (bimClass(storey) != BimSchema.Storey)
			throw new BimError("element is not a Storey: " + storeyId.value);
		var property = storey.property(propertyName);
		if (property == null)
			return null;
		if (property.type != TypedProperty.TypeElementReference)
			throw new BimError("Storey level property has the wrong type: " + propertyName);
		var reference:PersistentReference = cast property.value;
		if (reference.documentId != cad.id.value || reference.targetType != PersistentReference.ElementTarget)
			throw new BimError("Storey level reference is invalid: " + propertyName);
		return new ElementReference(new cadkit.parametric.DocumentId(reference.documentId), new ElementId(reference.targetId));
	}

	public function aggregateChildren(parentId:ElementId):Array<Element>
		return relationshipTargets(parentId, BimSchema.Aggregates);

	public function containedElements(storeyId:ElementId):Array<Element> {
		if (bimClass(cad.element(storeyId)) != BimSchema.Storey)
			throw new BimError("element is not a Storey: " + storeyId.value);
		return relationshipTargets(storeyId, BimSchema.Contains);
	}

	private function relationshipTargets(sourceId:ElementId, typeName:String):Array<Element> {
		var result:Array<Element> = [];
		for (relationship in cad.relationshipsForElement(sourceId))
			if (relationship.typeName == typeName && relationship.source.documentId.value == cad.id.value
				&& relationship.source.elementId.value == sourceId.value)
				result.push(cad.element(relationship.target.elementId));
		result.sort(function(a, b) return Reflect.compare(a.id.value, b.id.value));
		return result;
	}

	private function validateStoreyLevels(baseLevel:Null<ElementReference>, topLevel:Null<ElementReference>):Void {
		if (baseLevel != null)
			cad.resolveElement(baseLevel, ElementKind.Level);
		if (topLevel != null)
			cad.resolveElement(topLevel, ElementKind.Level);
		if (baseLevel != null && topLevel != null && cad.levelElevation(topLevel) <= cad.levelElevation(baseLevel))
			throw new BimError("Storey top Level must be above its base Level");
	}

	private function bimClass(element:Element):String {
		var property = element.property("bim.class");
		if (property == null || property.type != TypedProperty.TypeToken || property.tokenDomain != BimSchema.ElementClass)
			return "";
		return cast property.value;
	}

	private function isContainable(classification:String):Bool
		return classification == BimSchema.Space || classification == BimSchema.Wall || classification == BimSchema.Window
			|| classification == BimSchema.Door || classification == BimSchema.Slab;

	private function canAggregate(parent:String, child:String):Bool
		return (parent == BimSchema.Project && child == BimSchema.Site)
			|| (parent == BimSchema.Site && child == BimSchema.Building)
			|| (parent == BimSchema.Building && child == BimSchema.Storey);

	private function ensureNoSpatialParent(childId:ElementId, relationshipType:String):Void {
		for (relationship in cad.relationshipsForElement(childId))
			if (relationship.typeName == relationshipType && relationship.target.documentId.value == cad.id.value
				&& relationship.target.elementId.value == childId.value)
				throw new BimError("BIM spatial object already has a " + relationshipType + " parent: " + childId.value);
	}

	private function removeContainmentFor(elementId:ElementId):Void {
		for (relationship in cad.relationshipsForElement(elementId))
			if (relationship.typeName == BimSchema.Contains && relationship.target.documentId.value == cad.id.value
				&& relationship.target.elementId.value == elementId.value)
				cad.removeRelationship(relationship.id);
	}

	private function removeIncomingSpatialEdges(elementId:ElementId):Void {
		for (relationship in cad.relationshipsForElement(elementId))
			if ((relationship.typeName == BimSchema.Aggregates || relationship.typeName == BimSchema.Contains)
				&& relationship.target.documentId.value == cad.id.value && relationship.target.elementId.value == elementId.value)
				cad.removeRelationship(relationship.id);
	}

	public function createWall(name:String, length:Float, thickness:Float, height:Float):Element {
		var transaction = cad.beginTransaction();
		try {
			var body = cad.add(new BoxFeature(length, thickness, height));
			cad.trackFeatureCreation(body);
			var wall = cad.createElement(name, body);
			installWall(wall.id, body);
			cad.recompute();
			transaction.commit();
			return wall;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function createLevelWall(name:String, length:Float, thickness:Float, base:ElementReference, top:ElementReference, baseOffset:Float = 0,
			topOffset:Float = 0):Element {
		var transaction = cad.beginTransaction();
		try {
			var body = cad.add(new LevelBoxFeature(length, thickness, base, top, baseOffset, topOffset));
			cad.trackFeatureCreation(body);
			var wall = cad.createElement(name, body);
			installWall(wall.id, body);
			cad.recompute();
			transaction.commit();
			return wall;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function installWall(elementId:ElementId, uncutOutput:Feature):Void {
		var element = cad.element(elementId);
		if (element.kind != "geometry" || uncutOutput.document != cad)
			throw new BimError("invalid wall role reference: " + elementId.value);
		var existing = element.property("bim.class");
		if (existing != null)
			throw new BimError("duplicate or conflicting BIM class: " + elementId.value);
		element.setProperty(TypedProperty.token("bim.class", "wall", "bim.element-class"));
		element.setProperty(TypedProperty.featureReference("bim.authoredFeature", PersistentReference.feature(cad, uncutOutput.id)));
	}

	public function wallRole(id:ElementId):WallRole {
		var element = cad.element(id);
		var classification = element.property("bim.class");
		var authored = element.property("bim.authoredFeature");
		if (classification == null || classification.type != TypedProperty.TypeToken || classification.tokenDomain != "bim.element-class"
			|| classification.value != "wall" || authored == null
			|| authored.type != TypedProperty.TypeFeatureReference)
			throw new BimError("element is not a hosted wall: " + id.value);
		var reference:PersistentReference = cast authored.value;
		if (reference.documentId != cad.id.value || reference.targetType != PersistentReference.FeatureTarget)
			throw new BimError("wall authored feature reference is invalid: " + id.value);
		var featureId = Std.parseInt(reference.targetId);
		var feature = featureId == null ? null : cad.featureById(featureId);
		if (feature == null)
			throw new BimError("wall authored feature is unresolved: " + id.value);
		return new WallRole(id, feature);
	}

	public function allWallRoles():Array<WallRole> {
		var result:Array<WallRole> = [];
		for (element in cad.allElements()) {
			var classification = element.property("bim.class");
			if (classification != null && classification.value == "wall")
				result.push(wallRole(element.id));
		}
		result.sort(function(a, b) return Reflect.compare(a.elementId.value, b.elementId.value));
		return result;
	}

	public function relationship(openingId:ElementId):HostRelationship {
		var result = hostRelationshipForOpening(openingId);
		if (result == null)
			throw new BimError("opening is not hosted: " + openingId.value);
		return toHostRelationship(result);
	}

	public function openingsForWall(wallId:ElementId):Array<HostRelationship> {
		var result:Array<HostRelationship> = [];
		for (relationship in cad.allRelationships())
			if (relationship.typeName == "bim.host" && relationship.target.documentId.value == cad.id.value
				&& relationship.target.elementId.value == wallId.value)
				result.push(toHostRelationship(relationship));
		result.sort(function(a, b) return Reflect.compare(a.openingId.value, b.openingId.value));
		return result;
	}

	public function allRelationships():Array<HostRelationship> {
		var result:Array<HostRelationship> = [];
		for (relationship in cad.allRelationships())
			if (relationship.typeName == "bim.host")
				result.push(toHostRelationship(relationship));
		result.sort(function(a, b) return Reflect.compare(a.openingId.value, b.openingId.value));
		return result;
	}

	private function hostRelationshipForOpening(openingId:ElementId):Null<Relationship> {
		for (relationship in cad.relationshipsForElement(openingId))
			if (relationship.typeName == "bim.host" && relationship.source.documentId.value == cad.id.value
				&& relationship.source.elementId.value == openingId.value)
				return relationship;
		return null;
	}

	private function toHostRelationship(relationship:Relationship):HostRelationship {
		if (relationship.source.documentId.value != cad.id.value || relationship.target.documentId.value != cad.id.value)
			throw new BimError("BIM host relationship endpoints must belong to the same document");
		var openingId = relationship.source.elementId;
		var wallId = relationship.target.elementId;
		wallRole(wallId);
		var unhostPlacementProperty = relationship.property("unhostPlacement");
		var outputProperty = relationship.property("output");
		if (unhostPlacementProperty == null || unhostPlacementProperty.type != TypedProperty.TypePlacement
			|| outputProperty == null || outputProperty.type != TypedProperty.TypeToken || outputProperty.tokenDomain != "bim.host-output")
			throw new BimError("BIM host relationship metadata is incomplete: " + relationship.id.value);
		var parent:Null<ElementReference> = null;
		var parentProperty = relationship.property("unhostParent");
		if (parentProperty != null) {
			if (parentProperty.type != TypedProperty.TypeElementReference)
				throw new BimError("BIM host relationship parent metadata is invalid");
			var parentReference:PersistentReference = cast parentProperty.value;
			parent = new ElementReference(new DocumentId(parentReference.documentId), new ElementId(parentReference.targetId));
		}
		var depthProperty = relationship.property("unhostDepth");
		var depth:Null<Float> = null;
		if (depthProperty != null) {
			if (depthProperty.type != QuantityKind.Length || depthProperty.unit != "mm")
				throw new BimError("BIM host relationship depth metadata is invalid");
			depth = cast depthProperty.value;
		}
		return new HostRelationship(openingId, wallId, hostLength(relationship, "along"), hostLength(relationship, "sill"),
			cast outputProperty.value, cast unhostPlacementProperty.value, parent, depth);
	}

	private function hostLength(relationship:Relationship, name:String):Float {
		var property = relationship.property(name);
		if (property == null || property.type != QuantityKind.Length || property.unit != "mm")
			throw new BimError("BIM host relationship length metadata is invalid: " + name);
		return cast property.value;
	}

	private function storeHostRelationship(relationship:Relationship, value:HostRelationship):Void {
		relationship.setProperty(TypedProperty.quantity("along", QuantityKind.Length, value.along, "mm"));
		relationship.setProperty(TypedProperty.quantity("sill", QuantityKind.Length, value.sill, "mm"));
		relationship.setProperty(TypedProperty.token("output", value.outputName, "bim.host-output"));
		relationship.setProperty(TypedProperty.placement("unhostPlacement", value.unhostPlacement));
		if (value.unhostParent == null)
			relationship.removeProperty("unhostParent");
		else
			relationship.setProperty(TypedProperty.elementReference("unhostParent", value.unhostParent));
		if (value.unhostDepth == null)
			relationship.removeProperty("unhostDepth");
		else
			relationship.setProperty(TypedProperty.quantity("unhostDepth", QuantityKind.Length, value.unhostDepth, "mm"));
	}

	private function synchronizeHostPlacementAuthority():Void {
		var hosted = new Map<String, Bool>();
		for (relationship in cad.allRelationships())
			if (relationship.typeName == "bim.host" && relationship.source.documentId.value == cad.id.value) {
				hosted.set(relationship.source.elementId.value, true);
				var value = toHostRelationship(relationship);
				var opening = cad.element(value.openingId);
				opening.restorePlacementDerived(true);
				var plane = opening.localPlacement.location.plane;
				var desired = new Placement(new Plane(new Vector(value.along, 0, value.sill), Vector.X(), Vector.Z()));
				if (opening.placementParent == null || opening.placementParent.documentId.value != cad.id.value
					|| opening.placementParent.elementId.value != value.wallId.value || plane.origin.x != value.along || plane.origin.y != 0
					|| plane.origin.z != value.sill || plane.xDirection.x != 1 || plane.xDirection.y != 0 || plane.xDirection.z != 0
					|| plane.normal.x != 0 || plane.normal.y != 0 || plane.normal.z != 1)
					cad.restoreElementPlacement(opening, desired, new ElementReference(cad.id, value.wallId));
			}
		for (element in cad.allElements()) {
			if (element.kind != "instance")
				continue;
			var classification = element.property("bim.class");
			var isBimOpening = classification != null && (classification.value == "window" || classification.value == "door");
			if (isBimOpening)
				element.restorePlacementDerived(hosted.exists(element.id.value));
		}
	}

	public function hostOpening(opening:InstanceElement, wallId:ElementId, along:Float, sill:Float):Void {
		if (hostRelationshipForOpening(opening.id) != null)
			throw new BimError("opening is already hosted: " + opening.id.value);
		applyHosting(opening,
			new HostRelationship(opening.id, wallId, along, sill, "opening", opening.localPlacement, opening.placementParent, opening.overrideValue("depth")));
	}

	public function rehostOpening(openingId:ElementId, wallId:ElementId, along:Float, sill:Float):Void {
		var opening:InstanceElement = cast cad.element(openingId);
		var old = relationship(openingId);
		applyHosting(opening, new HostRelationship(openingId, wallId, along, sill, old.outputName, old.unhostPlacement, old.unhostParent, old.unhostDepth));
	}

	public function moveOpening(openingId:ElementId, along:Float, sill:Float):Void {
		var old = relationship(openingId);
		rehostOpening(openingId, old.wallId, along, sill);
	}

	/** Resize a wall and synchronize the opening tools' through-wall depth atomically. */
	public function resizeWall(wallId:ElementId, length:Float, thickness:Float, ?height:Float):Void {
		var role = wallRole(wallId);
		var transaction = cad.beginTransaction();
		try {
			if (role.body != null) {
				var body:BoxFeature = cast role.body;
				body.width.set(length);
				body.depth.set(thickness);
				if (height != null)
					body.height.set(height);
			} else {
				if (height != null)
					throw new BimError("level-driven wall height follows its levels");
				var body:LevelBoxFeature = cast role.levelBody;
				body.width.set(length);
				body.depth.set(thickness);
			}
			for (relationship in openingsForWall(wallId)) {
				var opening:InstanceElement = cast cad.element(relationship.openingId);
				opening.setOverride("depth", thickness);
			}
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function unhostOpening(openingId:ElementId):Void {
		var transaction = cad.beginTransaction();
		try {
			unhostInTransaction(openingId);
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			synchronizeHostPlacementAuthority();
			throw error;
		}
	}

	/** Remove an opening and its host relation in one undoable edit. */
	public function removeOpening(openingId:ElementId):Void {
		var transaction = cad.beginTransaction();
		try {
			if (hostRelationshipForOpening(openingId) != null)
				unhostInTransaction(openingId);
			removeContainmentFor(openingId);
			cad.removeElement(openingId);
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			synchronizeHostPlacementAuthority();
			throw error;
		}
	}

	/** A wall with openings must be emptied explicitly before deletion. */
	public function removeWall(wallId:ElementId):Void {
		var role = wallRole(wallId);
		if (openingsForWall(wallId).length != 0)
			throw new BimError("wall still hosts openings: " + wallId.value);
		for (element in cad.allElements())
			if (element.placementParent != null && element.placementParent.elementId.value == wallId.value)
				throw new BimError("wall still has placement child: " + element.id.value);
		var transaction = cad.beginTransaction();
		try {
			removeContainmentFor(wallId);
			cad.removeElement(wallId);
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	private function unhostInTransaction(openingId:ElementId):Void {
		var relation = relationship(openingId);
		var coreRelationship = hostRelationshipForOpening(openingId);
		if (coreRelationship == null)
			throw new BimError("opening is not hosted: " + openingId.value);
		var opening:InstanceElement = cast cad.element(openingId);
		cad.removeRelationship(coreRelationship.id);
		opening.restorePlacementDerived(false);
		if (relation.unhostDepth == null)
			opening.removeOverride("depth");
		else
			opening.setOverride("depth", relation.unhostDepth);
		var hostedPlacement = opening.localPlacement;
		var hostedParent = opening.placementParent;
		cad.restoreElementPlacement(opening, relation.unhostPlacement, relation.unhostParent);
		cad.recordDocumentChange(new PlacementChange(cad, opening, hostedPlacement, hostedParent, relation.unhostPlacement, relation.unhostParent));
		rebuildWall(relation.wallId);
	}

	private function applyHosting(opening:InstanceElement, next:HostRelationship):Void {
		var target = wallRole(next.wallId);
		validateOpening(opening, target, next, false);
		var coreRelationship = hostRelationshipForOpening(opening.id);
		var before = coreRelationship == null ? null : toHostRelationship(coreRelationship);
		var transaction = cad.beginTransaction();
		try {
			var source = new ElementReference(cad.id, opening.id);
			var parent = new ElementReference(cad.id, target.elementId);
			if (opening.property("bim.class") == null)
				opening.setProperty(TypedProperty.token("bim.class", "window", "bim.element-class"));
			if (coreRelationship == null)
				coreRelationship = cad.createRelationship("bim.host", source, parent);
			else
				cad.setRelationshipEndpoints(coreRelationship, source, parent);
			storeHostRelationship(coreRelationship, next);
			opening.restorePlacementDerived(true);
			var oldPlacement = opening.localPlacement;
			var oldParent = opening.placementParent;
			var newParent = parent;
			var newPlacement = new Placement(new Plane(new Vector(next.along, 0, next.sill), Vector.X(), Vector.Z()));
			cad.restoreElementPlacement(opening, newPlacement, newParent);
			cad.recordDocumentChange(new PlacementChange(cad, opening, oldPlacement, oldParent, newPlacement, newParent));
			opening.setOverride("depth", target.thickness);
			if (before != null && before.wallId.value != next.wallId.value)
				rebuildWall(before.wallId);
			rebuildWall(next.wallId);
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
			synchronizeHostPlacementAuthority();
			throw error;
		}
	}

	private function validateOpening(opening:InstanceElement, wall:WallRole, candidate:HostRelationship, checkDepth:Bool = true):Void {
		if (!Math.isFinite(candidate.along) || !Math.isFinite(candidate.sill))
			throw new BimError("host coordinates must be finite for opening " + opening.id.value);
		var definition = cad.definition(opening.definitionId);
		if (definition.output(candidate.outputName).purpose != "tool")
			throw new BimError("definition output is not a tool: " + candidate.outputName);
		var width = opening.resolved("width");
		var height = opening.resolved("height");
		var parent = opening.placementParent;
		var plane = opening.localPlacement.location.plane;
		var origin = plane.origin;
		if (checkDepth
			&& (parent == null
				|| parent.documentId.value != cad.id.value
				|| parent.elementId.value != wall.elementId.value
				|| !opening.placementDerived
				|| Math.abs(origin.x - candidate.along) > 1e-7
				|| Math.abs(origin.y) > 1e-7
				|| Math.abs(origin.z - candidate.sill) > 1e-7
				|| Math.abs(plane.xDirection.x - 1) > 1e-7
				|| Math.abs(plane.xDirection.y) > 1e-7
				|| Math.abs(plane.xDirection.z) > 1e-7
				|| Math.abs(plane.normal.x) > 1e-7
				|| Math.abs(plane.normal.y) > 1e-7
				|| Math.abs(plane.normal.z - 1) > 1e-7))
			throw new BimError("opening " + opening.id.value + " placement does not match its host coordinates");
		if (checkDepth && Math.abs(opening.resolved("depth") - wall.thickness) > 1e-7)
			throw new BimError("opening " + opening.id.value + " depth does not match wall " + wall.elementId.value);
		if (candidate.along < 0
			|| candidate.sill < 0
			|| candidate.along + width > wall.length
			|| candidate.sill + height > wall.height)
			throw new BimError("opening " + opening.id.value + " lies outside wall " + wall.elementId.value);
		for (other in openingsForWall(wall.elementId)) {
			if (other.openingId.value == opening.id.value)
				continue;
			var otherInstance:InstanceElement = cast cad.element(other.openingId);
			if (candidate.along < other.along + otherInstance.resolved("width")
				&& other.along < candidate.along + width
				&& candidate.sill < other.sill + otherInstance.resolved("height")
				&& other.sill < candidate.sill + height)
				throw new BimError("opening " + opening.id.value + " overlaps opening " + other.openingId.value);
		}
	}

	private function validateAllOpenings():Void {
		for (wall in allWallRoles()) {
			cad.element(wall.elementId);
			wall.height;
		}
		for (relationship in allRelationships()) {
			var element = cad.element(relationship.openingId);
			if (element.kind != "instance")
				throw new BimError("hosted opening is not an instance: " + relationship.openingId.value);
			var opening:InstanceElement = cast element;
			validateOpening(opening, wallRole(relationship.wallId), relationship);
		}
	}

	private function synchronizeLevelPlacements():Void {
		synchronizeHostPlacementAuthority();
		for (wall in allWallRoles())
			if (wall.levelBody != null) {
				var element = cad.element(wall.elementId);
				var origin = element.localPlacement.location.plane.origin;
				var desired = wall.baseElevation();
				if (origin.z != desired)
					cad.restoreElementPlacement(element,
						new Placement(new Plane(new Vector(origin.x, origin.y, desired), element.localPlacement.location.plane.xDirection,
							element.localPlacement.location.plane.normal)),
						element.placementParent);
			}
	}

	private function rebuildWall(wallId:ElementId):Void {
		var role = wallRole(wallId);
		var wall = cad.element(wallId);
		var oldCut:Null<BooleanFeature> = null;
		if (wall.output != role.uncutOutput)
			oldCut = cast wall.output;
		if (oldCut != null) {
			var oldCollection:ToolCollectionFeature = cast oldCut.second;
			cad.setFeatureActive(oldCut, false);
			cad.setFeatureActive(oldCollection, false);
			for (tool in oldCollection.tools)
				cad.setFeatureActive(tool, false);
		}
		var hosted = openingsForWall(wallId);
		if (hosted.length == 0) {
			wall.setOutput(role.uncutOutput);
			return;
		}
		var tools:Array<Feature> = [];
		var labels:Array<String> = [];
		for (relationship in hosted) {
			var reference = new ElementReference(cad.id, relationship.openingId);
			var tool = cad.add(new DefinitionOutputFeature(reference, relationship.outputName));
			cad.trackFeatureCreation(tool);
			tools.push(tool);
			labels.push(relationship.openingId.value);
		}
		var collection = cad.add(new ToolCollectionFeature(tools, labels));
		cad.trackFeatureCreation(collection);
		var cut = cad.add(new BooleanFeature(role.uncutOutput, collection, BooleanOperation.Cut));
		cad.trackFeatureCreation(cut);
		wall.setOutput(cut);
	}

	public function installRelationship(value:HostRelationship):Void {
		if (hostRelationshipForOpening(value.openingId) != null)
			throw new BimError("duplicate host relationship: " + value.openingId.value);
		var opening = cad.element(value.openingId);
		if (opening.kind != "instance")
			throw new BimError("hosted opening is not an instance: " + value.openingId.value);
		wallRole(value.wallId);
		var relation = cad.installRelationship(new RelationshipId(), "bim.host", new ElementReference(cad.id, value.openingId),
			new ElementReference(cad.id, value.wallId));
		if (opening.property("bim.class") == null)
			opening.restoreProperty("bim.class", TypedProperty.token("bim.class", "window", "bim.element-class"));
		storeHostRelationship(relation, value);
		opening.restorePlacementDerived(true);
		cad.restoreElementPlacement(opening, new Placement(new Plane(new Vector(value.along, 0, value.sill), Vector.X(), Vector.Z())),
			new ElementReference(cad.id, value.wallId));
	}

	public function undo():Bool {
		var result = cad.undo();
		if (result) {
			synchronizeHostPlacementAuthority();
			cad.recompute();
		}
		return result;
	}

	public function redo():Bool {
		var result = cad.redo();
		if (result) {
			synchronizeHostPlacementAuthority();
			cad.recompute();
		}
		return result;
	}

	public function close():Void {
		cad.removeBeforeRecomputeHook(beforeHookId);
		cad.removeAfterRecomputeHook(afterHookId);
		cad.close();
	}
}
