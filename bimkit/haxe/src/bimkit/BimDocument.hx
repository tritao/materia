package bimkit;

import bimkit.HostRelationshipChange;
import bimkit.WallRoleChange;
import bimkit.BimWindowDefinition;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.Definition;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Feature;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;
import cadkit.parametric.PlacementChange;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.LevelBoxFeature;
import cadkit.parametric.features.DefinitionOutputFeature;
import cadkit.parametric.features.ToolCollectionFeature;

/** BIM roles and hosting relationships layered over an ordinary CadKit document. */
class BimDocument {
	public final cad:Document;

	private final walls:Map<String, WallRole>;
	private final relationships:Map<String, HostRelationship>;

	public function new(?cad:Document) {
		BimWindowDefinition.registerEvaluator();
		this.cad = cad == null ? new Document() : cad;
		walls = new Map();
		relationships = new Map();
		this.cad.beforeRecompute = validateAllOpenings;
		this.cad.afterRecompute = synchronizeLevelPlacements;
	}

	public function createWindowDefinition(name:String, width:Float, height:Float, frameThickness:Float, depth:Float):Definition
		return BimWindowDefinition.create(cad, name, width, height, frameThickness, depth);

	public function createWall(name:String, length:Float, thickness:Float, height:Float):Element {
		var transaction = cad.beginTransaction();
		try {
			var body = cad.add(new BoxFeature(length, thickness, height));
			cad.trackFeatureCreation(body);
			var wall = cad.createElement(name, body);
			var role = new WallRole(wall.id, body);
			restoreWallRole(role, true);
			cad.recordDocumentChange(new WallRoleChange(this, role, false, true));
			cad.setOutputTracked(body);
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
			var role = new WallRole(wall.id, body);
			restoreWallRole(role, true);
			cad.recordDocumentChange(new WallRoleChange(this, role, false, true));
			cad.setOutputTracked(body);
			cad.recompute();
			transaction.commit();
			return wall;
		} catch (error:Dynamic) {
			transaction.cancel();
			throw error;
		}
	}

	public function installWall(elementId:ElementId, uncutOutput:Feature):Void {
		if (walls.exists(elementId.value))
			throw new BimError("duplicate wall role: " + elementId.value);
		var element = cad.element(elementId);
		if (element.kind != "geometry" || uncutOutput.document != cad)
			throw new BimError("invalid wall role reference: " + elementId.value);
		walls.set(elementId.value, new WallRole(elementId, uncutOutput));
	}

	public function restoreWallRole(role:WallRole, present:Bool):Void {
		if (present)
			walls.set(role.elementId.value, role);
		else
			walls.remove(role.elementId.value);
	}

	public function wallRole(id:ElementId):WallRole {
		var result = walls.get(id.value);
		if (result == null)
			throw new BimError("element is not a hosted wall: " + id.value);
		return result;
	}

	public function allWallRoles():Array<WallRole> {
		var result = [for (role in walls) role];
		result.sort(function(a, b) return Reflect.compare(a.elementId.value, b.elementId.value));
		return result;
	}

	public function relationship(openingId:ElementId):HostRelationship {
		var result = relationships.get(openingId.value);
		if (result == null)
			throw new BimError("opening is not hosted: " + openingId.value);
		return result;
	}

	public function openingsForWall(wallId:ElementId):Array<HostRelationship> {
		var result = [
			for (relationship in relationships)
				if (relationship.wallId.value == wallId.value) relationship
		];
		result.sort(function(a, b) return Reflect.compare(a.openingId.value, b.openingId.value));
		return result;
	}

	public function allRelationships():Array<HostRelationship> {
		var result = [for (relationship in relationships) relationship];
		result.sort(function(a, b) return Reflect.compare(a.openingId.value, b.openingId.value));
		return result;
	}

	public function hostOpening(opening:InstanceElement, wallId:ElementId, along:Float, sill:Float):Void {
		if (relationships.exists(opening.id.value))
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
		var validator = cad.beforeRecompute;
		cad.beforeRecompute = null;
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
			cad.beforeRecompute = validator;
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			cad.beforeRecompute = validator;
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
			throw error;
		}
	}

	/** Remove an opening and its host relation in one undoable edit. */
	public function removeOpening(openingId:ElementId):Void {
		var transaction = cad.beginTransaction();
		try {
			if (relationships.exists(openingId.value))
				unhostInTransaction(openingId);
			cad.removeElement(openingId);
			cad.recompute();
			transaction.commit();
		} catch (error:Dynamic) {
			transaction.cancel();
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
			restoreWallRole(role, false);
			cad.recordDocumentChange(new WallRoleChange(this, role, true, false));
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
		var opening:InstanceElement = cast cad.element(openingId);
		restoreRelationship(openingId.value, null);
		cad.recordDocumentChange(new HostRelationshipChange(this, openingId.value, relation, null));
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
		var before = relationships.get(opening.id.value);
		var transaction = cad.beginTransaction();
		try {
			restoreRelationship(opening.id.value, next);
			cad.recordDocumentChange(new HostRelationshipChange(this, opening.id.value, before, next));
			var oldPlacement = opening.localPlacement;
			var oldParent = opening.placementParent;
			var newParent = new ElementReference(cad.id, target.elementId);
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
		for (wall in walls) {
			cad.element(wall.elementId);
			wall.height;
		}
		for (relationship in relationships) {
			var element = cad.element(relationship.openingId);
			if (element.kind != "instance")
				throw new BimError("hosted opening is not an instance: " + relationship.openingId.value);
			var opening:InstanceElement = cast element;
			validateOpening(opening, wallRole(relationship.wallId), relationship);
		}
	}

	private function synchronizeLevelPlacements():Void {
		for (wall in walls)
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
			cad.setOutputTracked(role.uncutOutput);
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
		cad.setOutputTracked(cut);
	}

	public function restoreRelationship(opening:String, value:Null<HostRelationship>):Void {
		if (value == null)
			relationships.remove(opening);
		else
			relationships.set(opening, value);
		var element = cad.element(new ElementId(opening));
		element.restorePlacementDerived(value != null);
	}

	public function installRelationship(value:HostRelationship):Void {
		if (relationships.exists(value.openingId.value))
			throw new BimError("duplicate host relationship: " + value.openingId.value);
		var opening = cad.element(value.openingId);
		if (opening.kind != "instance")
			throw new BimError("hosted opening is not an instance: " + value.openingId.value);
		wallRole(value.wallId);
		restoreRelationship(value.openingId.value, value);
		cad.restoreElementPlacement(opening, new Placement(new Plane(new Vector(value.along, 0, value.sill), Vector.X(), Vector.Z())),
			new ElementReference(cad.id, value.wallId));
	}

	public function undo():Bool {
		var result = cad.undo();
		if (result)
			cad.recompute();
		return result;
	}

	public function redo():Bool {
		var result = cad.redo();
		if (result)
			cad.recompute();
		return result;
	}

	public function close():Void {
		cad.beforeRecompute = null;
		cad.afterRecompute = null;
		cad.close();
	}
}
