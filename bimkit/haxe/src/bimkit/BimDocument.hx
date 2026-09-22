package bimkit;

import bimkit.HostRelationshipChange;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.parametric.Document;
import cadkit.parametric.Element;
import cadkit.parametric.ElementId;
import cadkit.parametric.ElementReference;
import cadkit.parametric.Feature;
import cadkit.parametric.InstanceElement;
import cadkit.parametric.Placement;
import cadkit.parametric.PlacementChange;
import cadkit.parametric.features.BooleanFeature;
import cadkit.parametric.features.BooleanOperation;
import cadkit.parametric.features.BoxFeature;
import cadkit.parametric.features.DefinitionOutputFeature;
import cadkit.parametric.features.ToolCollectionFeature;

/** BIM roles and hosting relationships layered over an ordinary CadKit document. */
class BimDocument {
	public final cad:Document;

	private final walls:Map<String, WallRole>;
	private final relationships:Map<String, HostRelationship>;

	public function new(?cad:Document) {
		this.cad = cad == null ? new Document() : cad;
		walls = new Map();
		relationships = new Map();
	}

	public function createWall(name:String, length:Float, thickness:Float, height:Float):Element {
		var body = cad.add(new BoxFeature(length, thickness, height));
		var wall = cad.createElement(name, body);
		walls.set(wall.id.value, new WallRole(wall.id, body));
		cad.setOutput(body);
		cad.recompute();
		return wall;
	}

	public function installWall(elementId:ElementId, uncutOutput:Feature):Void
		walls.set(elementId.value, new WallRole(elementId, uncutOutput));

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

	public function hostOpening(opening:InstanceElement, wallId:ElementId, along:Float, sill:Float):Void {
		if (relationships.exists(opening.id.value))
			throw new BimError("opening is already hosted: " + opening.id.value);
		applyHosting(opening, new HostRelationship(opening.id, wallId, along, sill));
	}

	public function rehostOpening(openingId:ElementId, wallId:ElementId, along:Float, sill:Float):Void {
		var opening:InstanceElement = cast cad.element(openingId);
		applyHosting(opening, new HostRelationship(openingId, wallId, along, sill));
	}

	public function moveOpening(openingId:ElementId, along:Float, sill:Float):Void {
		var old = relationship(openingId);
		rehostOpening(openingId, old.wallId, along, sill);
	}

	private function applyHosting(opening:InstanceElement, next:HostRelationship):Void {
		var target = wallRole(next.wallId);
		validateOpening(opening, target, next);
		var before = relationships.get(opening.id.value);
		var transaction = cad.beginTransaction();
		try {
			restoreRelationship(opening.id.value, next);
			cad.recordDocumentChange(new HostRelationshipChange(this, opening.id.value, before, next));
			opening.setOverride("depth", target.thickness);
			var oldPlacement = opening.localPlacement;
			var oldParent = opening.placementParent;
			var newParent = new ElementReference(cad.id, target.elementId);
			var newPlacement = new Placement(new Plane(new Vector(next.along, 0, next.sill), Vector.X(), Vector.Z()));
			cad.restoreElementPlacement(opening, newPlacement, newParent);
			cad.recordDocumentChange(new PlacementChange(cad, opening, oldPlacement, oldParent, newPlacement, newParent));
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

	private function validateOpening(opening:InstanceElement, wall:WallRole, candidate:HostRelationship):Void {
		if (!Math.isFinite(candidate.along) || !Math.isFinite(candidate.sill))
			throw new BimError("host coordinates must be finite for opening " + opening.id.value);
		var definition = cad.definition(opening.definitionId);
		if (definition.output(candidate.outputName).purpose != "tool")
			throw new BimError("definition output is not a tool: " + candidate.outputName);
		var width = opening.resolved("width");
		var height = opening.resolved("height");
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

	private function rebuildWall(wallId:ElementId):Void {
		var role = wallRole(wallId);
		var wall = cad.element(wallId);
		var hosted = openingsForWall(wallId);
		if (hosted.length == 0) {
			wall.setOutput(role.uncutOutput);
			return;
		}
		var tools:Array<Feature> = [];
		var labels:Array<String> = [];
		for (relationship in hosted) {
			var reference = new ElementReference(cad.id, relationship.openingId);
			tools.push(cad.add(new DefinitionOutputFeature(reference, relationship.outputName)));
			labels.push(relationship.openingId.value);
		}
		var collection = cad.add(new ToolCollectionFeature(tools, labels));
		var cut = cad.add(new BooleanFeature(role.uncutOutput, collection, BooleanOperation.Cut));
		wall.setOutput(cut);
		cad.setOutput(cut);
	}

	public function restoreRelationship(opening:String, value:Null<HostRelationship>):Void {
		if (value == null)
			relationships.remove(opening);
		else
			relationships.set(opening, value);
		var element = cad.element(new ElementId(opening));
		element.restorePlacementDerived(value != null);
	}

	public function installRelationship(value:HostRelationship):Void
		restoreRelationship(value.openingId.value, value);

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

	public function close():Void
		cad.close();
}
