package nativekit.scene;

import NativeKitScene;
import NativeKitSceneRender;

/** Read-only spatial queries and CPU ray picking for one scene snapshot. */
class SpatialIndex {
	final owner:Ownednkscene_render_spatial_index;
	var disposed:Bool = false;

	private function new(owner:Ownednkscene_render_spatial_index) {
		this.owner = owner;
	}

	public static function create(snapshot:SceneSnapshot, ?view:SceneView):SpatialIndex {
		var owner:Ownednkscene_render_spatial_index;
		if (view == null) {
			var made = NativeKitSceneRender.nkscene_render_spatial_index_create(snapshot.nativeHandle());
			check(made.status, "spatialIndex.create");
			owner = made.out_index;
		} else {
			var made = NativeKitSceneRender.nkscene_render_spatial_index_create_with_view(
				snapshot.nativeHandle(), view.nativeValue());
			check(made.status, "spatialIndex.create");
			owner = made.out_index;
		}
		return new SpatialIndex(owner);
	}

	public function sourceRevision():haxe.Int64 {
		ensureLive();
		var result = NativeKitSceneRender.nkscene_render_spatial_index_get_revision(owner.borrow());
		check(result.status, "spatialIndex.sourceRevision");
		return result.out_revision;
	}

	/** Refit this index after a transform-only update to one existing node. */
	public function updateNode(snapshot:SceneSnapshot, node:NodeId):Bool {
		ensureLive();
		var result = NativeKitSceneRender.nkscene_render_spatial_index_update_node(
			owner.borrow(), snapshot.nativeHandle(), node.nativeValue());
		check(result.status, "spatialIndex.updateNode");
		return result.out_updated != 0;
	}

	/** Returns nodes whose snapshot bounds overlap the supplied box. */
	public function queryBounds(minX:Float, minY:Float, minZ:Float,
			maxX:Float, maxY:Float, maxZ:Float):Array<NodeId> {
		ensureLive();
		var bounds = new nkscene_bounds();
		bounds.set_valid(1);
		bounds.set_minimum(0, minX);
		bounds.set_minimum(1, minY);
		bounds.set_minimum(2, minZ);
		bounds.set_maximum(0, maxX);
		bounds.set_maximum(1, maxY);
		bounds.set_maximum(2, maxZ);
		var result = NativeKitSceneRender.nkscene_render_spatial_index_query_bounds(
			owner.borrow(), bounds);
		check(result.status, "spatialIndex.queryBounds");
		return readNodes(result.out_count);
	}

	/** Returns nodes whose snapshot bounds intersect the supplied ray. */
	public function queryRay(originX:Float, originY:Float, originZ:Float,
			directionX:Float, directionY:Float, directionZ:Float):Array<NodeId> {
		ensureLive();
		var ray = makeRay(originX, originY, originZ, directionX, directionY, directionZ),
			result = NativeKitSceneRender.nkscene_render_spatial_index_query_ray(owner.borrow(), ray);
		check(result.status, "spatialIndex.queryRay");
		return readNodes(result.out_count);
	}

	/** Returns the nearest visible triangle hit, including source and subelement identity. */
	public function pickRay(originX:Float, originY:Float, originZ:Float,
			directionX:Float, directionY:Float, directionZ:Float):PickResult {
		ensureLive();
		var ray = makeRay(originX, originY, originZ, directionX, directionY, directionZ),
			result = NativeKitSceneRender.nkscene_render_spatial_index_pick_ray(owner.borrow(), ray);
		check(result.status, "spatialIndex.pickRay");
		return new PickResult(result.out_result);
	}

	public function dispose():Void {
		if (disposed)
			return;
		owner.close();
		disposed = true;
	}

	public function isDisposed():Bool
		return disposed;

	function readNodes(count:haxe.Int64):Array<NodeId> {
		var result:Array<NodeId> = [],
			total = haxe.Int64.toInt(count);
		for (index in 0...total) {
			var node = NativeKitSceneRender.nkscene_render_spatial_index_get_node(
				owner.borrow(), index);
			check(node.status, "spatialIndex.getNode");
			result.push(NodeId.fromNative(node.out_result.get_node()));
		}
		return result;
	}

	static function makeRay(originX:Float, originY:Float, originZ:Float,
			directionX:Float, directionY:Float, directionZ:Float):nkscene_render_ray {
		var ray = new nkscene_render_ray();
		ray.set_origin(0, originX);
		ray.set_origin(1, originY);
		ray.set_origin(2, originZ);
		ray.set_direction(0, directionX);
		ray.set_direction(1, directionY);
		ray.set_direction(2, directionZ);
		return ray;
	}

	function ensureLive():Void {
		if (disposed)
			throw "Spatial index has been disposed";
	}

	static function check(status:Int, operation:String):Void {
		if (status != 0)
			throw '$operation failed with NativeKit scene status $status';
	}
}
