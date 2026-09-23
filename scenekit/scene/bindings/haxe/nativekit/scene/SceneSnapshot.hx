package nativekit.scene;

import NativeKitScene;

/** Immutable read-only scene state captured at one scene revision. */
class SceneSnapshot {
	final owner:Ownednkscene_snapshot;
	var disposed:Bool = false;
	var nodeCache:Null<Array<SceneNode>> = null;
	var nodeIndex:Null<Map<String, SceneNode>> = null;
	var childrenIndex:Null<Map<String, Array<NodeId>>> = null;
	var sourceNodeIndex:Null<Map<String, Array<NodeId>>> = null;
	var geometryNodeIndex:Null<Map<String, Array<NodeId>>> = null;
	var materialNodeIndex:Null<Map<String, Array<NodeId>>> = null;

	@:allow(Scene)
	private function new(owner:Ownednkscene_snapshot) {
		this.owner = owner;
	}

	public function nativeHandle():nkscene_snapshot {
		ensureLive();
		return owner.borrow();
	}

	public function revision():haxe.Int64 {
		ensureLive();
		var result = NativeKitScene.nkscene_snapshot_get_revision(owner.borrow());
		check(result.status, "snapshot.revision");
		return result.out_revision;
	}

	public function nodeCount():Int {
		ensureLive();
		var result = NativeKitScene.nkscene_snapshot_get_node_count(owner.borrow());
		check(result.status, "snapshot.nodeCount");
		return haxe.Int64.toInt(result.out_count);
	}

	public function nodeAt(index:Int):SceneNode {
		ensureLive();
		if (index < 0)
			throw "SceneSnapshot node index cannot be negative";
		var cached = nodeCache;
		if (cached != null) {
			if (index >= cached.length)
				throw "SceneSnapshot node index is out of range";
			return cached[index];
		}
		var value = new nkscene_snapshot_node();
		value.set_struct_size(nkscene_snapshot_node.size());
		var result = NativeKitScene.nkscene_snapshot_get_node(owner.borrow(), index, value);
		check(result.status, "snapshot.nodeAt");
		return new SceneNode(value);
	}

	/** Returns all published scene nodes. */
	public function nodes():Array<SceneNode>
		return ensureNodeCache();

	public function findNode(node:NodeId):Null<SceneNode> {
		ensureLive();
		var value = new nkscene_snapshot_node();
		value.set_struct_size(nkscene_snapshot_node.size());
		var result = NativeKitScene.nkscene_snapshot_find_node(
			owner.borrow(), node.nativeValue(), value);
		if (result.status == NativeKitSceneConstants.NKS_ERROR_STALE_ID)
			return null;
		check(result.status, "snapshot.findNode");
		return new SceneNode(value);
	}

	public function childrenOf(parent:NodeId):Array<NodeId> {
		ensureNodeCache();
		var result = childrenIndex.get(key(parent));
		return result == null ? [] : result.copy();
	}

	/** Returns nodes associated with one source entity. */
	public function nodesForSource(source:haxe.Int64):Array<NodeId> {
		ensureLive();
		if (sourceNodeIndex == null)
			sourceNodeIndex = new Map();
		var sourceKey = haxe.Int64.toStr(source),
			cached = sourceNodeIndex.get(sourceKey);
		if (cached != null)
			return cached;

		var entity = new nkscene_entity_id();
		entity.set_value(source);
		var countResult = NativeKitScene.nkscene_snapshot_get_source_node_count(
			owner.borrow(), entity);
		check(countResult.status, "snapshot.sourceNodeCount");
		var result:Array<NodeId> = [];
		for (index in 0...haxe.Int64.toInt(countResult.out_count)) {
		var value = NativeKitScene.nkscene_snapshot_get_source_node(
				owner.borrow(), entity, index);
			check(value.status, "snapshot.sourceNode");
			result.push(NodeId.fromNative(value.out_node));
		}
		sourceNodeIndex.set(sourceKey, result);
		return result;
	}

	/** Returns nodes associated with one geometry resource. */
	public function nodesForGeometry(geometry:Geometry):Array<NodeId> {
		ensureLive();
		if (geometryNodeIndex == null)
			geometryNodeIndex = new Map();
		var geometryKey = haxe.Int64.toStr(geometry.id().get_value()),
			cached = geometryNodeIndex.get(geometryKey);
		if (cached != null)
			return cached;

		var countResult = NativeKitScene.nkscene_snapshot_get_geometry_node_count(
			owner.borrow(), geometry.id());
		check(countResult.status, "snapshot.geometryNodeCount");
		var result:Array<NodeId> = [];
		for (index in 0...haxe.Int64.toInt(countResult.out_count)) {
			var value = NativeKitScene.nkscene_snapshot_get_geometry_node(
				owner.borrow(), geometry.id(), index);
			check(value.status, "snapshot.geometryNode");
			result.push(NodeId.fromNative(value.out_node));
		}
		geometryNodeIndex.set(geometryKey, result);
		return result;
	}

	/** Returns nodes associated with one material resource. */
	public function nodesForMaterial(material:Material):Array<NodeId> {
		ensureLive();
		if (materialNodeIndex == null)
			materialNodeIndex = new Map();
		var materialKey = haxe.Int64.toStr(material.id().get_value()),
			cached = materialNodeIndex.get(materialKey);
		if (cached != null)
			return cached;

		var countResult = NativeKitScene.nkscene_snapshot_get_material_node_count(
			owner.borrow(), material.id());
		check(countResult.status, "snapshot.materialNodeCount");
		var result:Array<NodeId> = [];
		for (index in 0...haxe.Int64.toInt(countResult.out_count)) {
			var value = NativeKitScene.nkscene_snapshot_get_material_node(
				owner.borrow(), material.id(), index);
			check(value.status, "snapshot.materialNode");
			result.push(NodeId.fromNative(value.out_node));
		}
		materialNodeIndex.set(materialKey, result);
		return result;
	}

	public function dispose():Void {
		if (disposed)
			return;
		owner.close();
		disposed = true;
	}

	public function isDisposed():Bool
		return disposed;

	function ensureLive():Void {
		if (disposed)
			throw "Scene snapshot has been disposed";
	}

	static function check(status:Int, operation:String):Void {
		if (status != NativeKitSceneConstants.NKS_OK)
			throw '$operation failed with NativeKit scene status $status';
	}

	function ensureNodeCache():Array<SceneNode> {
		var result = nodeCache;
		if (result != null)
			return result;

		result = [];
		var byValue:Map<String, SceneNode> = new Map(),
			byParent:Map<String, Array<NodeId>> = new Map(),
			count = nodeCount(),
			pageSize = NativeKitSceneConstants.NKS_SCENE_SNAPSHOT_NODE_PAGE_CAPACITY,
			page = new nkscene_snapshot_node_page();
		page.set_struct_size(nkscene_snapshot_node_page.size());
		var offset = 0;
		while (offset < count) {
			var pageResult = NativeKitScene.nkscene_snapshot_get_node_page(
				owner.borrow(), offset, page);
			check(pageResult.status, "snapshot.nodePage");
			var pageCount = page.get_count();
			if (pageCount <= 0 || pageCount > pageSize || pageCount > count - offset)
				throw "SceneSnapshot node page returned an invalid count";
			for (index in 0...pageCount) {
				var info = new SceneNode(page.get_nodes(index));
				result.push(info);
				byValue.set(key(info.node()), info);
				var parent = info.parent();
				if (parent != null) {
					var children = byParent.get(key(parent));
					if (children == null) {
						children = [];
						byParent.set(key(parent), children);
					}
					children.push(info.node());
				}
			}
			offset += pageCount;
		}
		nodeCache = result;
		nodeIndex = byValue;
		childrenIndex = byParent;
		return result;
	}

	static function key(node:NodeId):String
		return haxe.Int64.toStr(node.stableValue());
}
