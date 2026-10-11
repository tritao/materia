package app;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import stockkit.Stock;
import stockkit.StockColoring;
import stockkit.StockMesh;
import stockkit.StockPreviewWorker;

/**
 * A simulated stock shown in the scene: contoured on its own thread from snapshots as it changes, in chunks, so only
 * the chunks that changed are meshed again and handed over. What `MachiningStock` (material cut away) and
 * `PrintingStock` (material laid down) share; the stock itself belongs to them.
 */
class StockMeshes {
	/** Contours the stock on its own thread, from snapshots, as it changes. */
	final preview:StockPreviewWorker;
	/** Each chunk's latest mesh, row by row, as refreshes have delivered them. */
	var chunkMeshes:Array<Null<StockMesh>> = [];
	var changed = true;

	public function new(stock:Stock, coloring:StockColoring) {
		preview = new StockPreviewWorker(stock, coloring);
	}

	/** The stock changed: it is to be contoured again. */
	public function markChanged():Void changed = true;

	/** Whether the stock changed since the last refresh started. */
	public function hasChanged():Bool return changed;

	/** Starts contouring the stock as it is now, unless a refresh is under way; returns whether it started. */
	public function refresh():Bool {
		if (!preview.refresh()) return false;
		changed = false;
		return true;
	}

	/**
	 * The chunks finished refreshes rebuilt, as geometry by chunk index, or null when none finished.
	 * `chunks` is how many chunks the stock shows as.
	 */
	public function take():Null<Map<Int, GeometryData>> {
		var updates = preview.take();
		if (updates.length == 0) return null;
		var changedChunks:Map<Int, GeometryData> = new Map();
		for (update in updates) {
			if (chunkMeshes.length != update.chunksX * update.chunksY)
				chunkMeshes = [for (_ in 0...update.chunksX * update.chunksY) null];
			for (k in 0...update.chunks.length) {
				chunkMeshes[update.chunks[k]] = update.meshes[k];
				changedChunks.set(update.chunks[k], meshGeometry([update.meshes[k]]));
			}
		}
		return changedChunks;
	}

	public function chunks():Int return chunkMeshes.length;

	/** The whole stock as it is now, as one mesh: waits for its contouring. */
	public function geometry():GeometryData {
		preview.wait();
		refresh();
		preview.wait();
		take();
		return meshGeometry([for (mesh in chunkMeshes) if (mesh != null) mesh]);
	}

	/** Stops the worker: it reads colouring targets while it contours, so stop it before they go. */
	public function dispose():Void preview.dispose();

	static function meshGeometry(meshes:Array<StockMesh>):GeometryData {
		var vertices = 0, triangles = 0;
		for (mesh in meshes) {
			vertices += mesh.vertexCount;
			triangles += mesh.triangleCount;
		}
		var positions = Bytes.alloc(vertices * 12), normals = Bytes.alloc(vertices * 12);
		var colors = Bytes.alloc(vertices * 4), indices = Bytes.alloc(triangles * 12);
		var vertexBase = 0, indexBase = 0;
		for (mesh in meshes) {
			positions.blit(vertexBase * 12, mesh.positions, 0, mesh.vertexCount * 12);
			normals.blit(vertexBase * 12, mesh.normals, 0, mesh.vertexCount * 12);
			colors.blit(vertexBase * 4, mesh.colors, 0, mesh.vertexCount * 4);
			if (vertexBase == 0) indices.blit(indexBase * 4, mesh.indices, 0, mesh.triangleCount * 12);
			else for (index in 0...mesh.triangleCount * 3)
				indices.setInt32((indexBase + index) * 4, mesh.indices.getInt32(index * 4) + vertexBase);
			vertexBase += mesh.vertexCount;
			indexBase += mesh.triangleCount * 3;
		}
		var geometry = new GeometryData();
		geometry.addStream(1, 2, positions, vertices, 12);
		geometry.addStream(2, 2, normals, vertices, 12);
		geometry.addStream(6, 4, colors, vertices, 4);
		geometry.setIndexBuffer(indices, triangles * 3);
		// Bounds of the meshes' own vertices, so a chunk culls as itself.
		if (vertices > 0) {
			var low = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
			var high = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
			for (vertex in 0...vertices)
				for (axis in 0...3) {
					var value = positions.getFloat(vertex * 12 + axis * 4);
					if (value < low[axis]) low[axis] = value;
					if (value > high[axis]) high[axis] = value;
				}
			geometry.setBounds(low[0], low[1], low[2], high[0], high[1], high[2]);
		}
		return geometry;
	}
}
