package examples.nativekit;

import cadkit.Mesh;
import nativekit.gpu.Buffer;
import nativekit.gpu.Renderer;

/** Optional application-layer bridge; CadKit itself does not import NativeKit. */
class MeshUpload {
	public static function upload(renderer:Renderer, mesh:Mesh):{
		vertices:Buffer,
		normals:Buffer,
		indices:Buffer
	} {
		return {
			vertices: Buffer.fromBytes(renderer, mesh.vertices),
			normals: Buffer.fromBytes(renderer, mesh.normals),
			indices: Buffer.fromBytes(renderer, mesh.indices)
		};
	}
}
