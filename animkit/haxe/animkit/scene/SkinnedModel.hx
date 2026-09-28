package animkit.scene;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import haxe.io.Bytes;
import nativekit.scene.Geometry;
import nativekit.scene.GeometryData;
import nativekit.scene.ImageData;
import nativekit.scene.Material;
import nativekit.scene.MaterialData;
import nativekit.scene.NodeId;
import nativekit.scene.SamplerData;
import nativekit.scene.Scene;
import nativekit.scene.Texture;
import nativekit.scene.TextureData;
import nativekit.scene.Transform;

/**
 * Presents an animation instance in a SceneKit scene: one root node with a
 * child node per primitive. Skinning runs on the CPU, so update() republishes
 * each primitive's deformed positions and normals after the instance is
 * evaluated. Place the model by transforming root. The instance's asset must
 * still be live during construction, which reads its indices, UVs, and images.
 */
class SkinnedModel {
	static inline var SEMANTIC_POSITION:Int = 1;
	static inline var SEMANTIC_NORMAL:Int = 2;
	static inline var SEMANTIC_TEXCOORD0:Int = 4;
	static inline var FORMAT_FLOAT32X2:Int = 1;
	static inline var FORMAT_FLOAT32X3:Int = 2;
	static inline var ALPHA_MASK:Int = 2;
	static inline var ALPHA_BLEND:Int = 3;

	public final root:NodeId;
	public final primitiveNodes:Array<NodeId> = [];
	public final instance:AnimationInstance;

	final scene:Scene;
	final geometries:Array<Geometry> = [];
	final indices:Array<Bytes> = [];
	final texcoords:Array<Null<Bytes>> = [];

	public function new(scene:Scene, instance:AnimationInstance, ?parent:NodeId, ?name:String) {
		this.scene = scene;
		this.instance = instance;
		var asset = instance.asset;
		var materials = createMaterials(asset);
		var fallback = scene.createMaterial();
		scene.setMaterialData(fallback, MaterialData.opaque(0.8, 0.8, 0.8));

		for (index in 0...asset.primitives.length) {
			indices.push(asset.readIndices(index));
			texcoords.push(asset.readTexcoords(index));
		}
		var bounds = instance.bounds();
		var data = [for (index in 0...asset.primitives.length) geometryData(index, bounds)];
		for (geometry in scene.createGeometryBatch(data))
			geometries.push(geometry);

		var transaction = scene.beginTransaction();
		root = transaction.createNode();
		if (parent != null)
			transaction.setParent(root, parent);
		transaction.setName(root, name != null ? name : "Animated model");
		transaction.setTransform(root, Transform.identity());
		for (index in 0...asset.primitives.length) {
			var primitive = asset.primitives[index];
			var node = transaction.createNode();
			transaction.setParent(node, root);
			transaction.setName(node, primitive.name);
			transaction.setGeometry(node, geometries[index]);
			transaction.setMaterial(node, primitive.material >= 0 ? materials[primitive.material] : fallback);
			transaction.setTransform(node, Transform.identity());
			primitiveNodes.push(node);
		}
		transaction.commit();
	}

	/** Publishes the instance's current deformed geometry. Call after evaluating it. */
	public function update():Void {
		var bounds = instance.bounds();
		for (index in 0...geometries.length)
			scene.setGeometryData(geometries[index], geometryData(index, bounds));
	}

	function geometryData(index:Int, bounds:Array<Float>):GeometryData {
		var vertexCount = instance.asset.primitives[index].vertexCount;
		var data = new GeometryData()
			.addStream(SEMANTIC_POSITION, FORMAT_FLOAT32X3, instance.readPositions(index), vertexCount, 12)
			.addStream(SEMANTIC_NORMAL, FORMAT_FLOAT32X3, instance.readNormals(index), vertexCount, 12);
		var uv = texcoords[index];
		if (uv != null)
			data.addStream(SEMANTIC_TEXCOORD0, FORMAT_FLOAT32X2, uv, vertexCount, 8);
		data.setIndexBuffer(indices[index], instance.asset.primitives[index].indexCount);
		// The whole-model bounds contain every primitive and cost no per-vertex Haxe work.
		return data.setBounds(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);
	}

	function createMaterials(asset:AnimationAsset):Array<Material> {
		var textures:Array<Null<Texture>> = [];
		var sampler = scene.createSampler();
		scene.setSamplerData(sampler, new SamplerData());
		for (image in 0...asset.imageCount) {
			var width = asset.imageWidth(image);
			if (width == 0) {
				textures.push(null);
				continue;
			}
			var sceneImage = scene.createImage();
			scene.setImageData(sceneImage, new ImageData(width, asset.imageHeight(image)).setPixels(asset.readImage(image)));
			var texture = scene.createTexture();
			scene.setTextureData(texture, new TextureData(sceneImage));
			textures.push(texture);
		}
		var data:Array<MaterialData> = [];
		for (source in asset.materials) {
			var color = source.baseColor;
			var material = new MaterialData()
				.setBaseColor(color[0], color[1], color[2], color[3])
				.setMetallic(source.metallic)
				.setRoughness(source.roughness)
				.setEmissive(source.emissive[0], source.emissive[1], source.emissive[2])
				.setAlphaMode(source.alphaMode)
				.setDoubleSided(source.doubleSided);
			if (source.alphaMode == ALPHA_MASK)
				material.setAlphaCutoff(source.alphaCutoff);
			if (source.alphaMode == ALPHA_BLEND)
				material.setOpaque(false);
			if (source.baseColorImage >= 0 && textures[source.baseColorImage] != null)
				material.setBaseColorTexture(textures[source.baseColorImage], sampler);
			data.push(material);
		}
		return data.length == 0 ? [] : scene.createMaterialBatch(data);
	}
}
