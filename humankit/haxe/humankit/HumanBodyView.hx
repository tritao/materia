package humankit;

import haxe.io.Bytes;
import nativekit.scene.GeometryData;
import nativekit.scene.MaterialData;
import nativekit.scene.NodeId;
import nativekit.scene.Scene;
import nativekit.scene.Transform;
import humankit.rig.HumanBone;
import humankit.rig.HumanPose;
import humankit.rig.HumanoidRig;
import humankit.rig.Mat4;

/**
 * Draws a human's collision capsules or skeleton beneath its root node, for
 * checking a body proxy against the mesh. Bones are rigid, so every part keeps
 * one geometry and only its node moves; update() places them from a pose in
 * the character's model space.
 */
class HumanBodyView {
	static inline var BONE_THICKNESS:Float = 0.02;
	static inline var SEGMENTS:Int = 12;
	static inline var CAP_RINGS:Int = 4;

	final scene:Scene;
	final proxy:HumanBodyProxy;
	final capsuleNodes:Array<NodeId> = [];
	final boneNodes:Array<NodeId> = [];
	final bones:Array<Array<HumanBone>> = [];
	var display:HumanDisplay = Mesh;

	/** Builds hidden capsule and skeleton nodes beneath parent (the character root). */
	public function new(scene:Scene, parent:NodeId, proxy:HumanBodyProxy, rest:HumanPose) {
		this.scene = scene;
		this.proxy = proxy;
		for (pair in skeletonPairs(rest.rig))
			bones.push(pair);
		var capsuleMaterial = scene.createMaterial();
		scene.setMaterialData(capsuleMaterial, MaterialData.opaque(0.95, 0.55, 0.15).setRoughness(0.6));
		var boneMaterial = scene.createMaterial();
		scene.setMaterialData(boneMaterial, MaterialData.opaque(0.2, 0.75, 0.95).setRoughness(0.5));
		var shapes = [for (capsule in proxy.capsules) capsuleMesh(capsule.radius, capsule.length)].concat([
			for (pair in bones)
				GeometryData.box(BONE_THICKNESS, BONE_THICKNESS, Math.max(distance(rest, pair[0], pair[1]), 1e-3))
		]);
		var geometries = scene.createGeometryBatch(shapes);
		var transaction = scene.beginTransaction();
		for (index in 0...geometries.length) {
			var node = transaction.createNode();
			var isCapsule = index < proxy.capsules.length;
			transaction.setParent(node, parent);
			transaction.setName(node, isCapsule ? 'Capsule (${proxy.capsules[index].name})' :
				'Bone (${bones[index - proxy.capsules.length].join(" to ")})');
			transaction.setGeometry(node, geometries[index]);
			transaction.setMaterial(node, isCapsule ? capsuleMaterial : boneMaterial);
			transaction.setVisibility(node, false);
			(isCapsule ? capsuleNodes : boneNodes).push(node);
		}
		transaction.commit();
		update(rest);
	}

	/** Shows the capsules, the skeleton, or neither (the mesh alone). */
	public function show(mode:HumanDisplay):Void {
		display = mode;
		var transaction = scene.beginTransaction();
		for (node in capsuleNodes)
			transaction.setVisibility(node, mode == Capsules);
		for (node in boneNodes)
			transaction.setVisibility(node, mode == Skeleton);
		transaction.commit();
	}

	/** Places every part from a pose in model space. */
	public function update(pose:HumanPose):Void {
		var transaction = scene.beginTransaction();
		var placements = proxy.place(pose);
		for (index in 0...capsuleNodes.length)
			transaction.setTransform(capsuleNodes[index], transform(placements[index].center, placements[index].rotation));
		for (index in 0...boneNodes.length) {
			var from = pose.bonePosition(bones[index][0]), to = pose.bonePosition(bones[index][1]);
			if (from == null || to == null)
				continue;
			var center = [(from[0] + to[0]) * 0.5, (from[1] + to[1]) * 0.5, (from[2] + to[2]) * 0.5];
			transaction.setTransform(boneNodes[index],
				transform(center, HumanBodyProxy.rotationFromZ(Mat4.normalize(Mat4.subtract(to, from)))));
		}
		transaction.commit();
	}

	public function nodes():Array<NodeId>
		return capsuleNodes.concat(boneNodes);

	/** Parent-to-child bone pairs the rig provides, from the pelvis outwards. */
	static function skeletonPairs(rig:HumanoidRig):Array<Array<HumanBone>> {
		var pairs:Array<Array<HumanBone>> = [];
		var spine = [for (bone in [Pelvis, Spine, Spine2, Chest, Neck, Head]) if (rig.has(bone)) bone];
		for (index in 1...spine.length)
			pairs.push([spine[index - 1], spine[index]]);
		for (chain in [
			[Chest, UpperArmL, ForearmL, HandL, MiddleL],
			[Chest, UpperArmR, ForearmR, HandR, MiddleR],
			[Pelvis, ThighL, ShinL, FootL, ToeTipL],
			[Pelvis, ThighR, ShinR, FootR, ToeTipR]
		]) {
			var present = [for (bone in chain) if (rig.has(bone)) bone];
			for (index in 1...present.length)
				pairs.push([present[index - 1], present[index]]);
		}
		return pairs;
	}

	static function transform(center:Array<Float>, q:Array<Float>):Transform {
		var x = q[0], y = q[1], z = q[2], w = q[3];
		var m = [
			1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y), 0.0,
			2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x), 0.0,
			2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y), 0.0,
			center[0], center[1], center[2], 1.0
		];
		var result = Transform.identity();
		for (index in 0...16)
			result.set(index, m[index]);
		return result;
	}

	static function distance(pose:HumanPose, a:HumanBone, b:HumanBone):Float {
		var from = pose.bonePosition(a), to = pose.bonePosition(b);
		if (from == null || to == null)
			return 0.0;
		var delta = Mat4.subtract(to, from);
		return Math.sqrt(Mat4.dot(delta, delta));
	}

	/** A capsule along local Z: two hemispheres of CAP_RINGS rings joined by a cylinder. */
	static function capsuleMesh(radius:Float, length:Float):GeometryData {
		var rings = CAP_RINGS * 2 + 2;
		var columns = SEGMENTS + 1;
		var positions = Bytes.alloc(rings * columns * 12), normals = Bytes.alloc(rings * columns * 12);
		var vertex = 0;
		for (ring in 0...rings) {
			// Rings 0..CAP_RINGS cover the lower hemisphere, the rest the upper.
			var upper = ring > CAP_RINGS;
			var step = upper ? ring - CAP_RINGS - 1 : ring;
			var latitude = (upper ? step : step - CAP_RINGS) * (Math.PI * 0.5 / CAP_RINGS);
			var offset = (upper ? 0.5 : -0.5) * length;
			for (column in 0...columns) {
				var longitude = column * (Math.PI * 2.0 / SEGMENTS);
				var nx = Math.cos(latitude) * Math.cos(longitude), ny = Math.cos(latitude) * Math.sin(longitude),
					nz = Math.sin(latitude);
				positions.setFloat(vertex * 12, nx * radius);
				positions.setFloat(vertex * 12 + 4, ny * radius);
				positions.setFloat(vertex * 12 + 8, nz * radius + offset);
				normals.setFloat(vertex * 12, nx);
				normals.setFloat(vertex * 12 + 4, ny);
				normals.setFloat(vertex * 12 + 8, nz);
				vertex++;
			}
		}
		var indices = Bytes.alloc((rings - 1) * SEGMENTS * 6 * 4);
		var index = 0;
		for (ring in 0...rings - 1)
			for (column in 0...SEGMENTS) {
				var a = ring * columns + column, b = a + 1, c = a + columns, d = c + 1;
				for (value in [a, b, d, a, d, c]) {
					indices.setInt32(index * 4, value);
					index++;
				}
			}
		var half = length * 0.5 + radius;
		return new GeometryData()
			.addStream(1, 2, positions, vertex, 12)
			.addStream(2, 2, normals, vertex, 12)
			.setIndexBuffer(indices, index)
			.setBounds(-radius, -radius, -half, radius, radius, half);
	}
}
