package nativekit.scene;

import NativeKitScene;
import NativeKitSceneRender;

/** Immutable-at-render-boundary view configuration with typed overrides. */
class SceneView {
	final value:nkscene_render_view;
	var visibilityOverrides:Array<nkscene_render_visibility_override> = [];
	var materialOverrides:Array<nkscene_render_material_override> = [];
	var selectionOverrides:Array<nkscene_render_material_override> = [];
	var hoverOverrides:Array<nkscene_render_material_override> = [];
	var isolatedSources:Array<nkscene_entity_id> = [];
	var isolatedNodes:Array<nkscene_node_id> = [];
	var sourceVisibilityOverrides:Array<nkscene_render_source_visibility_override> = [];
	var sourceMaterialOverrides:Array<nkscene_render_source_material_override> = [];
	var clipPlanes:Array<nkscene_render_clip_plane> = [];
	var poseOverrides:Array<nkscene_render_pose_override> = [];

	public function new() {
		value = new nkscene_render_view();
		value.set_struct_size(nkscene_render_view.size());
	}

	public function setRoot(root:NodeId):SceneView {
		value.set_root(root.nativeValue());
		return this;
	}

	public function clearRoot():SceneView {
		value.set_root(new nkscene_node_id());
		return this;
	}

	public function setIncludeInvisible(include:Bool):SceneView {
		value.set_include_invisible(include ? 1 : 0);
		return this;
	}

	/** Enables bounds culling and applies a column-major world-to-clip matrix. */
	public function setViewProjection(transform:Transform):SceneView {
		var camera = new nkscene_render_camera();
		camera.set_enabled(1);
		camera.set_view_projection(transform.nativeValue());
		value.set_camera(camera);
		value.set_camera_view_pose_enabled(0);
		return this;
	}

	/** Disables bounds culling and restores the default identity projection. */
	public function clearViewProjection():SceneView {
		value.set_camera(new nkscene_render_camera());
		value.set_camera_view_pose_enabled(0);
		return this;
	}

	/** Supplies the exact eye and view direction for an explicit projection. */
	public function setCameraViewPose(position:Array<Float>, viewDirection:Array<Float>,
			orthographic:Bool = false):SceneView {
		if (position.length != 3 || viewDirection.length != 3)
			throw "invalid camera view pose";
		value.set_camera_view_pose_enabled(1);
		for (axis in 0...3) {
			value.set_camera_world_position(axis, position[axis]);
			value.set_camera_view_direction(axis, viewDirection[axis]);
		}
		value.set_camera_orthographic(orthographic ? 1 : 0);
		return this;
	}

	/** Renders an infinite z=0 grid behind scene geometry for editor views.
	 *  Forward, right, and up are world-space ray basis vectors; right/up include
	 *  the camera's field-of-view scale for clip-space coordinates in [-1, 1]. */
	public function setWorkplaneGrid(eye:Array<Float>, forward:Array<Float>,
			right:Array<Float>, up:Array<Float>, spacing:Float, cameraDistance:Float):SceneView {
		if (eye.length != 3 || forward.length != 3 || right.length != 3 || up.length != 3 ||
			spacing <= 0.0) throw "invalid workplane grid data";
		value.set_workplane_grid_enabled(1);
		for (axis in 0...3) {
			value.set_workplane_grid_eye_spacing(axis, eye[axis]);
			value.set_workplane_grid_forward(axis, forward[axis]);
			value.set_workplane_grid_right(axis, right[axis]);
			value.set_workplane_grid_up(axis, up[axis]);
		}
		value.set_workplane_grid_eye_spacing(3, spacing);
		value.set_workplane_grid_forward(3, cameraDistance);
		return this;
	}

	/** Sets three world-space directions, intensities, and optional RGB light colors. */
	public function setStudioLighting(directions:Array<Float>, sky:Array<Float>, ground:Array<Float>,
			?colors:Array<Float>):SceneView {
		if (directions.length != 12 || sky.length != 3 || ground.length != 3)
			throw "invalid studio lighting data";
		if (colors != null && colors.length != 12) throw "invalid studio light colors";
		value.set_studio_lighting_enabled(1);
		for (index in 0...12) value.set_studio_light_directions(index, directions[index]);
		for (index in 0...12) value.set_studio_light_colors(index,
			colors == null ? (index % 4 == 3 ? 0.0 : 1.0) : colors[index]);
		for (index in 0...3) {
			value.set_studio_ambient_sky(index, sky[index]);
			value.set_studio_ambient_ground(index, ground[index]);
		}
		return this;
	}

	/** Selects a scene camera node when no explicit matrix is set. */
	public function setCameraNode(node:NodeId):SceneView {
		value.set_camera_node(node.nativeValue());
		return this;
	}

	public function clearCameraNode():SceneView {
		value.set_camera_node(new nkscene_node_id());
		return this;
	}

	/** Adds a conservative node-level section plane. Kept side is normal * p + distance >= 0. */
	public function addClipPlane(normalX:Float, normalY:Float, normalZ:Float,
			distance:Float, enabled:Bool = true):SceneView {
		var plane = new nkscene_render_clip_plane();
		plane.set_normal(0, normalX);
		plane.set_normal(1, normalY);
		plane.set_normal(2, normalZ);
		plane.set_distance(distance);
		plane.set_enabled(enabled ? 1 : 0);
		clipPlanes.push(plane);
		value.set_clip_planes(clipPlanes);
		return this;
	}

	public function clearClipPlanes():SceneView {
		clipPlanes.resize(0);
		value.set_clip_planes(clipPlanes);
		return this;
	}

	public function setVisibility(node:NodeId, visible:Bool):SceneView {
		var stable = node.stableValue();
		for (override in visibilityOverrides) {
			if (override.get_node().get_value() == stable) {
				override.set_visible(visible ? 1 : 0);
				value.set_visibility_overrides(visibilityOverrides);
				return this;
			}
		}
		var override = new nkscene_render_visibility_override();
		override.set_node(node.nativeValue());
		override.set_visible(visible ? 1 : 0);
		visibilityOverrides.push(override);
		value.set_visibility_overrides(visibilityOverrides);
		return this;
	}

	public function setNodeVisibility(node:NodeId, visible:Bool):SceneView
		return setVisibility(node, visible);

	public function setMaterial(node:NodeId, material:Material):SceneView {
		setMaterialOverride(materialOverrides, node, material);
		value.set_material_overrides(materialOverrides);
		return this;
	}

	public function setNodeMaterial(node:NodeId, material:Material):SceneView
		return setMaterial(node, material);

	/** Adds or replaces a runtime world-space pose without changing the scene snapshot. */
	public function setPose(node:NodeId, transform:Transform):SceneView {
		for (override in poseOverrides) {
			if (override.get_node().get_value() == node.stableValue()) {
				override.set_world_transform(transform.nativeValue());
				value.set_pose_overrides(poseOverrides);
				value.set_pose_override_count(poseOverrides.length);
				return this;
			}
		}
		var override = new nkscene_render_pose_override();
		override.set_node(node.nativeValue());
		override.set_world_transform(transform.nativeValue());
		poseOverrides.push(override);
		value.set_pose_overrides(poseOverrides);
		value.set_pose_override_count(poseOverrides.length);
		return this;
	}

	/** Replaces all world-space poses with one native publication. */
	public function replacePoses(nodes:Array<NodeId>, transforms:Array<Transform>):SceneView {
		if (nodes.length != transforms.length) throw "Pose node/transform count mismatch";
		var replacements:Array<nkscene_render_pose_override> = [];
		for (index in 0...nodes.length) {
			var override = new nkscene_render_pose_override();
			override.set_node(nodes[index].nativeValue());
			override.set_world_transform(transforms[index].nativeValue());
			replacements.push(override);
		}
		poseOverrides = replacements;
		value.set_pose_overrides(poseOverrides);
		value.set_pose_override_count(poseOverrides.length);
		return this;
	}

	public function clearPoses():SceneView {
		poseOverrides.resize(0);
		value.set_pose_overrides(poseOverrides);
		value.set_pose_override_count(0);
		return this;
	}

	@:allow(SelectionSet)
	function setSelectionMaterial(node:NodeId, material:Material):SceneView {
		setMaterialOverride(selectionOverrides, node, material);
		value.set_selection_overrides(selectionOverrides);
		return this;
	}

	@:allow(SceneInteraction)
	function setHoverMaterial(node:NodeId, material:Material):SceneView {
		setMaterialOverride(hoverOverrides, node, material);
		value.set_hover_overrides(hoverOverrides);
		return this;
	}

	/** Adds or replaces a source-level visibility rule. */
	public function setSourceVisibility(source:haxe.Int64, visible:Bool):SceneView {
		for (override in sourceVisibilityOverrides) {
			if (override.get_source().get_value() == source) {
				override.set_visible(visible ? 1 : 0);
				value.set_source_visibility_overrides(sourceVisibilityOverrides);
				return this;
			}
		}
		var override = new nkscene_render_source_visibility_override();
		var sourceValue = new nkscene_entity_id();
		sourceValue.set_value(source);
		override.set_source(sourceValue);
		override.set_visible(visible ? 1 : 0);
		sourceVisibilityOverrides.push(override);
		value.set_source_visibility_overrides(sourceVisibilityOverrides);
		return this;
	}

	/** Adds or replaces a source-level base material rule. */
	public function setSourceMaterial(source:haxe.Int64, material:Material):SceneView {
		for (override in sourceMaterialOverrides) {
			if (override.get_source().get_value() == source) {
				override.set_material(material.id());
				value.set_source_material_overrides(sourceMaterialOverrides);
				return this;
			}
		}
		var override = new nkscene_render_source_material_override();
		var sourceValue = new nkscene_entity_id();
		sourceValue.set_value(source);
		override.set_source(sourceValue);
		override.set_material(material.id());
		sourceMaterialOverrides.push(override);
		value.set_source_material_overrides(sourceMaterialOverrides);
		return this;
	}

	/** Adds or removes a source from the declarative isolation set. */
	public function setIsolatedSource(source:haxe.Int64, isolated:Bool):SceneView {
		for (index in 0...isolatedSources.length) {
			if (isolatedSources[index].get_value() == source) {
				if (!isolated)
					isolatedSources.splice(index, 1);
				value.set_isolated_sources(isolatedSources);
				return this;
			}
		}
		if (isolated) {
			var sourceValue = new nkscene_entity_id();
			sourceValue.set_value(source);
			isolatedSources.push(sourceValue);
		}
		value.set_isolated_sources(isolatedSources);
		return this;
	}

	/** Adds or removes an explicit node from the isolation set. */
	public function setIsolatedNode(node:NodeId,
			isolated:Bool):SceneView {
		var stable = node.stableValue();
		for (index in 0...isolatedNodes.length) {
			if (isolatedNodes[index].get_value() == stable) {
				if (!isolated)
					isolatedNodes.splice(index, 1);
				value.set_isolated_nodes(isolatedNodes);
				return this;
			}
		}
		if (isolated)
			isolatedNodes.push(node.nativeValue());
		value.set_isolated_nodes(isolatedNodes);
		return this;
	}

	/** Clears source and explicit node isolation while preserving rules. */
	public function clearIsolation():SceneView {
		isolatedSources.resize(0);
		isolatedNodes.resize(0);
		value.set_isolated_sources(isolatedSources);
		value.set_isolated_nodes(isolatedNodes);
		return this;
	}

	public function clearSourceFilter():SceneView {
		isolatedSources.resize(0);
		sourceVisibilityOverrides.resize(0);
		sourceMaterialOverrides.resize(0);
		value.set_isolated_sources(isolatedSources);
		value.set_source_visibility_overrides(sourceVisibilityOverrides);
		value.set_source_material_overrides(sourceMaterialOverrides);
		return this;
	}

	public function clearSelectionOverrides():SceneView {
		selectionOverrides.resize(0);
		value.set_selection_overrides(selectionOverrides);
		return this;
	}

	public function clearHoverOverrides():SceneView {
		hoverOverrides.resize(0);
		value.set_hover_overrides(hoverOverrides);
		return this;
	}

	public function clearOverrides():SceneView {
		visibilityOverrides.resize(0);
		materialOverrides.resize(0);
		selectionOverrides.resize(0);
		hoverOverrides.resize(0);
		isolatedSources.resize(0);
		isolatedNodes.resize(0);
		sourceVisibilityOverrides.resize(0);
		sourceMaterialOverrides.resize(0);
		value.set_visibility_overrides(visibilityOverrides);
		value.set_material_overrides(materialOverrides);
		value.set_selection_overrides(selectionOverrides);
		value.set_hover_overrides(hoverOverrides);
		value.set_isolated_sources(isolatedSources);
		value.set_isolated_nodes(isolatedNodes);
		value.set_source_visibility_overrides(sourceVisibilityOverrides);
		value.set_source_material_overrides(sourceMaterialOverrides);
		return this;
	}

	public function applyVisibilityFilter(filter:VisibilityFilter):SceneView {
		filter.apply(this);
		return this;
	}

	/** Applies reusable base visibility and material presentation rules. */
	public function applyPolicy(policy:SceneViewPolicy):SceneView
		return policy.apply(this);

	public function applySelection(selection:SelectionSet, highlight:Material):SceneView {
		clearSelectionOverrides();
		selection.apply(this, highlight);
		return this;
	}

	/** Replaces the hover layer; hover takes precedence over selection. */
	public function applyHover(node:Null<NodeId>, highlight:Material):SceneView {
		clearHoverOverrides();
		if (node != null)
			setHoverMaterial(node, highlight);
		return this;
	}

	public function visibilityOverrideValues():Array<nkscene_render_visibility_override>
		return visibilityOverrides.copy();

	public function materialOverrideValues():Array<nkscene_render_material_override>
		return materialOverrides.copy();

	public function selectionOverrideValues():Array<nkscene_render_material_override>
		return selectionOverrides.copy();

	public function hoverOverrideValues():Array<nkscene_render_material_override>
		return hoverOverrides.copy();

	public function visibilityOverrideCount():Int
		return visibilityOverrides.length;

	public function materialOverrideCount():Int
		return materialOverrides.length + selectionOverrides.length + hoverOverrides.length;

	public function selectionOverrideCount():Int
		return selectionOverrides.length;

	public function hoverOverrideCount():Int
		return hoverOverrides.length;

	public function isolatedSourceCount():Int
		return isolatedSources.length;

	public function isolatedNodeCount():Int
		return isolatedNodes.length;

	public function sourceVisibilityOverrideCount():Int
		return sourceVisibilityOverrides.length;

	public function sourceMaterialOverrideCount():Int
		return sourceMaterialOverrides.length;

	public function clipPlaneCount():Int
		return clipPlanes.length;

	@:allow(SceneRenderer, SpatialIndex)
	function nativeValue():nkscene_render_view
		return value;

	function setMaterialOverride(overrides:Array<nkscene_render_material_override>,
			node:NodeId, material:Material):Void {
		var stable = node.stableValue(), materialValue = material.id();
		for (override in overrides) {
			if (override.get_node().get_value() == stable) {
				override.set_material(materialValue);
				return;
			}
		}
		var override = new nkscene_render_material_override();
		override.set_node(node.nativeValue());
		override.set_material(materialValue);
		overrides.push(override);
	}
}
