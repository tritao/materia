import haxe.Json;
import cadkit.MeshFeatureEdges;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSurface;
import cadkit.threads.ThreadTexture;
import cadkit.threads.Threads;
import haxe.io.Bytes;
import haxeon.editor.ViewportLook;
import machinekit.catalog.CatalogMetadata;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentType;
import machinekit.component.ComponentView;
import machinekit.component.ComponentValue;
import machinekit.component.MachineComponent;
import machinekit.component.MachineKitComponents;
import materia.project.MaterialLibrary;
import sys.FileSystem;
import sys.io.File;

/**
 * Exports MachineKit's component library for the docs site's component catalogue: every registered recipe with its
 * parameters, catalogue rows and their provenance, connectors, mass and bill-of-materials line, and a GLB of the
 * default part at preview detail. Runs as a workspace test from this project's directory: by default it re-exports
 * and fails if anything differs from the committed `data/components.json` and `../site/public/components/*.glb`;
 * with `DOCS_COMPONENTS_WRITE=1` it rewrites them.
 */
class DocsComponents {
	static inline var DATA = "data/components.json";
	static inline var MODELS = "../site/public/components";
	static inline var ENVIRONMENTS = "../site/public/components/environment";
	static inline var THREADS = "../site/public/components/threads";

	/** Cosmetic thread textures, by file below MODELS, shared by every model with that thread form. */
	static final threadTextures:Map<String, Bytes> = [];

	/** The catalogue's views: one turn about the vertical, and the angles down from it a part can get (see `view`). */
	static inline var THETA = 35.0;

	static final PHIS = [50.0, 62.0, 72.0];

	/** The editor's default lighting, which the catalogue reproduces. A function, not a static initialiser, so it
	 * does not depend on another class's static initialisation order. */
	static function look():ViewportLook
		return ViewportLook.preset(ViewportLook.Studio);

	public static function main():Void {
		var write = Sys.getEnv("DOCS_COMPONENTS_WRITE") == "1";
		var entries:Array<Dynamic> = [], models = new Map<String, Bytes>(), modelCount = 0, failures:Array<String> = [];
		var registry = MachineKitComponents.standard();
		var types = registry.all();
		types.sort((a, b) -> Reflect.compare(a.id, b.id));
		for (type in types) {
			try {
				var exported = export(type, registry);
				entries.push(exported.entry);
				if (exported.model != null) {
					models.set(modelName(type.id), exported.model);
					modelCount++;
				}
			} catch (error:Dynamic) {
				// A recipe whose defaults cannot be built is listed without a part; the catalogue says so.
				failures.push('${type.id}: $error');
				entries.push({id: type.id, label: type.label(), family: family(type.id), category: registry.category(type.id),
					kind: registry.kind(type.id), error: Std.string(error)});
			}
		}
		for (file => bytes in threadTextures)
			models.set(file, bytes);
		for (phi in PHIS)
			models.set('environment/${environmentName(phi)}', StudioEnvironment.hdr(look(), THETA, phi));
		// The viewers' display mapping, from the same look (docs/site/src/model-view.json has the rest).
		var exposure = look().exposure;
		var display = {exposure: exposure > 0 ? exposure : 1.0, toneMapping: exposure > 0 ? "neutral" : "none"};
		var text = Json.stringify({display: display, categories: registry.categoryNames(), components: entries}, null, " ") + "\n";
		var stale = 0;
		if (write) {
			if (!FileSystem.exists("data")) FileSystem.createDirectory("data");
			File.saveContent(DATA, text);
			if (FileSystem.exists(MODELS)) {
				for (name in FileSystem.readDirectory(MODELS))
					if (StringTools.endsWith(name, ".glb") && !models.exists(name))
						FileSystem.deleteFile('$MODELS/$name');
			} else {
				FileSystem.createDirectory(MODELS);
			}
			for (directory in [ENVIRONMENTS, THREADS])
				if (!FileSystem.exists(directory))
					FileSystem.createDirectory(directory);
			for (name => bytes in models)
				File.saveBytes('$MODELS/$name', bytes);
		} else {
			if (!FileSystem.exists(DATA) || File.getContent(DATA) != text) {
				Sys.println('$DATA is stale');
				stale++;
			}
			for (name => bytes in models)
				if (!FileSystem.exists('$MODELS/$name') || File.getBytes('$MODELS/$name').compare(bytes) != 0) {
					Sys.println('$MODELS/$name is stale');
					stale++;
				}
		}
		for (failure in failures)
			Sys.println('no part from defaults: $failure');
		Sys.println('${entries.length} components, $modelCount models' + (write ? " written" : ""));
		if (stale > 0) {
			Sys.println("rerun with DOCS_COMPONENTS_WRITE=1 and review the change");
			Sys.exit(1);
		}
	}

	static function export(type:ComponentType, registry:ComponentRegistry):{entry:Dynamic, model:Null<Bytes>} {
		var component = type.create();
		var parameters = [for (parameter in type.parameters()) describeParameter(parameter)];
		var entry:Dynamic = {
			id: type.id,
			label: type.label(),
			family: family(type.id),
			category: registry.category(type.id),
			kind: registry.kind(type.id),
			designation: component.designation,
			description: component.description,
			parameters: parameters,
			connectors: [for (connector in component.connectors()) {name: connector.name, role: Std.string(connector.role)}],
			bom: {partNumber: component.bom.partNumber, description: component.bom.description, material: component.bom.material},
			material: component.materialId
		};
		var catalogs = catalogRows(type);
		if (catalogs.length > 0)
			entry.catalogs = catalogs;
		try {
			var mass = component.massProperties();
			entry.mass = {kg: round(mass.mass), source: Std.string(mass.source)};
		} catch (_:Dynamic) {}
		var model:Null<Bytes> = null;
		if (component.hasGeometry()) {
			var exported = glb(component, threadTextures);
			model = exported.bytes();
			entry.model = modelName(type.id);
			entry.view = view(exported);
			entry.fidelity = component.geometryFidelity();
		}
		return {entry: entry, model: model};
	}

	static function describeParameter(parameter:ComponentParameter):Dynamic {
		var described:Dynamic = {name: parameter.name, type: typeName(parameter.type), value: valueText(parameter.defaultValue)};
		if (parameter.unit != null) described.unit = parameter.unit;
		if (parameter.minimum != null) described.minimum = parameter.minimum;
		if (parameter.maximum != null) described.maximum = parameter.maximum;
		switch unwrap(parameter.type) {
			case Choice(options): described.options = options;
			case CatalogDesignation(index): described.options = index.designations();
			default:
		}
		return described;
	}

	static function unwrap(type:ComponentParameterType):ComponentParameterType
		return switch type {
			case Optional(inner): unwrap(inner);
			default: type;
		};

	static function typeName(type:ComponentParameterType):String
		return switch type {
			case Scalar: "number";
			case Length: "length";
			case Angle: "angle";
			case Count: "count";
			case Bool: "yes/no";
			case Text: "text";
			case Choice(_): "choice";
			case CatalogDesignation(_): "catalogue";
			case Optional(inner): typeName(inner) + " (optional)";
		};

	static function valueText(value:ComponentValue):Null<Dynamic>
		return switch value {
			case Number(number): round(number);
			case Integer(integer): integer;
			case Boolean(flag): flag;
			case Token(token): token;
			case Unset: null;
		};

	/** Every catalogue row a recipe offers, with where its data comes from and how far it was checked. */
	static function catalogRows(type:ComponentType):Array<Dynamic> {
		var catalogs:Array<Dynamic> = [];
		for (parameter in type.parameters())
			switch unwrap(parameter.type) {
				case CatalogDesignation(index):
					catalogs.push({
						parameter: parameter.name,
						rows: [
							for (designation in index.designations()) {
								var metadata:CatalogMetadata = index.metadata(designation);
								var row:Dynamic = {
									designation: designation,
									source: metadata.source,
									dimensions: Std.string(metadata.dimensionKind),
									conformance: Std.string(metadata.conformance)
								};
								if (metadata.standard != null) row.standard = metadata.standard;
								if (metadata.verifiedFields != null) row.verified = metadata.verifiedFields;
								row;
							}
						]
					});
				default:
			}
		return catalogs;
	}

	static function family(id:String):String {
		var parts = id.split(".");
		return parts.length > 2 ? parts[1] : "other";
	}

	static function modelName(id:String):String
		return id.split(".").slice(1).join("-") + ".glb";

	/**
	 * The default part at preview detail as a binary glTF, in metres, coloured by its material, with its feature
	 * edges, turned to its presentation view. Threaded faces carry the editor's cosmetic thread texture: each is
	 * replaced by CadKit's ThreadSurface over its own triangles (textured inside the thread's span, plain outside),
	 * and the textures are written once per thread form into `threadTextures`.
	 */
	static function glb(component:MachineComponent, threadTextures:Map<String, Bytes>):Glb {
		var part = component.geometry(ComponentDetail.Preview);
		var mesh = part.shape.tessellateRelative();
		var threads = Threads.renderData(part.shape);
		part.close();
		var features = MeshFeatureEdges.ofMesh(mesh), edges:Array<Float> = [];
		for (segment in 0...features.length)
			if (features[segment])
				for (value in 0...6)
					edges.push(mesh.edgeSegments.getDouble(segment * 48 + value * 8));
		var points:Array<Float> = [], normals:Array<Float> = [];
		for (index in 0...mesh.vertexCount * 3) {
			points.push(mesh.vertices.getDouble(index * 8));
			normals.push(mesh.normals.getDouble(index * 8));
		}
		// The part turned to its presentation view: declared by the component, or chosen from its bounds.
		var minimum = [Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY, Math.POSITIVE_INFINITY];
		var maximum = [Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY, Math.NEGATIVE_INFINITY];
		for (index in 0...points.length) {
			var axis = index % 3;
			if (points[index] < minimum[axis]) minimum[axis] = points[index];
			if (points[index] > maximum[axis]) maximum[axis] = points[index];
		}
		var view = component.presentationView();
		if (view == null) view = ComponentView.derive(minimum, maximum);
		var u = view.up, f = view.front;
		// Rows: the local directions that become glTF's +X, +Y (up) and +Z (towards the viewer); millimetres to metres.
		var axes = [[u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2], u[0] * f[1] - u[1] * f[0]], u, f];
		function turn(values:Array<Float>, scale:Float):Array<Float>
			return [
				for (index in 0...values.length) {
					var base = index - index % 3, row = axes[index % 3];
					(row[0] * values[base] + row[1] * values[base + 1] + row[2] * values[base + 2]) * scale;
				}
			];
		var visual = MaterialLibrary.require(component.materialId).visual;
		// Material colours are sRGB, as the editor's shader reads them; glTF factors are linear. The editor's
		// display roughness floor applies too, so highlights spread the same way.
		var color = [for (channel in visual.baseColor) srgbToLinear(channel)];
		var roughness = Math.max(visual.roughness, look().roughnessFloor);
		var model = new Glb();
		var surface = model.material(color, visual.metallic, roughness);
		var threaded:Map<Int, Bool> = [];
		for (thread in threads)
			threaded.set(thread.faceIndex, true);
		var baseIndices:Array<Int> = [], outsidePoints:Array<Float> = [], outsideNormals:Array<Float> = [];
		for (range in mesh.faceRanges)
			if (!threaded.exists(range.faceIndex))
				for (index in range.firstIndex...range.firstIndex + range.indexCount)
					baseIndices.push(mesh.indices.getInt32(index * 4));
		for (thread in threads) {
			var faceTriangles:Array<Int> = [];
			for (range in mesh.faceRanges)
				if (range.faceIndex == thread.faceIndex)
					for (index in range.firstIndex...range.firstIndex + range.indexCount)
						faceTriangles.push(mesh.indices.getInt32(index * 4));
			var frame = new Plane(new Vector(thread.origin[0], thread.origin[1], thread.origin[2]),
				new Vector(thread.xDirection[0], thread.xDirection[1], thread.xDirection[2]),
				new Vector(thread.direction[0], thread.direction[1], thread.direction[2]));
			var unwrapped = ThreadSurface.unwrap(thread.faceName, thread.spec, frame, thread.length, thread.spec.pitch,
				thread.ringAtStart, thread.ringAtEnd, points, normals, faceTriangles);
			outsidePoints = outsidePoints.concat(unwrapped.outsidePositions);
			outsideNormals = outsideNormals.concat(unwrapped.outsideNormals);
			for (value in unwrapped.rings) edges.push(value);
			var name = threadTextureName(thread.spec);
			for (normal in [false, true]) {
				var file = 'threads/$name-${normal ? "normal" : "shade"}.png';
				if (!threadTextures.exists(file))
					threadTextures.set(file, Png.rgba(ThreadTexture.SIZE, ThreadTexture.SIZE,
						ThreadTexture.pixels(thread.spec, normal).sub(0, ThreadTexture.SIZE * ThreadTexture.SIZE * 4)));
			}
			var material = model.material(color, visual.metallic, roughness, 'threads/$name-shade.png', 'threads/$name-normal.png');
			model.triangles(turn(unwrapped.positions, 0.001), turn(unwrapped.normals, 1), unwrapped.uvs, unwrapped.triangles, material);
		}
		model.triangles(turn(points, 0.001), turn(normals, 1), null, baseIndices, surface);
		model.triangles(turn(outsidePoints, 0.001), turn(outsideNormals, 1), null, [for (index in 0...Std.int(outsidePoints.length / 3)) index],
			surface);
		// Edges contrast with the part, as in the editor: dark on light surfaces, light on dark ones.
		var edge = [for (channel in ViewportLook.edgeColor(visual.baseColor)) srgbToLinear(channel)];
		model.lines(turn(edges, 0.001), model.unlitMaterial([edge[0], edge[1], edge[2], 1.0]));
		return model;
	}

	/** One texture pair per thread form: profile, size, pitch, starts and hand. */
	static function threadTextureName(spec:materia.project.ThreadSpecData):String
		return ('${spec.profile}-${spec.majorDiameter}-${spec.pitch}-${spec.starts}-${spec.hand}').toLowerCase().split(".").join("_");

	static function srgbToLinear(value:Float):Float {
		return value <= 0.04045 ? value / 12.92 : Math.pow((value + 0.055) / 1.055, 2.4);
	}

	/**
	 * The angle the catalogue shows a part from, used by both its thumbnail and its 3D viewer: the camera's turn about
	 * the vertical and its angle down from the vertical, in degrees. Flat parts are seen from higher up, so a plate
	 * shows its face rather than its edge; tall ones from closer to level.
	 */
	static function view(model:Glb):{theta:Float, phi:Float, environment:String} {
		var width = model.max[0] - model.min[0], height = model.max[1] - model.min[1], depth = model.max[2] - model.min[2];
		var footprint = Math.max(width, depth);
		var phi = height < 0.35 * footprint ? PHIS[0] : height > 2.5 * footprint ? PHIS[2] : PHIS[1];
		return {theta: THETA, phi: phi, environment: '/components/environment/${environmentName(phi)}'};
	}

	static function environmentName(phi:Float):String
		return 'studio-${Std.int(phi)}.hdr';

	/** Six decimals; through `fround` (a Float), since `Math.round` returns an Int that large values overflow. */
	static function round(value:Float):Float
		return Math.fround(value * 1e6) / 1e6;
}
