import haxe.Json;
import cadkit.MeshFeatureEdges;
import haxe.io.Bytes;
import haxeon.editor.ViewportLook;
import machinekit.catalog.CatalogMetadata;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentType;
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
			if (!FileSystem.exists(ENVIRONMENTS))
				FileSystem.createDirectory(ENVIRONMENTS);
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
			var exported = glb(component);
			model = exported.bytes;
			entry.model = modelName(type.id);
			entry.view = view(exported);
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

	/** The default part at preview detail as a binary glTF, in metres, coloured by its material, with its feature edges. */
	static function glb(component:MachineComponent):Glb {
		var part = component.geometry(ComponentDetail.Preview);
		var mesh = part.shape.tessellateRelative();
		var features = MeshFeatureEdges.ofMesh(mesh), edges:Array<Float> = [];
		for (segment in 0...features.length)
			if (features[segment])
				for (value in 0...6)
					edges.push(mesh.edgeSegments.getDouble(segment * 48 + value * 8));
		var material = MaterialLibrary.require(component.materialId).visual;
		// Material colours are sRGB, as the editor's shader reads them; glTF factors are linear. The editor's
		// display roughness floor applies too, so highlights spread the same way.
		return Glb.write(mesh.vertices, mesh.normals, mesh.indices, mesh.vertexCount, mesh.indexCount, edges, 0.001,
			[for (channel in material.baseColor) srgbToLinear(channel)], material.metallic,
			Math.max(material.roughness, look().roughnessFloor));
	}

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
