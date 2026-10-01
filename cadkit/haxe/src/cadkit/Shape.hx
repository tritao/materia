package cadkit;

import CadKit;
import cadkit.Geometry;

/** Owning, headless CAD shape façade for Haxeon applications. */
class Shape {
	private var native:CadKit.OwnedShapeHandle;
	private var meshCache:Array<{linearDeflection:Float, angularDeflection:Float, mesh:Mesh}>;
	/** A shape never changes, so its names are read across the ABI once per kind. */
	private var faceNames:Null<Array<String>>;
	private var edgeNames:Null<Array<String>>;
	private var vertexNames:Null<Array<String>>;
	private var solidNames:Null<Array<String>>;
	private var faceAliases:Null<Array<Array<String>>>;
	private var solidAliases:Null<Array<Array<String>>>;

	private function new(native:CadKit.OwnedShapeHandle) {
		this.native = native;
		meshCache = [];
		faceNames = null;
		edgeNames = null;
		vertexNames = null;
		solidNames = null;
		faceAliases = null;
		solidAliases = null;
	}

	public static function fromOwnedHandle(native:CadKit.OwnedShapeHandle):Shape {
		return new Shape(native);
	}

	public static function box(width:Float, depth:Float, height:Float):Shape {
		return new Shape(CadKit.boxChecked(width, depth, height));
	}

	public static function cylinder(radius:Float, height:Float):Shape {
		return new Shape(CadKit.cylinderChecked(radius, height));
	}

	public static function sphere(radius:Float):Shape {
		return new Shape(CadKit.sphereChecked(radius));
	}

	public static function importStep(path:String):Shape {
		return new Shape(CadKit.stepImportChecked(path));
	}

	/** Import an authored STEP text payload without relying on its original file path. */
	public static function importStepText(text:String):Shape {
		return new Shape(CadKit.stepImportTextChecked(text));
	}

	public function exportStep(path:String):Void {
		CadKit.stepExportChecked(native.borrow(), path);
	}

	public function cloneShape():Shape {
		return new Shape(CadKit.shapeCloneChecked(native.borrow()));
	}

	/** Borrow the native handle for a synchronous bulk ABI call. */
	public function borrowHandle():CadKit.ShapeHandle {
		return native.borrow();
	}

	public function bounds():CadKit.Bounds {
		return CadKit.shapeBoundsChecked(native.borrow());
	}

	public function area():Float {
		return CadKit.shapeAreaChecked(native.borrow());
	}

	public function volume():Float {
		return CadKit.shapeVolumeChecked(native.borrow());
	}

	/** Compute volume, area, centroid, and unit-density centroidal inertia. */
	public function massProperties():PhysicalProperties {
		var nativeProperties = CadKit.shapePhysicalPropertiesChecked(native.borrow());
		var tensor = nativeProperties.get_inertia();
		return new PhysicalProperties(nativeProperties.get_volume(), nativeProperties.get_surfaceArea(),
			cadkit.modeling.Vector.fromNative(nativeProperties.get_centerOfMass()),
			new cadkit.InertiaTensor(tensor.get_xx(), tensor.get_xy(), tensor.get_xz(), tensor.get_yy(),
				tensor.get_yz(), tensor.get_zz()));
	}

	public function kind():CadKit.ShapeKind {
		return CadKit.shapeKindGetChecked(native.borrow());
	}

	public function subshapeCount(kind:CadKit.ShapeKind):Int {
		return CadKit.shapeSubshapeCountChecked(native.borrow(), kind);
	}

	public function subshape(kind:CadKit.ShapeKind, index:Int):Shape {
		return new Shape(CadKit.shapeSubshapeAtChecked(native.borrow(), kind, index));
	}

	public function faces():FaceCollection {
		return new FaceCollection(this);
	}

	public function edges():EdgeCollection {
		return new EdgeCollection(this);
	}

	public function vertices():VertexCollection {
		return new VertexCollection(this);
	}

	/**
		The topological names of this shape's faces, edges or vertices, indexed like `subshape(kind, index)`
		(plans/TOPOLOGICAL_NAMING.md). Names are opaque text that survives parametric edits.
	**/
	public function elementNames(kind:CadKit.ShapeKind):Array<String> {
		var cached = kind == CadKit.ShapeKind.Face ? faceNames : kind == CadKit.ShapeKind.Edge ? edgeNames
			: kind == CadKit.ShapeKind.Vertex ? vertexNames : kind == CadKit.ShapeKind.Solid ? solidNames : null;
		if (cached == null) {
			if (subshapeCount(kind) == 0) {
				cached = [];
			} else {
				var bytes = CadKit.shapeCopyElementNamesBytesChecked(native.borrow(), kind);
				cached = bytes.getString(0, bytes.length).split("\n");
			}
			if (kind == CadKit.ShapeKind.Face)
				faceNames = cached;
			else if (kind == CadKit.ShapeKind.Edge)
				edgeNames = cached;
			else if (kind == CadKit.ShapeKind.Vertex)
				vertexNames = cached;
			else if (kind == CadKit.ShapeKind.Solid)
				solidNames = cached;
		}
		return cached.copy();
	}

	public function elementName(kind:CadKit.ShapeKind, index:Int):String {
		var names = elementNames(kind);
		if (index < 0 || index >= names.length)
			throw "element index is out of range";
		return names[index];
	}

	/**
		Other names each face or solid also answers to, indexed like `elementNames`: a merge keeps the smallest name
		and the others as aliases. Edges and vertices have none.
	**/
	public function elementAliases(kind:CadKit.ShapeKind):Array<Array<String>> {
		var cached = kind == CadKit.ShapeKind.Face ? faceAliases : kind == CadKit.ShapeKind.Solid ? solidAliases : null;
		if (cached == null) {
			cached = readAliases(kind);
			if (kind == CadKit.ShapeKind.Face)
				faceAliases = cached;
			else if (kind == CadKit.ShapeKind.Solid)
				solidAliases = cached;
		}
		var known:Array<Array<String>> = cast cached;
		return [for (aliases in known) aliases.copy()];
	}

	function readAliases(kind:CadKit.ShapeKind):Array<Array<String>> {
		var result:Array<Array<String>> = [for (_ in 0...subshapeCount(kind)) []];
		if (kind != CadKit.ShapeKind.Face && kind != CadKit.ShapeKind.Solid)
			return result;
		var bytes = CadKit.shapeCopyElementAliasesBytesChecked(native.borrow(), kind);
		if (bytes.length == 0)
			return result;
		for (line in bytes.getString(0, bytes.length).split("\n")) {
			var tab = line.indexOf("\t");
			if (tab <= 0)
				continue;
			var index = Std.parseInt(line.substr(0, tab));
			if (index != null && index >= 0 && index < result.length)
				result[index].push(line.substr(tab + 1));
		}
		return result;
	}

	/**
		A copy whose faces, edges or vertices are named by `ids` (one per subshape; "" keeps the current name).
		Each id is escaped into a name. Edge and vertex names hold where faces cannot name them: boundary and
		wire edges and their vertices.
	**/
	public function withElementNames(kind:CadKit.ShapeKind, ids:Array<String>):Shape {
		return new Shape(CadKit.shapeSeedNamesChecked(native.borrow(), kind, ids.join("\n")));
	}

	/** A copy whose names that no input has are prefixed by `tag:`: what was created from the inputs. */
	public function stamped(tag:String, inputs:Array<Shape>):Shape {
		var refs:Array<CadKit.ShapeRef> = [];
		for (input in inputs) {
			var ref = new CadKit.ShapeRef();
			ref.set_shape(input.borrowHandle());
			refs.push(ref);
		}
		return new Shape(CadKit.shapeStampNamesChecked(native.borrow(), tag, refs));
	}

	/**
		`copy` with every name prefixed by `tag:`, for one instance among copies of a shape (a pattern's `i2.0`, a
		mirror's `m`), so the copies stay distinguishable. Takes ownership of `copy`.
	**/
	public static function instance(copy:Shape, tag:String):Shape {
		try {
			var result = copy.stamped(tag, []);
			copy.close();
			return result;
		} catch (error:Dynamic) {
			copy.close();
			throw error;
		}
	}

	public static function namingScheme():Int {
		return CadKit.namingSchemeVersionChecked();
	}

	public function sameAs(other:Shape):Bool {
		return CadKit.shapeIsSameChecked(native.borrow(), other.native.borrow()) != 0;
	}

	public function surfaceKind():CadKit.SurfaceKind {
		return CadKit.faceSurfaceKindChecked(native.borrow());
	}

	public function center():CadKit.Vec3 {
		return CadKit.faceCenterChecked(native.borrow());
	}

	public function faceArea():Float {
		return CadKit.faceAreaChecked(native.borrow());
	}

	public function faceNormal():CadKit.Vec3 {
		return CadKit.faceNormalChecked(native.borrow());
	}

	/** The axis of a cylindrical, conical, spherical, toroidal or revolved face (throws for other surfaces). */
	public function faceAxis():CadKit.GeometricAxis {
		return CadKit.faceAxisChecked(native.borrow());
	}

	/** The center, normal and radius of a circular edge (throws for other curves). */
	public function edgeAxis():CadKit.GeometricAxis {
		return CadKit.edgeAxisChecked(native.borrow());
	}

	public function curveKind():CadKit.CurveKind {
		return CadKit.edgeCurveKindChecked(native.borrow());
	}

	public function edgeLength():Float {
		return CadKit.edgeLengthChecked(native.borrow());
	}

	public function tangentAt(parameter:Float = 0.5):CadKit.Vec3 {
		return CadKit.edgeTangentAtChecked(native.borrow(), parameter);
	}

	public function positionAt(parameter:Float):CadKit.Vec3 {
		return CadKit.edgePositionAtChecked(native.borrow(), parameter);
	}

	public function position():CadKit.Vec3 {
		return CadKit.vertexPositionChecked(native.borrow());
	}

	public function tessellate(linearDeflection:Float = 0.1, angularDeflection:Float = 0.5):Mesh {
		for (entry in meshCache) {
			if (entry.linearDeflection == linearDeflection &&
				entry.angularDeflection == angularDeflection)
				return entry.mesh;
		}
		var mesh = Mesh.fromShape(
			native.borrow(),
			Geometry.meshOptions(linearDeflection, angularDeflection), linearDeflection);
		meshCache.push({
			linearDeflection: linearDeflection,
			angularDeflection: angularDeflection,
			mesh: mesh
		});
		return mesh;
	}

	/** Tessellate with a linear deflection relative to this shape's size. */
	public function tessellateRelative(relativeDeflection:Float = 0.005,
		angularDeflection:Float = 0.5):Mesh {
		if (!Math.isFinite(relativeDeflection) || relativeDeflection <= 0)
			throw "Relative tessellation deflection must be positive";
		var box = bounds(), minimum = box.get_min(), maximum = box.get_max();
		var dx = maximum.get_x() - minimum.get_x();
		var dy = maximum.get_y() - minimum.get_y();
		var dz = maximum.get_z() - minimum.get_z();
		var diagonal = Math.sqrt(dx * dx + dy * dy + dz * dz);
		return tessellate(Math.max(1e-6, diagonal * relativeDeflection), angularDeflection);
	}

	public function translate(delta:CadKit.Vec3):Shape {
		return new Shape(CadKit.shapeTranslateChecked(native.borrow(), delta));
	}

	public function translateOperation(delta:CadKit.Vec3):Operation {
		return new Operation(CadKit.shapeTranslateOperationChecked(native.borrow(), delta));
	}

	public function extrude(delta:CadKit.Vec3):Shape {
		return new Shape(CadKit.shapeExtrudeChecked(native.borrow(), delta));
	}

	public function extrudeOperation(delta:CadKit.Vec3):Operation {
		return new Operation(CadKit.shapeExtrudeOperationChecked(native.borrow(), delta));
	}

	public function revolve(
		axisOrigin:CadKit.Vec3,
		axisDirection:CadKit.Vec3,
		angle:Float):Shape {
		return new Shape(CadKit.shapeRevolveChecked(
			native.borrow(), axisOrigin, axisDirection, angle));
	}

	public function revolveOperation(
		axisOrigin:CadKit.Vec3,
		axisDirection:CadKit.Vec3,
		angle:Float):Operation {
		return new Operation(CadKit.shapeRevolveOperationChecked(
			native.borrow(), axisOrigin, axisDirection, angle));
	}

	public function fillet(radius:Float):Shape {
		return new Shape(CadKit.shapeFilletChecked(native.borrow(), radius));
	}

	public function filletOperation(radius:Float):Operation {
		return new Operation(CadKit.shapeFilletOperationChecked(native.borrow(), radius));
	}

	public function chamfer(distance:Float):Shape {
		return new Shape(CadKit.shapeChamferChecked(native.borrow(), distance));
	}

	public function chamferOperation(distance:Float):Operation {
		return new Operation(CadKit.shapeChamferOperationChecked(native.borrow(), distance));
	}

	public function filletEdges(edges:Array<Edge>, radius:Float):Shape {
		return new Shape(CadKit.shapeFilletEdgesChecked(
			native.borrow(), edgeRefs(edges), radius));
	}

	public function filletEdgesOperation(edges:Array<Edge>, radius:Float):Operation {
		return new Operation(CadKit.shapeFilletEdgesOperationChecked(
			native.borrow(), edgeRefs(edges), radius));
	}

	public function chamferEdges(edges:Array<Edge>, distance:Float):Shape {
		return new Shape(CadKit.shapeChamferEdgesChecked(
			native.borrow(), edgeRefs(edges), distance));
	}

	public function chamferEdgesOperation(edges:Array<Edge>, distance:Float):Operation {
		return new Operation(CadKit.shapeChamferEdgesOperationChecked(
			native.borrow(), edgeRefs(edges), distance));
	}

	public function rotateOperation(axis:CadKit.Vec3, angle:Float):Operation {
		return new Operation(CadKit.shapeRotateOperationChecked(native.borrow(), axis, angle));
	}

	public function mirror(planeOrigin:CadKit.Vec3, planeNormal:CadKit.Vec3):Shape {
		return new Shape(CadKit.shapeMirrorChecked(native.borrow(), planeOrigin, planeNormal));
	}

	public function mirrorOperation(planeOrigin:CadKit.Vec3, planeNormal:CadKit.Vec3):Operation {
		return new Operation(CadKit.shapeMirrorOperationChecked(native.borrow(), planeOrigin, planeNormal));
	}

	public function fuse(other:Shape):Shape {
		return new Shape(CadKit.fuseChecked(native.borrow(), other.native.borrow()));
	}

	public function fuseOperation(other:Shape):Operation {
		return new Operation(CadKit.fuseOperationChecked(native.borrow(), other.native.borrow()));
	}

	public function cut(other:Shape):Shape {
		return new Shape(CadKit.cutChecked(native.borrow(), other.native.borrow()));
	}

	public function cutOperation(other:Shape):Operation {
		return new Operation(CadKit.cutOperationChecked(native.borrow(), other.native.borrow()));
	}

	public function common(other:Shape):Shape {
		return new Shape(CadKit.commonChecked(native.borrow(), other.native.borrow()));
	}

	public function commonOperation(other:Shape):Operation {
		return new Operation(CadKit.commonOperationChecked(native.borrow(), other.native.borrow()));
	}

	public function close():Bool {
		meshCache.resize(0);
		return native.close();
	}

	public function isClosed():Bool {
		return native.isClosed();
	}

	private static function edgeRefs(edges:Array<Edge>):Array<CadKit.ShapeRef> {
		if (edges == null || edges.length == 0)
			throw new SelectionError(
				SelectionErrorKind.Empty,
				"edge finishing requires at least one selected edge");

		var refs:Array<CadKit.ShapeRef> = [];
		for (edge in edges) {
			if (edge == null)
				throw new SelectionError(
					SelectionErrorKind.Invalid,
					"edge finishing does not accept null edges");
			var reference = new CadKit.ShapeRef();
			reference.set_shape(edge.borrowHandle());
			refs.push(reference);
		}
		return refs;
	}
}
