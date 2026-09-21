package cadkit;

import CadKit;
import cadkit.Geometry;

/** Owning, headless CAD shape façade for Haxeon applications. */
class Shape {
	private var native:CadKit.OwnedShapeHandle;
	private var meshCache:Array<{linearDeflection:Float, angularDeflection:Float, mesh:Mesh}>;

	private function new(native:CadKit.OwnedShapeHandle) {
		this.native = native;
		meshCache = [];
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

	public function curveKind():CadKit.CurveKind {
		return CadKit.edgeCurveKindChecked(native.borrow());
	}

	public function edgeLength():Float {
		return CadKit.edgeLengthChecked(native.borrow());
	}

	public function tangentAt(parameter:Float = 0.5):CadKit.Vec3 {
		return CadKit.edgeTangentAtChecked(native.borrow(), parameter);
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
			Geometry.meshOptions(linearDeflection, angularDeflection));
		meshCache.push({
			linearDeflection: linearDeflection,
			angularDeflection: angularDeflection,
			mesh: mesh
		});
		return mesh;
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
