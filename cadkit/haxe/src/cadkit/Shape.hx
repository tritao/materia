package cadkit;

import CadKit;
import cadkit.Geometry;
import cadkit.threads.ThreadedFace;
import cadkit.threads.Threads;

/** Owning, headless CAD shape façade for Haxeon applications. */
class Shape {
  private var threadAttributes:Array<ThreadedFace>;
  /** Hash of the exact bytes of `threadAttributes` ("" when there are none): the part of the key that names the attributes. */
  private var attributeDigest:String;
  private var native:CadKit.OwnedShapeHandle;
  /** How this shape was made, hashed (see `ShapeKey`); null when it was made by something unkeyed. */
  public var key:Null<String>;
  /** Why `key` is null, for coverage reports (the first unkeyed ancestor or operation). */
  public var gap:Null<String>;
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
    key = null;
    gap = "native-handle";
    threadAttributes = [];
    attributeDigest = "";
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
    return primitive(
      new Shape(CadKit.boxChecked(width, depth, height)),
      "box",
      ShapeKey.number(width) + "," + ShapeKey.number(depth) + "," + ShapeKey.number(height)
    );
  }

  public static function cylinder(radius:Float, height:Float):Shape {
    return primitive(
      new Shape(CadKit.cylinderChecked(radius, height)),
      "cylinder",
      ShapeKey.number(radius) + "," + ShapeKey.number(height)
    );
  }

  public static function sphere(radius:Float):Shape {
    return primitive(new Shape(CadKit.sphereChecked(radius)), "sphere", ShapeKey.number(radius));
  }

  public static function importStep(path:String):Shape {
    // The key is the file's content, so the same file elsewhere (or edited in place) is the right entry.
    var content = sys.io.File.getBytes(path);
    return primitive(new Shape(CadKit.stepImportChecked(path)), "step", ShapeKey.bytesHash(content));
  }

  /** Import an authored STEP text payload without relying on its original file path. */
  public static function importStepText(text:String):Shape {
    return primitive(
      new Shape(CadKit.stepImportTextChecked(text)),
      "step",
      ShapeKey.bytesHash(haxe.io.Bytes.ofString(text))
    );
  }

  public function exportStep(path:String):Void {
    CadKit.stepExportChecked(native.borrow(), path);
  }

  /** Explicit shape finalization, preserving aliases and threaded-face history.
	 * Merges redundant same-domain topology; leaves this shape unchanged. */
  public function refined():Shape return operationShape(refineOperation());

  public function refineOperation():Operation {
    return new Operation(
      CadKit.shapeRefineOperationChecked(native.borrow()),
      [this],
      ShapeKey.op("refine-op", "", [this]),
      "refine"
    );
  }

  public function cloneShape():Shape {
    var copy = copiedAttributes(new Shape(CadKit.shapeCloneChecked(native.borrow())));
    // A clone is the same shape, attributes included: same key, same reason when it has none.
    copy.key = key;
    copy.gap = key == null ? (gap ?? "unkeyed") : null;
    return copy;
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

  /** The least distance between this shape and `other`. Solids count as volumes: zero where they touch, overlap or one holds the other. */
  public function distance(other:Shape):Float {
    return CadKit.shapeDistanceChecked(native.borrow(), other.native.borrow());
  }

  /** Exact closest-point pairs for positive separation. The first query returns
	 * their count; zero means the shapes touch or overlap. */
  public function distanceWitness(other:Shape, index:Int = 0):CadKit.DistanceWitness {
    if (index < 0) throw "Distance witness index out of range";
    return CadKit.shapeDistanceWitnessChecked(native.borrow(), other.native.borrow(), index);
  }

  public function volume(?relativeTolerance:Float):Float {
    if (relativeTolerance != null) return CadKit.shapeVolumeWithToleranceChecked(native.borrow(), relativeTolerance);
    return CadKit.shapeVolumeChecked(native.borrow());
  }

  /**
   * Compute volume, area, centroid, and unit-density centroidal inertia. A keyed shape's answer is kept on disk
   * (`OpCache.query`): it is the same whether or not the shape was meshed before (checked in ShapeKeySmoke).
   */
  public function massProperties():PhysicalProperties return cadkit.cache.OpCache.query(
    "mass",
    "cadkit.Shape",
    this,
    "",
    () -> computeMassProperties(),
    properties -> properties.toBytes(),
    PhysicalProperties.fromBytes
  );

  private function computeMassProperties():PhysicalProperties {
    var nativeProperties = CadKit.shapePhysicalPropertiesChecked(native.borrow());
    var tensor = nativeProperties.get_inertia();
    return new PhysicalProperties(
      nativeProperties.get_volume(),
      nativeProperties.get_surfaceArea(),
      cadkit.modeling.Vector.fromNative(nativeProperties.get_centerOfMass()),
      new cadkit.InertiaTensor(
        tensor.get_xx(),
        tensor.get_xy(),
        tensor.get_xz(),
        tensor.get_yy(),
        tensor.get_yz(),
        tensor.get_zz()
      )
    );
  }

  public function kind():CadKit.ShapeKind {
    return CadKit.shapeKindGetChecked(native.borrow());
  }

  /** All tessellated faces belong to solids; a mixed solid/sheet compound is not a solid boundary. */
  public function hasSolidBoundary():Bool {
    if (kind() == CadKit.ShapeKind.Solid) return true;
    var count = subshapeCount(CadKit.ShapeKind.Solid);
    if (count == 0) return false;
    var faces = 0;
    for (index in 0...count) {
      var solid = subshape(CadKit.ShapeKind.Solid, index);
      try faces += solid.subshapeCount(CadKit.ShapeKind.Face) catch (error:Dynamic) {
        solid.close();
        throw error;
      }
      solid.close();
    }
    return faces == subshapeCount(CadKit.ShapeKind.Face);
  }

  public function subshapeCount(kind:CadKit.ShapeKind):Int {
    return CadKit.shapeSubshapeCountChecked(native.borrow(), kind);
  }

  public function subshape(kind:CadKit.ShapeKind, index:Int):Shape {
    return new Shape(CadKit.shapeSubshapeAtChecked(native.borrow(), kind, index));
  }

  /** Unique containing subshapes, with names inherited from this source. */
  public function ancestorCount(sub:Shape, kind:CadKit.ShapeKind):Int {
    return CadKit.shapeAncestorCountChecked(native.borrow(), sub.borrowHandle(), kind);
  }
  public function ancestor(sub:Shape, kind:CadKit.ShapeKind, index:Int):Shape {
    return new Shape(CadKit.shapeAncestorAtChecked(native.borrow(), sub.borrowHandle(), kind, index));
  }
  public function ancestors(sub:Shape, kind:CadKit.ShapeKind):Array<Shape> {
    var result:Array<Shape> = [];
    try {
      var count = ancestorCount(sub, kind);
      for (index in 0...count) result.push(ancestor(sub, kind, index));
      return result;
    } catch (error:Dynamic) {
      for (shape in result) shape.close();
      throw error;
    }
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
    var cached = kind == CadKit.ShapeKind.Face ? faceNames : kind == CadKit.ShapeKind.Edge
      ? edgeNames : kind == CadKit.ShapeKind.Vertex ? vertexNames : kind == CadKit.ShapeKind.Solid ? solidNames : null;
    if (cached == null) {
      if (subshapeCount(kind) == 0) {
        cached = [];
      } else {
        var bytes = CadKit.shapeCopyElementNamesBytesChecked(native.borrow(), kind);
        cached = bytes.getString(0, bytes.length).split("\n");
      }
      if (kind == CadKit.ShapeKind.Face) faceNames = cached;
      else if (kind == CadKit.ShapeKind.Edge) edgeNames = cached;
      else if (kind == CadKit.ShapeKind.Vertex) vertexNames = cached;
      else if (kind == CadKit.ShapeKind.Solid) solidNames = cached;
    }
    return cached.copy();
  }

  public function elementName(kind:CadKit.ShapeKind, index:Int):String {
    var names = elementNames(kind);
    if (index < 0 || index >= names.length) throw "element index is out of range";
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
      if (kind == CadKit.ShapeKind.Face) faceAliases = cached;
      else if (kind == CadKit.ShapeKind.Solid) solidAliases = cached;
    }
    var known:Array<Array<String>> = cast cached;
    return [for (aliases in known) aliases.copy()];
  }

  function readAliases(kind:CadKit.ShapeKind):Array<Array<String>> {
    var result:Array<Array<String>> = [for (_ in 0...subshapeCount(kind))[]];
    if (kind != CadKit.ShapeKind.Face && kind != CadKit.ShapeKind.Solid) return result;
    var bytes = CadKit.shapeCopyElementAliasesBytesChecked(native.borrow(), kind);
    if (bytes.length == 0) return result;
    for (line in bytes.getString(0, bytes.length).split("\n")) {
      var tab = line.indexOf("\t");
      if (tab <= 0) continue;
      var index = Std.parseInt(line.substr(0, tab));
      if (index != null && index >= 0 && index < result.length) result[index].push(line.substr(tab + 1));
    }
    return result;
  }

  /**
		A copy whose faces, edges or vertices are named by `ids` (one per subshape; "" keeps the current name).
		Each id is escaped into a name. Edge and vertex names hold where faces cannot name them: boundary and
		wire edges and their vertices.
	**/
  public function withElementNames(kind:CadKit.ShapeKind, ids:Array<String>):Shape {
    return derived(
      copiedAttributes(new Shape(CadKit.shapeSeedNamesChecked(native.borrow(), kind, ids.join("\n"))), true),
      "names",
      Std.string(kind) + "\n" + ids.join("\n"),
      [this]
    );
  }

  /** A copy whose names that no input has are prefixed by `tag:`: what was created from the inputs. */
  public function stamped(tag:String, inputs:Array<Shape>):Shape {
    var refs:Array<CadKit.ShapeRef> = [];
    for (input in inputs) {
      var ref = new CadKit.ShapeRef();
      ref.set_shape(input.borrowHandle());
      refs.push(ref);
    }
    return derived(
      copiedAttributes(new Shape(CadKit.shapeStampNamesChecked(native.borrow(), tag, refs)), true),
      "stamp",
      tag,
      [this].concat(inputs)
    );
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

  /** The version text of the OCCT this CadKit was built against (the same text shape bytes carry). */
  public static function occtVersion():String {
    return CadKit.occtVersion();
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

  /**
   * The mesh of this shape. Pure: it meshes an independent copy, so the shape's own topology, `bounds()` and later meshes never
   * depend on what was meshed before (OCCT stores triangulations on the shape's faces and a coarser request after a finer one
   * returns the finer mesh). A keyed shape's mesh is kept on disk (`OpCache.query`); the copy keeps topology order and names, so
   * the mesh's face and edge indices are this shape's.
   */
  public function tessellate(linearDeflection:Float = 0.1, angularDeflection:Float = 0.5):Mesh {
    for (entry in meshCache) {
      if (entry.linearDeflection == linearDeflection && entry.angularDeflection == angularDeflection) return entry.mesh;
    }
    var mesh = cadkit.cache.OpCache.query(
      "tessellate",
      "cadkit.Shape",
      this,
      ShapeKey.number(linearDeflection) + "," + ShapeKey.number(angularDeflection),
      () -> meshOfCopy(linearDeflection, angularDeflection),
      value -> value.toBytes(),
      Mesh.fromBytes
    );
    meshCache.push({linearDeflection: linearDeflection, angularDeflection: angularDeflection, mesh: mesh});
    return mesh;
  }

  private function meshOfCopy(linearDeflection:Float, angularDeflection:Float):Mesh {
    var copy = CadKit.shapeTranslateChecked(native.borrow(), Geometry.vec3(0, 0, 0));
    try {
      var mesh = Mesh.fromShape(
        copy.borrow(),
        Geometry.meshOptions(linearDeflection, angularDeflection),
        linearDeflection
      );
      copy.close();
      return mesh;
    } catch (error:Dynamic) {
      copy.close();
      throw error;
    }
  }

  /** Tessellate with a linear deflection relative to this shape's size. */
  public function tessellateRelative(relativeDeflection:Float = 0.005, angularDeflection:Float = 0.5):Mesh {
    if (!Math.isFinite(relativeDeflection)
      || relativeDeflection <= 0) throw "Relative tessellation deflection must be positive";
    var box = bounds(), minimum = box.get_min(), maximum = box.get_max();
    var dx = maximum.get_x() - minimum.get_x();
    var dy = maximum.get_y() - minimum.get_y();
    var dz = maximum.get_z() - minimum.get_z();
    var diagonal = Math.sqrt(dx * dx + dy * dy + dz * dz);
    return tessellate(Math.max(1e-6, diagonal * relativeDeflection), angularDeflection);
  }

  public function translate(delta:CadKit.Vec3):Shape {
    if (threadAttributes.length > 0) return operationShape(translateOperation(delta));
    return derived(
      new Shape(CadKit.shapeTranslateChecked(native.borrow(), delta)),
      "translate",
      ShapeKey.vec(delta),
      [this]
    );
  }

  public function translateOperation(delta:CadKit.Vec3):Operation {
    return new Operation(
      CadKit.shapeTranslateOperationChecked(native.borrow(), delta),
      [this],
      ShapeKey.op("translate-op", ShapeKey.vec(delta), [this]),
      "translate"
    );
  }

  public function extrude(delta:CadKit.Vec3):Shape {
    if (threadAttributes.length > 0) return operationShape(extrudeOperation(delta));
    return derived(
      new Shape(CadKit.shapeExtrudeChecked(native.borrow(), delta)),
      "extrude",
      ShapeKey.vec(delta),
      [this]
    );
  }

  public function extrudeOperation(delta:CadKit.Vec3):Operation {
    return new Operation(
      CadKit.shapeExtrudeOperationChecked(native.borrow(), delta),
      [this],
      ShapeKey.op("extrude-op", ShapeKey.vec(delta), [this]),
      "extrude"
    );
  }

  public function revolve(axisOrigin:CadKit.Vec3, axisDirection:CadKit.Vec3, angle:Float):Shape {
    if (threadAttributes.length > 0) return operationShape(revolveOperation(axisOrigin, axisDirection, angle));
    return derived(
      new Shape(CadKit.shapeRevolveChecked(native.borrow(), axisOrigin, axisDirection, angle)),
      "revolve",
      ShapeKey.vec(axisOrigin) + "|" + ShapeKey.vec(axisDirection) + "|" + ShapeKey.number(angle),
      [this]
    );
  }

  public function revolveOperation(axisOrigin:CadKit.Vec3, axisDirection:CadKit.Vec3, angle:Float):Operation {
    return new Operation(
      CadKit.shapeRevolveOperationChecked(native.borrow(), axisOrigin, axisDirection, angle),
      [this],
      ShapeKey.op(
        "revolve-op",
        ShapeKey.vec(axisOrigin) + "|" + ShapeKey.vec(axisDirection) + "|" + ShapeKey.number(angle),
        [this]
      ),
      "revolve"
    );
  }

  public function fillet(radius:Float):Shape {
    if (threadAttributes.length > 0) return operationShape(filletOperation(radius));
    return derived(
      new Shape(CadKit.shapeFilletChecked(native.borrow(), radius)),
      "fillet",
      ShapeKey.number(radius),
      [this]
    );
  }

  public function filletOperation(radius:Float):Operation {
    return new Operation(
      CadKit.shapeFilletOperationChecked(native.borrow(), radius),
      [this],
      ShapeKey.op("fillet-op", ShapeKey.number(radius), [this]),
      "fillet"
    );
  }

  public function chamfer(distance:Float):Shape {
    if (threadAttributes.length > 0) return operationShape(chamferOperation(distance));
    return derived(
      new Shape(CadKit.shapeChamferChecked(native.borrow(), distance)),
      "chamfer",
      ShapeKey.number(distance),
      [this]
    );
  }

  public function chamferOperation(distance:Float):Operation {
    return new Operation(
      CadKit.shapeChamferOperationChecked(native.borrow(), distance),
      [this],
      ShapeKey.op("chamfer-op", ShapeKey.number(distance), [this]),
      "chamfer"
    );
  }

  public function filletEdges(edges:Array<Edge>, radius:Float):Shape {
    if (threadAttributes.length > 0) return operationShape(filletEdgesOperation(edges, radius));
    return new Shape(CadKit.shapeFilletEdgesChecked(native.borrow(), edgeRefs(edges), radius)).unkeyed("fillet-edges");
  }

  public function filletEdgesOperation(edges:Array<Edge>, radius:Float):Operation {
    return new Operation(
      CadKit.shapeFilletEdgesOperationChecked(native.borrow(), edgeRefs(edges), radius),
      [this],
      null,
      "fillet-edges"
    );
  }

  public function chamferEdges(edges:Array<Edge>, distance:Float):Shape {
    if (threadAttributes.length > 0) return operationShape(chamferEdgesOperation(edges, distance));
    return new Shape(CadKit.shapeChamferEdgesChecked(native.borrow(),
      edgeRefs(edges), distance)).unkeyed("chamfer-edges");
  }

  public function chamferEdgesOperation(edges:Array<Edge>, distance:Float):Operation {
    return new Operation(
      CadKit.shapeChamferEdgesOperationChecked(native.borrow(), edgeRefs(edges), distance),
      [this],
      null,
      "chamfer-edges"
    );
  }

  public function rotateOperation(axis:CadKit.Vec3, angle:Float):Operation {
    return new Operation(
      CadKit.shapeRotateOperationChecked(native.borrow(), axis, angle),
      [this],
      ShapeKey.op("rotate-op", ShapeKey.vec(axis) + "|" + ShapeKey.number(angle), [this]),
      "rotate"
    );
  }

  public function mirror(planeOrigin:CadKit.Vec3, planeNormal:CadKit.Vec3):Shape {
    if (threadAttributes.length > 0) return operationShape(mirrorOperation(planeOrigin, planeNormal));
    return derived(
      new Shape(CadKit.shapeMirrorChecked(native.borrow(), planeOrigin, planeNormal)),
      "mirror",
      ShapeKey.vec(planeOrigin) + "|" + ShapeKey.vec(planeNormal),
      [this]
    );
  }

  public function mirrorOperation(planeOrigin:CadKit.Vec3, planeNormal:CadKit.Vec3):Operation {
    return new Operation(
      CadKit.shapeMirrorOperationChecked(native.borrow(), planeOrigin, planeNormal),
      [this],
      ShapeKey.op("mirror-op", ShapeKey.vec(planeOrigin) + "|" + ShapeKey.vec(planeNormal), [this]),
      "mirror"
    );
  }

  /** Kept on disk under the construction keys of both inputs when they have them (`OpCache`); the result is then the canonical form. */
  public function fuse(other:Shape):Shape return cadkit.cache.OpCache.boolean(
    "fuse",
    this,
    other,
    () -> fuseDirect(other)
  );

  private function fuseDirect(other:Shape):Shape {
    if (threadAttributes.length > 0 || other.threadAttributes.length > 0) return operationShape(fuseOperation(other));
    return new Shape(CadKit.fuseChecked(native.borrow(), other.native.borrow()));
  }

  public function fuseOperation(other:Shape):Operation {
    // History-keeping booleans are keyed under their own name: their result is the built form, not the canonical one a cached
    // `fuse` gives, and a key must name one form only.
    var key = ShapeKey.op("fuse-history", "", [this, other]);
    return cadkit.cache.OpCache.historyOperation(
      "fuse",
      key == null ? ShapeKey.gapOf("fuse", [this, other]) : null,
      () -> new Operation(
        CadKit.fuseOperationChecked(native.borrow(), other.native.borrow()),
        [this, other],
        key,
        "fuse"
      )
    );
  }

  /** Kept on disk under the construction keys of both inputs when they have them (`OpCache`); the result is then the canonical form. */
  public function cut(other:Shape):Shape return cadkit.cache.OpCache.boolean("cut",
    this, other, () -> cutDirect(other));

  private function cutDirect(other:Shape):Shape {
    if (threadAttributes.length > 0 || other.threadAttributes.length > 0) return operationShape(cutOperation(other));
    return new Shape(CadKit.cutChecked(native.borrow(), other.native.borrow()));
  }

  /**
   * This shape minus every tool in one native operation (the tools may overlap), instead of fusing the tools first. Kept on disk
   * under the construction keys of all the shapes when they have them (`OpCache`); the result is then the canonical form. The
   * caller checks that no shape carries thread attributes.
   */
  public function cutMany(tools:Array<Shape>):Shape {
    return cadkit.cache.OpCache.construct("cut-many", "", [this].concat(tools), () -> {
      var refs:Array<CadKit.ShapeRef> = [];
      for (tool in tools) {
        var ref = new CadKit.ShapeRef();
        ref.set_shape(tool.borrowHandle());
        refs.push(ref);
      }
      return new Shape(CadKit.cutManyChecked(native.borrow(), refs));
    });
  }

  public function cutOperation(other:Shape):Operation {
    // History-keeping booleans are keyed under their own name: their result is the built form, not the canonical one a cached
    // `cut` gives, and a key must name one form only.
    var key = ShapeKey.op("cut-history", "", [this, other]);
    return cadkit.cache.OpCache.historyOperation(
      "cut",
      key == null ? ShapeKey.gapOf("cut", [this, other]) : null,
      () -> new Operation(CadKit.cutOperationChecked(native.borrow(), other.native.borrow()), [this, other], key, "cut")
    );
  }

  /** Kept on disk under the construction keys of both inputs when they have them (`OpCache`); the result is then the canonical form. */
  public function common(other:Shape):Shape {
    // Shapes whose bounding boxes are apart share nothing: the answer is the empty shape, without a boolean (a large part of the
    // common operations of a scene are overlap questions with this answer).
    if (key != null && other.key != null && threadAttributes.length == 0 && other.threadAttributes.length == 0 && apartFrom(other)) {
      var empty = Shape.compound([]).canonical();
      empty.key = ShapeKey.op("common", "", [this, other]);
      empty.gap = null;
      return empty;
    }
    return cadkit.cache.OpCache.boolean("common", this, other, () -> commonDirect(other));
  }

  private var extent:Null<Array<Float>>;

  /** The bounding box as [minX, minY, minZ, maxX, maxY, maxZ], measured once per shape; null when it cannot be measured. */
  private function extentBox():Null<Array<Float>> {
    if (extent == null) {
      try {
        var box = bounds(), low = box.get_min(), high = box.get_max();
        extent = [low.get_x(), low.get_y(), low.get_z(), high.get_x(), high.get_y(), high.get_z()];
      } catch (error:Dynamic) {
        return null;
      }
    }
    return extent;
  }

  /** True when the bounding boxes are separated by more than a micrometre on some axis, so the shapes cannot meet. */
  private function apartFrom(other:Shape):Bool {
    var a = extentBox(), b = other.extentBox();
    if (a == null || b == null) return false;
    for (axis in 0...3) if (a[axis] > b[axis + 3] + 1e-3 || b[axis] > a[axis + 3] + 1e-3) return true;
    return false;
  }

  private function commonDirect(other:Shape):Shape {
    if (threadAttributes.length > 0 || other.threadAttributes.length > 0) return operationShape(commonOperation(other));
    return new Shape(CadKit.commonChecked(native.borrow(), other.native.borrow()));
  }

  public function commonOperation(other:Shape):Operation {
    // History-keeping booleans are keyed under their own name: their result is the built form, not the canonical one a cached
    // `common` gives, and a key must name one form only.
    var key = ShapeKey.op("common-history", "", [this, other]);
    return cadkit.cache.OpCache.historyOperation(
      "common",
      key == null ? ShapeKey.gapOf("common", [this, other]) : null,
      () -> new Operation(
        CadKit.commonOperationChecked(native.borrow(), other.native.borrow()),
        [this, other],
        key,
        "common"
      )
    );
  }

  /** The planar faces of this solid on the plane through `origin` with `normal`, holes kept; an empty region when it misses. */
  public function section(origin:CadKit.Vec3, normal:CadKit.Vec3):Shape {
    return new Shape(CadKit.shapeSectionChecked(native.borrow(), origin, normal));
  }

  /** Every face of this planar region grown by `distance` in its plane (negative shrinks it); a face may vanish or split. */
  public function planarOffset(distance:Float):Shape {
    return new Shape(CadKit.planarOffsetChecked(native.borrow(), distance));
  }

  public function planarCut(other:Shape):Shape {
    return new Shape(CadKit.planarCutChecked(native.borrow(), other.native.borrow()));
  }

  public function planarCommon(other:Shape):Shape {
    return new Shape(CadKit.planarCommonChecked(native.borrow(), other.native.borrow()));
  }

  public function planarFuse(other:Shape):Shape {
    return new Shape(CadKit.planarFuseChecked(native.borrow(), other.native.borrow()));
  }

  /** Exact boundary segments of a horizontal planar region: see `cad_planar_copy_boundary_bytes` (14 doubles per segment). */
  public function planarBoundary(chordTolerance:Float):haxe.io.Bytes {
    return CadKit.planarCopyBoundaryBytesChecked(native.borrow(), chordTolerance);
  }

  /**
   * The shape as bytes for a cache on disk: the native B-rep and element names (`cad_shape_write_bytes`) and the face
   * attributes CadKit keeps in Haxe. Triangulations are not kept, so a restored shape meshes as a fresh one does. The bytes
   * are the canonical form (see `canonical`), so they read back to a shape that writes exactly them again.
   */
  public function toBytes():haxe.io.Bytes {
    var nativeBytes = CadKit.shapeWriteBytesChecked(native.borrow());
    var attributes = cadkit.threads.FaceAttributeCodec.encode(threadAttributes);
    var out = new haxe.io.BytesOutput();
    out.writeInt32(shapeBytesVersion);
    out.writeInt32(nativeBytes.length);
    out.write(nativeBytes);
    out.writeInt32(attributes.length);
    out.write(attributes);
    out.writeInt32(adler(attributes));
    return out.getBytes();
  }

  /**
   * The canonical form of this shape: `fromBytes(toBytes())`, the one form a cache keeps and hands out.
   *
   * OCCT's binary reader rebuilds every `gp_Trsf` (locations: determinant normalisation and Gram-Schmidt) and `gp_Ax3`
   * (surface placements: Y recomputed from N and X), which moves last bits and flips signed zeros, in the geometry and so in
   * meshes built from it. Round-off makes that converge over a few passes rather than one, so `toBytes` writes the fixed point
   * (`cad_shape_write_bytes`): `fromBytes(b).toBytes()` is exactly `b`, and `canonical().canonical()` equals `canonical()`
   * in every byte, mesh included. A shape built from scratch is generally not its own canonical form (it can differ in the last
   * bits of rotated placements and in the sign of zeros), so anything that stores shapes and also computes them, a persistent
   * memo, must return the canonical form on both paths, never the computed shape on one and the restored one on the other.
   */
  public function canonical():Shape {
    var result = fromBytes(toBytes());
    // The canonical form differs from the built one in last-bit details, so it is another shape to anything keyed on how it was
    // made: `canonical` is an operation on the key (an unkeyed shape stays unkeyed).
    if (key != null) {
      result.key = ShapeKey.hash("cadkit-shape/1\ncanonical\n" + key);
      result.gap = null;
    } else result.gap = gap ?? "canonical";
    return result;
  }

  /** A shape from `toBytes`; damaged, truncated or foreign bytes throw. */
  public static function fromBytes(bytes:haxe.io.Bytes):Shape {
    var input = new haxe.io.BytesInput(bytes);
    if (input.readInt32() != shapeBytesVersion) throw "Unsupported shape bytes";
    var nativeLength = input.readInt32();
    if (nativeLength < 0 || nativeLength > bytes.length) throw "Damaged shape bytes";
    var nativeBytes = input.read(nativeLength);
    var attributeLength = input.readInt32();
    if (attributeLength < 0 || attributeLength > bytes.length) throw "Damaged shape bytes";
    var attributeBytes = input.read(attributeLength);
    if (input.readInt32() != adler(attributeBytes)) throw "Damaged shape bytes (attribute checksum)";
    var attributes = cadkit.threads.FaceAttributeCodec.decode(attributeBytes);
    var trailing = true;
    try input.readByte() catch (error:Dynamic) trailing = false;
    if (trailing) throw "Damaged shape bytes";
    var shape = new Shape(CadKit.shapeReadBytesChecked(nativeBytes));
    shape.setThreadedFaces(attributes);
    return shape;
  }

  static final shapeBytesVersion = 1;

  /** Adler-32 of the attribute section (the native section carries its own checksum). */
  static function adler(bytes:haxe.io.Bytes):Int {
    var a = 1, b = 0;
    for (index in 0...bytes.length) {
      a = (a + bytes.get(index)) % 65521;
      b = (b + a) % 65521;
    }
    return (b << 16) | a;
  }

  public function close():Bool {
    meshCache.resize(0);
    return native.close();
  }

  public function isClosed():Bool {
    return native.isClosed();
  }

  /** The threads on this shape's faces. Semantic face attributes travel with the immutable B-rep owner. */
  public function threadedFaces():Array<ThreadedFace
    > return [for (attribute in threadAttributes) if (attribute.detail == null) attribute];

  /** The cosmetic surface details (knurling, scales) on this shape's faces. */
  public function surfaceDetails():Array<ThreadedFace
    > return [for (attribute in threadAttributes) if (attribute.detail != null) attribute];

  /** Every face attribute, threads and details: what an operation carries from its inputs to its result. */
  public function faceAttributes():Array<ThreadedFace> return threadAttributes.copy();

  /**
   * Internal propagation boundary; attributes contain no native handles.
   *
   * Face attributes live in Haxe beside the B-rep, so they are part of what a shape is: a keyed shape whose attributes change
   * is keyed again from its old key and the exact bytes of the new attributes (the same construction with the same attributes
   * gives the same key, any other attribute another). Attributes are immutable, so the digest cannot go stale.
   */
  public function setThreadedFaces(attributes:Array<ThreadedFace>):Void {
    var digest = attributes.length == 0 ? "" : ShapeKey.bytesHash(cadkit.threads.FaceAttributeCodec.encode(attributes));
    var changed = digest != attributeDigest;
    threadAttributes = attributes.copy();
    attributeDigest = digest;
    if (changed && key != null) key = ShapeKey.hash("cadkit-shape/1\nattributes\n" + key + "\n" + digest);
  }

  private function copiedAttributes(copy:Shape, rename:Bool = false):Shape {
    try {
      var attributes:Array<ThreadedFace> = [];
      for (attribute in threadAttributes) {
        var index = Threads.namedIndex(this, attribute.faceName);
        attributes.push(rename ? attribute.renamed(copy.elementName(CadKit.ShapeKind.Face, index)) : attribute);
      }
      copy.setThreadedFaces(attributes);
      return copy;
    } catch (error:Dynamic) {
      copy.close();
      throw error;
    }
  }

  public static function compound(shapes:Array<Shape>):Shape {
    var refs:Array<CadKit.ShapeRef> = [];
    var attributes:Array<ThreadedFace> = [];
    for (shape in shapes) {
      var ref = new CadKit.ShapeRef();
      ref.set_shape(shape.borrowHandle());
      refs.push(ref);
      for (attribute in shape.faceAttributes()) attributes.push(attribute);
    }
    var result = fromOwnedHandle(CadKit.compoundChecked(refs));
    result.setThreadedFaces(attributes);
    return result;
  }

  /** `result` marked as made by `name` with `params` from `inputs`: keyed when every input is, else unkeyed with the reason. */
  private static function derived(result:Shape, name:String, params:String, inputs:Array<Shape>):Shape {
    var key = ShapeKey.op(name, params, inputs);
    result.key = key;
    result.gap = key == null ? ShapeKey.gapOf(name, inputs) : null;
    return result;
  }

  /** `result` marked as a primitive made from `params`. */
  private static function primitive(result:Shape, name:String, params:String):Shape {
    result.key = ShapeKey.leaf(name, params);
    result.gap = null;
    return result;
  }

  /** `result` marked as unkeyed for `reason` (an operation whose inputs this layer does not key). */
  public function unkeyed(reason:String):Shape {
    key = null;
    gap = reason;
    return this;
  }

  private static function operationShape(operation:Operation):Shape {
    try {
      var result = operation.resultShape();
      operation.close();
      return result;
    } catch (error:Dynamic) {
      operation.close();
      throw error;
    }
  }

  private static function edgeRefs(edges:Array<Edge>):Array<CadKit.ShapeRef> {
    if (edges == null || edges.length == 0) throw new SelectionError(
      SelectionErrorKind.Empty,
      "edge finishing requires at least one selected edge"
    );

    var refs:Array<CadKit.ShapeRef> = [];
    for (edge in edges) {
      if (edge == null) throw new SelectionError(
        SelectionErrorKind.Invalid,
        "edge finishing does not accept null edges"
      );
      var reference = new CadKit.ShapeRef();
      reference.set_shape(edge.borrowHandle());
      refs.push(reference);
    }
    return refs;
  }
}
