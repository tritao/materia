package machinekit.motion;

import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.assembly.HzMotionCatalog;
import machinekit.assembly.RbkMountData;
import machinekit.catalog.SupplierLibraries;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.MassProperties;
import machinekit.component.Solids;
import machinekit.component.SupplierCadComponent;
import materia.assembly.AssemblyFrames as F;

/** A bolt hole in a mount's base, foot frame: x across, z along from the motor face; measured top and counterbore where reviewed. */
typedef MotorMountHole = {
  var x:Float;
  var z:Float;
  var diameter:Float;
  @:optional var topY:Float;
  @:optional var counterboreDepth:Float;
  @:optional var counterboreDiameter:Float;
}

/**
 * A motor mount row: the motor flange it receives (square `face`, `pilot`, bolts on `pcd` or listed), the body
 * (`flange` thickness, rear block from `rearStart` to `rearEnd`), the screw support it carries (`bearing`, `shaftBore`,
 * the screw `endProfile` and `shoulderZ` it needs, its `oppositeSupport`), and its base bolts. Values in mm and kg.
 */
typedef MotorMountSpec = {
  var model:String;
  var maker:String;
  var revision:String;
  var reference:String;
  var family:String;
  /**
	 * How it holds the screw: `integrated`, its own fixed bearing pair in its rear block; `seat`, a pocket a separate
	 * flange support bolts into (`acceptsSupport`); `module`, the end body of a linear module.
	 */
  var construction:String;
  var acceptsSupport:Null<String>;
  var face:Float;
  var pilot:Float;
  var pilotDepth:Null<Float>;
  var pcd:Float;
  var motorBolts:Array<{x:Float, y:Float}>;
  var thread:String;
  var threadDepth:Null<Float>;
  /** Root radius of the tapped motor bolt holes, where measured on the CAD. */
  var threadRootRadius:Null<Float>;
  var flange:Float;
  var length:Float;
  var shaftHeight:Float;
  var width:Float;
  var rearStart:Float;
  var rearEnd:Float;
  var rearWidth:Float;
  var rearHeight:Float;
  var shaftBore:Float;
  var shoulderZ:Null<Float>;
  var massKg:Null<Float>;
  var baseHoles:Array<MotorMountHole>;
  var auxiliaryHoles:Array<MotorMountHole>;
  var underhang:Float;
  var bearing:Null<String>;
  var bearingArrangement:Null<String>;
  var maxAxialLoadKgf:Null<Float>;
  var oppositeSupport:Null<String>;
  var endProfile:Null<String>;
  var geometryBasis:String;
  /** Measured on the pinned CAD: the fixed bearing's centre below the motor face and the base bolts' seat. */
  var bearingCentreZ:Null<Float>;
  var boltSeatY:Null<Float>;
  var cadSha256:Null<String>;
}

/**
 * A purchased motor mount with an integrated fixed screw support: the motor bolts to its face and the screw's fixed
 * end runs in its bearings. One recipe for every product (HZMotion C/A/KK brackets, Vallder RBK brackets), drawn
 * `nominal` from the row or from its maker's pinned CAD (`hzmotion`, `vallder`). Where the CAD is reviewed, its
 * measured interfaces are the part's whichever drawing is shown; the drawing changes neither the part number nor the
 * published mass. Frame: base at y = 0, motor face at z = 0, the screw running towards +z at the shaft height.
 */
class MotorMount extends MachineComponent {
  public static inline var NOMINAL = "nominal";

  public final spec:MotorMountSpec;
  public final drawing:String;
  public final bearingGrade:String;
  public final model:String;
  public final shaftHeight:Float;
  public final pilotDiameter:Float;
  public final underhang:Float;

  static var rbkCache:Null<Array<Dynamic>>;

  public static function rbkModels():Array<String> return ["RBK12-57", "RBK15-57", "RBK15-86", "RBK17-86"];

  public static function hzmotionModels():Array<String
    > return [for (model in HzMotionCatalog.models()) if (HzMotionCatalog.get(model).kind == "mount") model];

  public static function models():Array<String> return hzmotionModels().concat(rbkModels());

  public static function revisions():Array<String> return HzMotionCatalog.revisions().concat(["vallder-rbk-v1"]);

  public static function defaultRevision(model:String):String return rbkModels().indexOf(model)
    >= 0 ? "vallder-rbk-v1" : HzMotionCatalog.revision();

  public static function of(
    model:String,
    ?revision:String,
    drawing:String = NOMINAL,
    bearingGrade:String = "unspecified"
  ):MotorMount return new MotorMount(
    model,
    revision == null ? defaultRevision(model) : revision,
    drawing,
    bearingGrade
  );

  /** The row for `model` at `revision`, with the reviewed CAD's measured interfaces applied where they belong to it. */
  public static function row(model:String,
    revision:String):MotorMountSpec return rbkModels().indexOf(model) >= 0 ? rbkRow(
      model,
      revision
    ) : hzmotionRow(
      model,
      revision
    );

  static function hzmotionRow(model:String, revision:String):MotorMountSpec {
    var s = HzMotionCatalog.get(model, revision);
    if (s.kind != "mount") throw 'HZMotion $model is not a motor mount';
    var half = s.pcd / (2 * Math.sqrt(2));
    var construction = switch s.family {
      case "A":
        "seat";
      case "KK":
        "module";
      case _:
        "integrated";
    };
    var spec:MotorMountSpec = {model: model,
      maker: "HZMotion", revision: s.drawingRevision, reference: HzMotionCatalog.sourceUrl(model),
      family: s.family, construction: construction, acceptsSupport: construction == "seat" ? model.split("-")[1] : null,
      face : s.face, pilot : s.pilot, pilotDepth : s.pilotDepth, pcd : s.pcd, motorBolts : [for (x in [
        -half,
        half
      ]) for (y in [
        -half,
        half
      ]) {
      x:x, y:y
    }
    ], thread:s.thread,
      threadDepth:s.threadDepth, threadRootRadius:null, flange:s.flange, length:s.length, shaftHeight:s.shaftHeight,
      width:s.width, rearStart:s.rearStart, rearEnd:s.rearEnd, rearWidth:s.rearWidth, rearHeight:s.rearHeight,
      shaftBore:s.shaftBore, shoulderZ:s.shoulderZ, massKg:s.massKg, baseHoles:[for (h in s.baseHoles) {
      x:h.x, z:h.z, diameter:h.diameter
    }
    ], auxiliaryHoles:[], underhang:0, bearing:s.bearing,
      bearingArrangement:s.bearingArrangement, maxAxialLoadKgf:s.maxAxialLoadKgf, oppositeSupport:s.oppositeSupport,
      endProfile:s.endProfile, geometryBasis:s.geometryBasis, bearingCentreZ:null, boltSeatY:null, cadSha256:null};
    // The BK12 STEP's datums belong to the corrected drawing revision; older revisions keep their drawing's interfaces.
    var reviewed = HzMotionCatalog.cadModels().indexOf(model) >= 0
      && (model != "M60D-BK12-C" || s.drawingRevision == HzMotionCatalog.revision());
    if (reviewed) {
      var asset = HzMotionCatalog.cadAsset(model);
      spec.cadSha256 = asset.sha256;
      if (asset.interfaces != null) {
        spec.width = asset.interfaces.width;
        spec.baseHoles = [for (h in asset.interfaces.baseHoles) {
          x:h.x, z:h.z, diameter:h.diameter
        }
        ];
        spec.bearingCentreZ = asset.interfaces.bearingCentreZ;
        spec.boltSeatY = asset.interfaces.boltSeatY;
      }
    }
    return spec;
  }

  static function rbkData(model:String, revision:String):Dynamic {
    if (rbkCache == null) {
      var data:Dynamic = haxe.Json.parse(RbkMountData.text());
      rbkCache = data.interfaces;
    }
    for (entry in rbkCache) if (entry.model == model && entry.revision == revision) return entry;
    throw "Unknown pinned RBK interface revision";
  }

  /** Vallder's RBK table weights, kg, bearings included (https://vallder.com/product/rbk-motor-bracket/). */
  static function rbkWeight(model:String):Float return switch model {
    case "RBK12-57":
      1.00;
    case "RBK15-57":
      1.20;
    case "RBK15-86":
      1.55;
    case "RBK17-86":
      2.65;
    case _:
      throw 'No published weight for $model';
  };

  /** RBK brackets publish their interfaces and CAD, not body dimensions: the nominal body follows the NEMA face. */
  static function rbkRow(model:String, revision:String):MotorMountSpec {
    var e = rbkData(model, revision), face = StringTools.endsWith(model, "-86") ? 86.0 : 57.0;
    var holes:Array<Dynamic> = e.base_holes,
      aux:Array<Dynamic> = e.auxiliary_holes, bolts:Array<Array<Float>> = e.motor_bolts_xy;
    var sh:Float = e.shaft_height_mm, deepest = 0.0;
    function hole(h:Dynamic):MotorMountHole {
      var result:MotorMountHole = {x: h.x, z: h.z, diameter: h.diameter};
      if (h.top_y_mm != null) result.topY = h.top_y_mm;
      if (h.counterbore_depth_mm != null) result.counterboreDepth = h.counterbore_depth_mm;
      if (h.counterbore_diameter != null) result.counterboreDiameter = h.counterbore_diameter;
      deepest = Math.max(deepest, h.z);
      return result;
    }
    var base = [for (h in holes) hole(h)], auxiliary = [for (h in aux) hole(h)];
    var length = deepest + 12, flange:Float = e.motor_flange_thickness_mm;
    var bolt0 = bolts[0], pcd = 2 * Math.sqrt(Math.pow(bolt0[0], 2) + Math.pow(bolt0[1] - sh, 2));
    return {model: model, maker: "Vallder", revision: revision,
      reference: e.drawing_url, family: "RBK", construction: "integrated", acceptsSupport: null, face: face,
      pilot: e.pilot_diameter_mm, pilotDepth: e.pilot_recess_depth_mm, pcd: pcd, motorBolts: [for (b in bolts) {
      x:b[0], y:b[1] - sh
    }
    ], thread:e.motor_thread,
      threadDepth:e.motor_thread_depth_mm, threadRootRadius:e.motor_thread_root_radius_mm, flange:flange, length:length,
      shaftHeight:sh, width:face, rearStart:flange, rearEnd:length, rearWidth:face * 0.6, rearHeight:sh + face * 0.3,
      shaftBore:e.shaft_diameter_mm, shoulderZ:null, massKg:rbkWeight(model), baseHoles:base, auxiliaryHoles:auxiliary,
      underhang:e.underhang_mm, bearing:e.bearing, bearingArrangement:null, maxAxialLoadKgf:null,
      oppositeSupport:null, endProfile:null, geometryBasis:"pinned CAD interfaces; nominal body from the NEMA face",
      bearingCentreZ:null, boltSeatY:null, cadSha256:e.asset_sha256};
  }

  /** The drawings a row offers: nominal, and its maker's pinned CAD where reviewed. */
  public static function drawings(spec:MotorMountSpec):Array<String> return spec.cadSha256 == null ? [NOMINAL] : [
    NOMINAL,
    spec.maker.toLowerCase()
  ];

  function new(model:String, revision:String, drawing:String, bearingGrade:String) {
    var s = row(model, revision);
    if (drawings(s).indexOf(drawing) < 0) throw 'No "$drawing" drawing of motor mount $model';
    if (["unspecified", "C3", "C4", "C5"].indexOf(bearingGrade) < 0) throw "Unknown bearing grade";
    if (bearingGrade != "unspecified"
      && (s.family == "A" || s.maker != "HZMotion")) throw "Part has no selectable fixed-end bearing grade";
    super(
      '${s.maker.toUpperCase()}-$model' + (bearingGrade == "unspecified" ? "" : '-$bearingGrade'),
      '${s.maker} $model motor mount',
      "unspecified"
    );
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Catalog").setToken("errorsBasis", "Assumed").setToken("source", s.reference)));
    lockManufacturingMaterial("unspecified");
    spec = s;
    this.drawing = drawing;
    this.bearingGrade = bearingGrade;
    this.model = model;
    shaftHeight = s.shaftHeight;
    pilotDiameter = s.pilot;
    underhang = s.underhang;
    var h = s.shaftHeight;
    // The base plane under the motor face: +Y (a connector's axis) up into the mount.
    addConnector("base", Mount, F.alongY(0, 0, 0, 0, 1, 0));
    addConnector("motor", Mount, Solids.axial(0, h, 0));
    addConnector("shaftAxis", Axis, Solids.axial(0, h, 0));
    // The fixed bearing pair's centre on the screw axis, where the CAD is measured.
    if (s.bearingCentreZ != null && s.construction == "integrated") addConnector("bearingAxis", Axis, Solids.axial(0, h, s.bearingCentreZ));
    for (i in 0...s.motorBolts.length) addConnector(
      'motorBolt${i + 1}',
      Face,
      Solids.axial(s.motorBolts[i].x, h + s.motorBolts[i].y, 0)
    );
    for (i in 0...s.baseHoles.length) addConnector(
      'baseBolt${i + 1}',
      Face,
      F.alongY(s.baseHoles[i].x, 0, s.baseHoles[i].z, 0, 1, 0)
    );
    for (i in 0...s.auxiliaryHoles.length) addConnector(
      'baseAux${i + 1}',
      Face,
      F.alongY(s.auxiliaryHoles[i].x, 0, s.auxiliaryHoles[i].z, 0, 1, 0)
    );
    if (s.shoulderZ != null) addConnector("screwShoulder", Axis, Solids.axial(0, h, s.shoulderZ));
    if (s.family == "A") addConnector("fixedSupport", Mount, F.alongY(0, h, s.rearEnd, 0, 0, -1));
  }

  public function motorThread():String return spec.thread;

  /** The face the motor bolts to, as `MotorFlanges` compares it. */
  public function receivingFace():MotorFlanges.MotorReceivingFace return {
    pilot: spec.pilot,
    pilotDepth: spec.pilotDepth,
    bolts: spec.motorBolts,
    thread: spec.thread,
    threadDepth: spec.threadDepth
  };

  /** Whether `flange` fits this mount nominally; empty when it does. */
  public function findings(flange:Null<MotorFlange>):Array<String> return MotorFlanges.findings(receivingFace(),
    flange);

  /** Drawing value only; null where absent. Not a measured effective thread engagement. */
  public function motorThreadDepth():Null<Float> return spec.threadDepth;

  /** Fixed-bearing centre below the motor face, measured on the pinned STEP. */
  public function bearingCentreZ():Float {
    if (spec.bearingCentreZ == null) throw '$model has no measured bearing centre';
    return spec.bearingCentreZ;
  }

  /** Base-bolt counterbore seat above the mounting base, measured on the pinned STEP. */
  public function boltSeatY():Float {
    if (spec.boltSeatY == null) throw '$model has no measured bolt seat';
    return spec.boltSeatY;
  }

  override public function massProperties():MassProperties {
    var published = spec.massKg;
    if (published
      == null) throw 'Published assembly mass unavailable for $model; do not assign a homogeneous material to its bearings';
    // Catalogue mass, with the nominal drawing's centroid; no inertia declaration.
    var p = nominal();
    try {
      var centre = p.massProperties().centerOfMass;
      p.close();
      declareMass(published, centre);
    } catch (e:Dynamic) {
      p.close();
      throw e;
    }
    return super.massProperties();
  }

  /** Axial rigidity of the integrated fixed bearing pair, N/mm: the catalogue-typical assumed value (the maker publishes none); null for a mount without one. */
  public function axialStiffness():Null<Float> return spec.construction == "integrated" && spec.bearingCentreZ != null ? LeadScrew.SUPPORT_BEARING_STIFFNESS : null;

  /** The seat the mount's base holes tap into: where its feet stand, y = 0 in the mount's frame. */
  override public function mountingFace():Null<machinekit.component.MountingFace> return machinekit.component.MountingFace
    .tappedFoot([for (hole in spec.baseHoles) {x: hole.x, z: hole.z, diameter: hole.diameter}], 0);

  /**
   * The faces the mount is bolted by. `motor`: the flange face the servo's flange screws thread into (the row's bolt pattern, thread
   * and measured thread depth, in the mount's material). `foot`: the base holes, clearance holes the screws pass, bearing on the
   * bolt seat where the reviewed CAD measured it (none otherwise: a mount whose seat is not measured is not derived).
   */
  override public function fasteningFaces():Array<machinekit.component.BoltedFace> {
    var faces:Array<machinekit.component.BoltedFace> = [];
    var depth = spec.threadDepth ?? 2 * machinekit.standard.SocketHeadCapScrew.catalog().get(spec.thread).diameter;
    var material = materialSpec().toLowerCase().indexOf("alumin") >= 0 ? machinekit.component.BoltedFace.ThreadMaterial.Aluminium : machinekit.component.BoltedFace.ThreadMaterial.Steel;
    faces.push(machinekit.component.BoltedFace.tapped(
      "motor",
      machinekit.component.BoltedFace.frameOf({x: 0, y: spec.shaftHeight, z: 0}, {x: 0, y: 0, z: 1}, {x: 1, y: 0, z: 0}),
      [for (b in spec.motorBolts) {x: b.x, y: b.y}],
      spec.thread,
      depth,
      Math.max(depth, spec.flange),
      false,
      material
    ));
    var seat = spec.boltSeatY;
    if (seat != null && spec.baseHoles.length > 0) {
      var hole = spec.baseHoles[0], bolt = machinekit.component.MountingFace.boltFor(hole.diameter);
      var recess = hole.counterboreDiameter != null && hole.counterboreDepth != null ? {diameter: hole.counterboreDiameter, depth: hole.counterboreDepth} : null;
      faces.push(machinekit.component.BoltedFace.clamped(
        "foot",
        machinekit.component.BoltedFace.frameOf({x: 0, y: 0, z: 0}, {x: 0, y: 1, z: 0}, {x: 1, y: 0, z: 0}),
        // The face's y axis runs along -Z of the mount.
        [for (h in spec.baseHoles) {x: h.x, y: -h.z}],
        hole.diameter,
        seat,
        bolt.name,
        recess
      ));
    }
    return faces;
  }

  override public function carriesJournal():Bool return true;

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    if (drawing == NOMINAL) return nominal();
    var exterior = supplierCad();
    if (detail != Envelope) return exterior;
    var b = exterior.shape.bounds(), lo = b.get_min(), hi = b.get_max();
    exterior.close();
    return Part.prism(
      [
        new Vector(lo.get_x(), lo.get_y()),
        new Vector(hi.get_x(), lo.get_y()),
        new Vector(hi.get_x(), hi.get_y()),
        new Vector(lo.get_x(), hi.get_y())
      ],
      lo.get_z(),
      hi.get_z()
    );
  }

  function supplierCad():Part {
    if (spec.maker == "HZMotion") {
      var asset = HzMotionCatalog.cadAsset(model);
      var imported = new SupplierCadComponent(
        SupplierLibraries.asset(SupplierLibraries.hzmotion(), asset.sha256),
        asset.sha256,
        designation,
        description
      );
      try {
        var raw = imported.geometry();
        imported.close();
        var t = asset.translation, placed = machinekit.assembly.PosedParts.moved(
          raw,
          F.fromRotationMatrix(t[0], t[1], t[2], asset.rotation)
        );
        raw.close();
        return placed;
      } catch (e:Dynamic) {
        imported.close();
        throw e;
      }
    }
    var e = rbkData(model, spec.revision), path = SupplierLibraries.asset(SupplierLibraries.vallder(), spec.cadSha256);
    if (!sys.FileSystem.exists(path)) throw 'Missing pinned RBK CAD asset $path';
    var imported = new SupplierCadComponent(path, spec.cadSha256, designation, description);
    try {
      var raw = imported.geometry();
      imported.close();
      var first = e.raw_orientation == "x_shaft_y_up";
      var location = new Location(new Plane(
        new Vector(0, first ? -e.raw_base_y : 0, -e.raw_motor_face_x),
        new Vector(0, 0, 1),
        first ? new Vector(-1, 0, 0) : new Vector(0, 1, 0)
      ));
      var normalized = raw.placed(location);
      raw.close();
      return normalized;
    } catch (error:Dynamic) {
      imported.close();
      throw error;
    }
  }

  /** The body from its row, by how it is built: each starts from the motor flange plate and ends in its rear block. */
  function nominal():Part return Solids.building([], tracked -> {function keep(p: Part): Part {
    tracked.push(p);
    return p;
  }
  var s = spec, h = s.shaftHeight, pieces:Array<Part> = [], cuts:Array<Part> = [];
  // The motor flange plate. The flange may hang below the base plane (RBK's underhang): its seat needs relief.
  pieces.push(keep(s.construction == "module" ? box(s.width, 0, s.rearHeight, 0, s.flange) : box(
    s.face,
    h - s.face / 2,
    h + s.face / 2,
    0,
    s.flange
  )));
  // Side webs from the flange to the rear block; undimensioned on seat and module bodies and on RBK (8 mm assumed).
  var web = (s.width - s.rearWidth) / 2;
  if (s.construction != "integrated" || s.family == "RBK" || !(web > 0)) web = 8;
  var inner = s.width / 2 - web;
  for (side in [-1, 1]) pieces.push(keep(box(
    web,
    0,
    s.construction == "module" ? 10 : 24,
    s.flange,
    s.length,
    side * (inner + web / 2)
  )));
  pieces.push(keep(box(s.rearWidth, 0, s.rearHeight, s.rearStart, s.rearEnd)));
  cuts.push(keep(Part.cylinderSpan(s.pilot / 2, -1, s.flange + 0.01, 0, h)));
  cuts.push(keep(Part.cylinderSpan(s.shaftBore / 2, s.rearStart - 1, s.rearEnd + 1, 0, h)));
  // A seat body's pocket takes its flange support's locating boss.
  if (s.construction == "seat") cuts.push(keep(Part.cylinderSpan(seatRadius(), s.rearStart - 1, s.rearEnd + 1, 0, h)));
  var thread = Std.parseFloat(s.thread.substr(1)),
    root = s.threadRootRadius != null ? s.threadRootRadius : thread == 6 ? 2.5 : thread == 5 ? 2.1 : 1.65;
  for (b in s.motorBolts) cuts.push(keep(Part.cylinderSpan(root, -1, s.flange + 1, b.x, h + b.y)));
  for (hole in s.baseHoles) cuts.push(keep(vertical(hole.diameter / 2, -1, s.rearHeight + 1, hole.x, hole.z)));
  var all = keep(Solids.union(pieces));
  return Solids.cut(all, cuts);
  });

  /** The seat pocket's radius: the accepted support's locating pilot, from its maker's drawing. */
  function seatRadius():Float {
    var support = spec.acceptsSupport;
    if (support == null) throw '$model accepts no support';
    return HzMotionCatalog.get(support).pilot / 2;
  }

  static function box(width:Float, y0:Float, y1:Float, z0:Float, z1:Float, x:Float = 0):Part {
    if (!(width > 0) || !(y1 > y0) || !(z1 > z0)) throw 'Invalid motor mount box: $width $y0 $y1 $z0 $z1';
    return Part.prism(
      [
        new Vector(x - width / 2, y0),
        new Vector(x + width / 2, y0),
        new Vector(x + width / 2, y1),
        new Vector(x - width / 2, y1)
      ],
      z0,
      z1
    );
  }

  static function vertical(radius:Float, y0:Float, y1:Float, x:Float, z:Float):Part {
    var raw = Part.cylinderSpan(radius, y0, y1);
    try {
      var p = raw.placed(new Location(new Plane(new Vector(x, 0, z), Vector.X(), Vector.Y())));
      raw.close();
      return p;
    } catch (e:Dynamic) {
      raw.close();
      throw e;
    }
  }

  static var recipe:Null<ComponentType>;

  public static function recipeType():ComponentType {
    if (recipe == null) recipe = new ComponentType("machinekit.motion.motor-mount", [
      P.choice("model", models(), "M60D-BK12-C"),
      P.choice("revision", revisions(), HzMotionCatalog.revision()),
      P.choice("bearingGrade", ["unspecified", "C3", "C4", "C5"], "unspecified"),
      P.choice("geometry", [NOMINAL, "hzmotion", "vallder"], NOMINAL)
    ], v -> {var model = v.token("model"), revision = v.token("revision");
    // A revision belongs to one maker's tables; the default follows the model.
    if ((rbkModels().indexOf(model) >= 0) != (revision == "vallder-rbk-v1")) revision = defaultRevision(model);
    return new MotorMount(model, revision, v.token("geometry"), v.token("bearingGrade"));
    }, true, false, ["geometry"], false).procured(_ -> Buy(null, null, null));
    return recipe;
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function values():ComponentValues return new ComponentValues().setToken("model", model).setToken(
    "revision",
    spec.revision
  ).setToken(
    "bearingGrade",
    bearingGrade
  ).setToken(
    "geometry",
    drawing
  );
}
