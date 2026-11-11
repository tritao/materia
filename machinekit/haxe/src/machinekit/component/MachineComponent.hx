package machinekit.component;

import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.InertiaTensor;
import machinekit.component.MassProperties.MassSource;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.project.MaterialLibrary;

/** Geometry generator, named connector frames, and a BOM line for one machine part.
 * `geometry()` returns a new owned Part in the component's CAD frame; the caller closes it.
 */
import machinekit.component.PartRecordings.RecordedHistory;

class MachineComponent {
  final facetList:Array<ComponentFacet> = [];

  /** The typed facts this part states about itself; look one kind up with its class's `of`. */
  public function facets():Array<ComponentFacet> return facetList.copy();

  public function coupling():Null<CouplingFacet> return CouplingFacet.of(this);

  /** Typed electrical endpoints, absent on unrelated components. */
  public function motorDrive():Null<machinekit.motion.MotorDrive> return null;
  public function axisMotor():Null<machinekit.motion.AxisMotor> return null;
  /** The flange this part bolts to a host with, when it has one; a motor's is its axis motor flange. */
  public function mountingFace():Null<MountingFace> {
    var motor = axisMotor();
    return motor == null ? null : new MountingFace(
      motor.flange.face,
      motor.flange.face,
      depth -> motor.mountingCutout(depth),
      null,
      motor.hostCutout()
    );
  }
  /**
	 * The faces where this part is bolted to another, with the holes they know (see `BoltedFace`). `JointFasteners` derives
	 * the fasteners of every pair of faces that meet in an assembly; a part with none is never fastened by derivation.
	 */
  public function fasteningFaces():Array<BoltedFace> return [];
  /** The faces where this part is located to another by dowel pins (see `PinnedFace`); `JointFasteners` derives the pins. */
  public function pinnedFaces():Array<machinekit.component.BoltedFace.PinnedFace> return [];

  /** The shoulders or keys that locate this part against a thrust in both directions (see `LocatingKey`); `JointSlip` counts them for the joint of the face they name. */
  public function locatingKeys():Array<machinekit.component.BoltedFace.LocatingKey> return [];

  var machining:Array<cadkit.parametric.HoleSpec> = [];

  /**
	 * Whether this part cuts the holes `JointFasteners` derives for it into its geometry: its `geometry()` ends with
	 * `machined(...)`. A part that lacks it cannot be a drilled-to-suit host (nor a clamped face with undrawn holes);
	 * deriving such a joint is refused rather than leaving the solid without the hole.
	 */
  public function machinable():Bool return isRecorded();

  /** Records the holes derived for this part, once; they are cut by `machined`. */
  public function machine(cuts:Array<cadkit.parametric.HoleSpec>):Void {
    if (!machinable()) throw 'Part "$designation" does not cut derived holes into its geometry';
    if (machining.length > 0) throw 'Part "$designation" is already machined';
    machining = cuts.copy();
    cachedBom = null;
    cachedMass = null;
    cachedMassMaterialId = null;
  }

  public function machinedCuts():Array<cadkit.parametric.HoleSpec> return machining.copy();

  /**
	 * The derived holes as a hole table (this part's frame): tapped, clearance, counterbore and dowel, from the mating plane. A tool starts 1 mm
	 * outside the plane (and a clearance hole ends 1 mm past the far face), so the table's mouth and depth are the hole's.
	 */
  public function derivedRouteHoles():Array<ManufacturingRoute.RouteHole> {
    var n = 0, rows:Array<ManufacturingRoute.RouteHole> = [];
    for (hole in machining) for (cut in cadkit.parametric.features.HolesFeature.cuts(hole)) {
      // The hole from its mouth; a recess from where it opens into the hole, out to the far face.
      var mouth = cut.recess ? cut.from : 0.0, depth = cut.recess ? hole.depth - cut.from : hole.depth;
      var p = materia.assembly.AssemblyFrames.transformPoint(hole.frame, hole.x, hole.y, mouth), q = materia.assembly.AssemblyFrames.transformVector(hole.frame, 0, 0, 1);
      rows.push({id: "J" + (++n), kind: cut.recess ? "bore" : hole.kind ?? "clearance", thread: cut.recess ? null : hole.thread, diameter: cut.diameter,
        nominal: cut.diameter, fit: cut.recess ? null : hole.fit, depth: depth, position: [p.x, p.y, p.z], direction: [q.x, q.y, q.z]});
    }
    return rows;
  }

  /** Geometry caches retain instance face labels and exact coordinates. A BOM
   * revision instead names the physical machining in the part frame, independent
   * of joint IDs, cut order and sub-picometre rigid-transform roundoff. Recipe
   * input identity remains exact. The1e-12 resolution is below CAD tolerance. */
  function machiningKey(forGeometry:Bool):String {
    var out:Array<String>=[];
    for(hole in machining) for(cut in cadkit.parametric.features.HolesFeature.cuts(hole)) {
      var kind=cut.recess?"counterbore":hole.kind, fit=cut.recess?null:hole.fit, through=cut.recess?null:hole.through;
      if(forGeometry)out.push([hole.frame.x,hole.frame.y,hole.frame.z,hole.frame.qx,hole.frame.qy,hole.frame.qz,hole.frame.qw,hole.x,hole.y,
        cut.diameter,cut.from,cut.to].map(v->ComponentNumberKey.encode(v)).join(",")+","+cut.name);
      else {
        var p=materia.assembly.AssemblyFrames.transformPoint(hole.frame,hole.x,hole.y,0),axis=materia.assembly.AssemblyFrames.transformVector(hole.frame,0,0,1);
        var values=[p.x,p.y,p.z,axis.x,axis.y,axis.z,cut.diameter,cut.from,cut.to];
        out.push(values.map(v->ComponentNumberKey.encode(Math.fround(v*1e12)/1e12)).join(",")+"|"+Std.string(kind)+"|"+Std.string(hole.thread)+"|"+Std.string(fit)+"|"+Std.string(through));
      }
    }
    if(!forGeometry)out.sort(Reflect.compare);
    return out.join(";");
  }

  /** `part` with the derived holes cut in (consumes it); the end of a machinable part's `geometry()`. */
  function machined(part:Part):Part {
    if (machining.length == 0) return part;
    // The tools a recorded `holes` feature cuts, named by hole, so a part built either way has the same holes and face names.
    return Solids.cut(part, cadkit.parametric.features.HolesFeature.tools(machining));
  }
  /**
	 * Whether a shaft turns in this part: its bearings (a screw support, a motor mount, a motor housing, a bearing block) carry
	 * a journal. A body that turns relative to the structure may touch such a part without it counting as a rubbing contact.
	 */
  public function carriesJournal():Bool return false;
  /**
	 * Whether this part is held by form rather than by a flat face: threaded onto a screw or into a hole (a nut, a levelling foot's
	 * adjuster) or clamped round a shaft (a coupling). The face-area rule for a fixed mate does not apply to it.
	 */
  public function fastensByForm():Bool return false;
  public function reducerSpec():Null<machinekit.motion.ReducerSpec> return null;
  public function motorDriver():Null<machinekit.motion.MotorDriver> return null;
  public function electricalSource():Null<machinekit.motion.ElectricalSource> return null;
  public function motorBrakeSpec():Null<machinekit.motion.MotorBrakeSpec> return null;
  /** Explicit contact assumptions for this part's generated simulation hull. */
  public function simulationContact():Null<materia.project.SceneContactSurface> return null;

  /** Typed fluid model when this component is a cylinder. */
  public function cylinderSpec():Null<machinekit.fluid.CylinderSpec> return null;

  /** Typed fluid model when this component is a directional valve. */
  public function valveSpec():Null<machinekit.fluid.ValveSpec> return null;

  /** Typed pressure data when this component supplies a fluid service. */
  public function pressureSource():Null<machinekit.fluid.PressureSource> return null;

  function addFacet(facet:ComponentFacet):Void {
    if (facet == null) throw 'Null facet on "$designation"';
    facet.check(this);
    if (Std.isOfType(facet,
      CouplingFacet) && coupling() != null) throw 'Duplicate coupling capability on "$designation"';
    facetList.push(facet);
    cachedBom = null;
  }

  /** Declare that this modeled piece is supplied with another purchase. */
  public function includedInPurchase(ownerPartNumber:String):Void {
    if (IncludedInPurchase.of(this) != null) throw 'Duplicate purchase owner on "$designation"';
    addFacet(new IncludedInPurchase(ownerPartNumber));
  }

  public final designation:String;
  /** Author-declared drawing identity, independent of the scene designation. */
  public var drawingNumber(default, null):Null<String>;
  /** Physical material changes go through setMaterial, so mechanically typed parts can enforce their class. */
  public var materialId(default, null):String;
  /** True when this component was built from explicit, non-catalog specifications. */
  public final codeOnly:Bool;
  public var bom(get, never):BomItem;
  public final description:String;
  var cachedBom:Null<BomItem>;
  final recipeInputs:Null<ComponentValues>;
  static var nextInstanceId = 0;
  /** Components are built on several threads (sizing runs in parallel): each takes its id under this lock. */
  static final instanceIdLock = new sys.thread.Mutex();

  /** Tells this component from every other, including ones built from the same recipe. */
  public final instanceId:Int;

  var manufacturingMaterial:Null<String>;
  var cachedMass:Null<MassProperties>;
  var cachedMassMaterialId:Null<String>;
  var declaredMass:Null<MassProperties>;
  /** Null for code-only parts and assemblies outside the v1 recipe registry. */
  public var type(get, never):Null<ComponentType>;
  final connectorList:Array<Connector> = [];
  final portList:Array<ComponentPort> = [];
  final bridgeList:Array<PortBridge> = [];
  final conversionList:Array<PortBridge> = [];

  function new(
    designation:String,
    description:String,
    ?material:String,
    codeOnly:Bool = false,
    ?recipeValues:ComponentValues
  ) {
    recipeInputs = recipeValues == null ? null : recipeValues.copy();
    instanceIdLock.acquire();
    instanceId = nextInstanceId++;
    instanceIdLock.release();
    if (designation == null || designation.length == 0) throw "Machine component needs a designation";
    if (designation.length > PartNumberLint.MAX_LENGTH)
      throw 'Designation exceeds ${PartNumberLint.MAX_LENGTH} characters: $designation';
    this.designation = designation;
    materialId = MaterialLibrary.fromSpec(material);
    this.codeOnly = codeOnly;
    this.description = description;
  }

  /** A made part declares its drawing identity once, independently of geometry. */
  function declareDrawingNumber(number:String):Void {
    if (number == null || StringTools.trim(number) == "" || drawingNumber != null)
      throw "A drawing number must be declared once and cannot be empty";
    drawingNumber = number;
    cachedBom = null;
  }

  public static function customDesignation(role:String, definingGeometry:Dynamic):String
    return PartNumbers.geometry("CUSTOM", role, PartNumbers.exactKey(definingGeometry));

  public function materialSpec():String return MaterialLibrary.require(materialId).physical.spec;

  /** Factories lock a process-specific material without changing ordinary component recipes. */
  public function lockManufacturingMaterial(spec:String):Void {
    if (spec == null || spec != materialSpec() || (manufacturingMaterial != null
      && manufacturingMaterial != spec)) throw "Manufacturing material lock differs from component material";
    manufacturingMaterial = spec;
  }

  public function setMaterial(spec:String):Void {
    if (manufacturingMaterial != null
      && spec != manufacturingMaterial) throw "Select a manufacturing variant to change this material";
    materialId = MaterialLibrary.fromSpec(spec);
    cachedBom = null;
    cachedMass = null;
    cachedMassMaterialId = null;
  }

  function get_bom():BomItem {
    if (cachedBom != null) return cachedBom;
    var recipe = type;
    var valuesKey = recipe == null ? null : recipe.bomKey(values());
    var intent = ProcurementIntent.of(this);
    var procurement = intent != null ? intent.value : recipe == null ? null : recipe.procurement(values());
    var requirements = intent != null ? intent.requirements() : recipe == null ? [] : recipe.requirements(values());
    var basis = engineeringMaturity();
    var maturity:Null<String> = basis == null ? null : Std.string(basis);
    cachedBom = {
      partNumber: recipe == null ? (drawingNumber == null ? designation : drawingNumber) : recipe.partNumber(this),
      description : description,
      quantity : 1,
      material : MaterialLibrary.require(materialId).physical.spec,
      typeId : recipe == null ? null : recipe.id,
      valuesKey : valuesKey,
      fingerprint : recipe == null ? machinedRevision(revisionFingerprint()) : BomRevision.fingerprint(recipe.bomKey(values()) + machinedSuffix(), recipe.generatorVersion),
      procurement : procurement,
      maturity : maturity,
      requirements : requirements
    };
    if (recipe != null && recipe.manufacturingFamily != null) {
      var family = recipe.manufacturingFamily;
      var variant = family.variant(values().token(family.variantParameter));
      cachedBom.familyId = family.id;
      cachedBom.manufacturingVariant = variant.id;
      cachedBom.manufacturingProcess = variant.process;
    }
    return cachedBom;
  }

  /** Rating maturity is supplied by engineering metadata when available. */
  public function engineeringMaturity():Null<machinekit.transmission.ValueBasis> {
    var metadata = EngineeringMetadata.of(this);
    return metadata == null ? null : metadata.ratingsBasis;
  }

  /** Code-only components override this when their generator inputs are declared. */
  public function revisionFingerprint():Null<String> return null;

  /**
	 * How this part is best looked at on its own, when its shape alone does not say (see `ComponentView.derive`);
	 * null leaves the choice to its bounds.
	 */
  public function presentationView():Null<ComponentView> return null;

  public function hasGeometry():Bool {
    if (isRecorded()) return true;
    try {
      var part = geometry(Preview);
      part.close();
      return true;
    } catch (_:NoGeometry) return false;
  }

  /**
   * The part's geometry as recorded steps: write them with `builder`, in the part's own frame, and return the body (without the
   * derived holes, which are added as a last step). A part that records gets its solid, its hole cuts and its scene history
   * from this, so it does not also override `geometry`. Null (the default) means the part does not record yet.
   */
  public function record(builder:cadkit.parametric.recording.DocumentBuilder, detail:ComponentDetail):Null<cadkit.parametric.Feature>
    return null;

  /** The part's recording at `detail`, derived holes included, or null when it does not record; the caller closes it. */
  public function recording(detail:ComponentDetail = Preview):Null<cadkit.parametric.Document> return PartRecordings.document(this, detail);

  var recordsGeometry:Null<Bool> = null;

  /** Whether `record` writes this part's geometry (asked once). */
  public function isRecorded():Bool {
    var known = recordsGeometry;
    if (known != null) return known;
    var probe = new cadkit.parametric.recording.DocumentBuilder();
    try {
      var records = record(probe, Preview) != null;
      probe.close();
      recordsGeometry = records;
      return records;
    } catch (error:Dynamic) {
      probe.close();
      throw error;
    }
  }

  static final recordedSolids = machinekit.cache.Memos.parts("recorded-geometry", 64);
  static final recordedHistories = new machinekit.cache.Memo<RecordedHistory>("recorded-history", 64);

  /**
   * What else a recording reads besides the recipe values, material and machining (a layout the part was made for), as stable
   * text; part of the key its solid and history are cached under. Empty when it reads nothing else.
   */
  public function recordingContext():String return "";

  function recordingKey(detail:ComponentDetail):String return geometryKey(detail) + "|recorded:" + recordingContext();

  /** The part's solid: from its recording when it records, evaluated once per recording key. */
  public function geometry(detail:ComponentDetail = Preview):Part {
    if (!isRecorded()) throw new NoGeometry(designation);
    return recordedSolids.get(recordingKey(detail), () -> {
      var document = recording(detail);
      if (document == null) throw new NoGeometry(designation);
      PartRecordings.solid(document);
    });
  }

  static final recordedDatumTables = new machinekit.cache.Memo<Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>>("recorded-datums", 64);

  /**
   * The named datum frames the recording declares (`DocumentBuilder.datum`), in the part's own frame: a datum's normal is the
   * frame's +Z, its x direction +X. Interfaces (bolted faces, mounts) are built from these, so they move with the geometry.
   */
  public function recordedDatums():Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> {
    if (!isRecorded()) return [];
    return recordedDatumTables.get(recordingKey(Preview), readDatums);
  }

  function readDatums():Map<String, materia.assembly.AssemblyRecord.AssemblyFrame> {
    var builder = new cadkit.parametric.recording.DocumentBuilder();
    try {
      record(builder, Preview);
      var frames = new Map<String, materia.assembly.AssemblyRecord.AssemblyFrame>();
      for (datum in builder.datums()) {
        var p = datum.plane;
        if (frames.exists(datum.name)) throw 'Recorded part "$designation" declares datum "${datum.name}" twice';
        frames.set(datum.name, BoltedFace.frameOf({x: p.origin.x, y: p.origin.y, z: p.origin.z}, {x: p.normal.x, y: p.normal.y, z: p.normal.z},
          {x: p.xDirection.x, y: p.xDirection.y, z: p.xDirection.z}));
      }
      builder.close();
      return frames;
    } catch (error:Dynamic) {
      builder.close();
      throw error;
    }
  }

  /** One recorded datum frame by name; throws naming the part when it is missing. */
  public function recordedDatum(name:String):materia.assembly.AssemblyRecord.AssemblyFrame {
    var frame = recordedDatums().get(name);
    if (frame == null) throw 'Recorded part "$designation" declares no datum "$name"';
    return frame;
  }

  /** The recording saved, with one row per top-level step and the faces it made, or null when the part does not record. */
  public function recordedHistory(detail:ComponentDetail = Preview):Null<RecordedHistory> {
    if (!isRecorded()) return null;
    return recordedHistories.get(recordingKey(detail), () -> {
      var document = recording(detail);
      if (document == null) throw new NoGeometry(designation);
      var steps = [for (row in cadkit.parametric.TimelineModel.of(document)) if (row.parent < 0)
        {id: row.id, label: row.group == "" ? row.label : row.group + " · " + row.label, faces: row.faces}];
      ({document: PartRecordings.encoded(document), steps: steps} : RecordedHistory);
    });
  }

  /**
	 * Names the geometry `geometry(detail)` builds, for a cache of it: the same for every component made from the same recipe
	 * inputs and material, which build the same shape, and for a component without a recipe (nothing says which components are
	 * alike) its own.
	 */
  /** The revision with the derived holes in it: the same part drilled differently is another revision. */
  function machinedRevision(revision:Null<String>):Null<String> return revision == null || machining.length == 0 ? revision : BomRevision.fingerprint(revision
    + machinedSuffix(), 1);

  function machinedSuffix(forGeometry:Bool=false):String return machining.length == 0 ? "" : "|machined:" + machiningKey(forGeometry);

  public function geometryKey(detail:ComponentDetail = Preview):String {
    var recipe = type, suffix = materialId + "|" + Std.string(detail);
    return recipe
      == null ? "instance|" + instanceId + "|" + suffix + machinedSuffix(true) : recipe.id + "|" + recipe.key(values()) + "|" + suffix
      + machinedSuffix(true);
  }

  /**
	 * How faithful the preview is. Derived where the geometry shows it: a preview that is one primitive solid (six
	 * faces or fewer: a box, a cylinder, a ring or a tube) is a placeholder of the part's size, `Envelope`; any other
	 * is `Nominal`. Parts dimensioned from a cited standard or datasheet declare `Datasheet`, imports of supplier CAD
	 * `Supplier`.
	 */
  public function geometryFidelity():GeometryFidelity {
    var preview:Part;
    try preview = geometry(Preview) catch (_:NoGeometry) return GeometryFidelity.Envelope;
    var faces = preview.shape.subshapeCount(CadKit.ShapeKind.Face);
    preview.close();
    return faces <= 6 ? GeometryFidelity.Envelope : GeometryFidelity.Nominal;
  }

  /**
	 * A purchased part whose housing is mostly air and electronics, not solid material, returns its product
	 * class's assumed effective density (kg/m³, `EffectiveDensities`): its mass is then its envelope's volume at
	 * that density, independent of the drawn preview and of its housing material. Null computes mass from the
	 * preview shape and the material's density.
	 */
  function effectiveDensity():Null<Float> return null;

  /**
	 * A simplified solid standing in for the part's mass, at its material's density, when the drawn preview should
	 * not decide it (a motor drawn in detail keeps the mass of its body); null uses the preview. The caller owns the
	 * returned part.
	 */
  public function massGeometry():Null<Part> return null;

  /**
	 * The volume, centroid and unit-density inertia of `geometry(Preview)`, computed exactly from the component's own
	 * representation without building the solid, or null when that representation cannot give them. A component returns
	 * them only where they equal integrating the preview solid, to rounding; `massProperties` then skips the solid.
	 */
  public function analyticVolume():Null<VolumeProperties> return null;

  /** The preview shape supplies volume and centroid; its density is kg/m³. */
  public function massProperties():MassProperties {
    if (declaredMass != null) return declaredMass;
    if (cachedMass != null && cachedMassMaterialId == materialId) return cachedMass;
    var effective = effectiveDensity();
    var basis:ComponentDetail = effective == null ? Preview : Envelope;
    var modelled = effective == null ? massGeometry() : null;
    var shareKey = type == null ? null : geometryKey(basis) + (modelled == null ? "" : "|mass-model");
    var computed = false;
    var mass = ComponentType.masses.get(shareKey, () -> {
      computed = true;
      return computedMass(effective, basis, modelled);
    });
    if (!computed && modelled != null) modelled.close();
    cachedMass = mass;
    cachedMassMaterialId = materialId;
    return mass;
  }

  /** `massProperties` worked out: analytically, else by integrating `modelled` or the geometry at `basis` (closing it). */
  function computedMass(effective:Null<Float>, basis:ComponentDetail, modelled:Null<Part>):MassProperties {
    var analytic = effective == null && modelled == null ? analyticVolume() : null;
    if (analytic != null) {
      var density = MaterialLibrary.require(materialId).physical.density;
      return new MassProperties(
        analytic.volume * 1e-9 * density,
        analytic.centre,
        Computed(Preview),
        analytic.inertia.scaled(density * 1e-9)
      );
    }
    var part:Part;
    if (modelled != null) {
      part = modelled;
    } else {
      try part = geometry(basis) catch (_:NoGeometry) throw 'Component "$designation" has no geometry or declared mass';
    }
    var result:MassProperties;
    try {
      var physical = part.massProperties();
      var density = effective == null ? MaterialLibrary.require(materialId).physical.density : effective;
      result = new MassProperties(
        physical.volume * 1e-9 * density,
        physical.centerOfMass,
        effective != null ? EffectiveDensity(effective) : modelled != null ? Modelled : Computed(Preview),
        physical.inertiaAtDensity(density * 1e-9)
      );
    } catch (error:Dynamic) {
      part.close();
      throw error;
    }
    part.close();
    return result;
  }

  /** Vendor mass overrides the geometry estimate, including after material changes. */
  function declareMass(kg:Float, centreOfMass:Vector, ?inertia:InertiaTensor):Void {
    if (!Math.isFinite(kg) || kg <= 0) throw 'Component "$designation" needs a positive declared mass';
    if (centreOfMass == null || !Math.isFinite(centreOfMass.x) || !Math.isFinite(centreOfMass.y)
      || !Math.isFinite(centreOfMass.z)) throw 'Component "$designation" needs a finite declared centre of mass';
    declaredMass = new MassProperties(kg, centreOfMass, Declared, inertia);
  }

  public function toolSpecs():Array<ToolSpec> return [];

  public function tool(name:String, values:ComponentValues):Part {
    for (spec in toolSpecs()) if (spec.name == name) return buildTool(name, spec.resolve(values));
    throw 'Unknown tool "$name" for "$designation"';
  }

  function buildTool(name:String, values:ComponentValues):Part throw 'Unknown tool "$name" for "$designation"';

  public function componentType():Null<ComponentType> return null;

  function get_type():Null<ComponentType> return componentType();

  /** Snapshot of constructor inputs, with the current physical material. Existing components
	 * may continue to compute their own values by overriding this method.
	 */
  public function values():ComponentValues {
    if (recipeInputs == null) throw 'Component "$designation" has no recipe values';
    var result = new ComponentValues(), recipe = componentType();
    for (name in recipeInputs.names()) if (name != "material") result.set(name, recipeInputs.get(name));
    if (recipe == null || recipe.materialEditable) result.setToken("material", materialSpec());
    return result;
  }

  /** Typed access to stored recipe inputs; geometry remains the component's responsibility. */
  public function parameterNumber(name:String):Float {
    if (recipeInputs == null) throw 'Component "$designation" has no recipe values';
    return recipeInputs.number(name);
  }
  public function parameterToken(name:String):String {
    if (recipeInputs == null) throw 'Component "$designation" has no recipe values';
    return recipeInputs.token(name);
  }
  public function parameterCount(name:String):Int {
    if (recipeInputs == null) throw 'Component "$designation" has no recipe values';
    return recipeInputs.integer(name);
  }

  public function connectors():Array<Connector> return connectorList.copy();

  public function connector(name:String):Connector {
    for (connector in connectorList) if (connector.name == name) return connector;
    throw 'Missing connector "$designation/$name"';
  }

  public function ports():Array<ComponentPort> return portList.copy();

  public function hasPort(name:String):Bool {
    for (entry in portList) if (entry.name == name) return true;
    return false;
  }

  public function port(name:String):ComponentPort {
    for (entry in portList) if (entry.name == name) return entry;
    throw 'Missing port "$designation/$name"';
  }

  public function bridges():Array<PortBridge> return bridgeList.copy();
  public function conversions():Array<PortBridge> return conversionList.copy();

  /** Adds an instance with every connector frame so it can be mated by name. */
  public function addTo(model:AssemblyModel, id:String, ?pose:AssemblyFrame):Void {
    model.add(id, pose);
    for (connector in connectorList) model.connector(id, connector.name, connector.frame);
  }

  function addConnector(name:String, role:ConnectorRole, frame:AssemblyFrame):Void {
    for (existing in connectorList) if (existing.name == name) throw 'Duplicate connector "$designation/$name"';
    connectorList.push({name: name, role: role, frame: frame});
  }

  function addPort(entry:ComponentPort):Void {
    if (entry == null || entry.name == null || entry.name.length == 0
      || entry.kind == null || entry.role == null || entry.iface == null) throw 'Invalid port on "$designation"';
    switch entry.iface {
      case PushIn(tubeOd):
        if (!Math.isFinite(tubeOd) || tubeOd <= 0) throw 'Invalid port interface "$designation/${entry.name}"';
      case Thread(name):
        if (name == null || name.length == 0) throw 'Invalid port interface "$designation/${entry.name}"';
      case Plug(name, pins):
        if (name == null || name.length == 0 || pins <= 0) throw 'Invalid port interface "$designation/${entry.name}"';
      case Coupling(key, channel):
        if (key == null || key.length == 0 || channel <= 0) throw 'Invalid port interface "$designation/${entry.name}"';
      case Unspecified:
    }
    for (existing in portList) if (existing.name == entry.name) throw 'Duplicate port "$designation/${entry.name}"';
    if (entry.connector != null) connector(entry.connector);
    portList.push(entry);
  }

  function addBridge(from:String, to:String):Void {
    var input = port(from), output = port(to);
    if (from == to
      || input.kind != output.kind) throw 'Port bridge "$designation/$from->$to" needs distinct ports of the same kind';
    if (input.role == Supply) throw 'Port bridge "$designation/$from->$to" cannot use a Supply inlet';
    for (existing in bridgeList) if (existing.from == from
      && existing.to == to) throw 'Duplicate port bridge "$designation/$from->$to"';
    bridgeList.push({from: from, to: to});
  }

  /** Declare a service conversion, such as pneumatic air producing vacuum. */
  function addConversion(from:String, to:String):Void {
    var input = port(from), output = port(to);
    if (from == to || input.role != Consumer || output.role
      != Supply) throw 'Port conversion "$designation/$from->$to" needs a Consumer input and Supply output';
    for (existing in conversionList) if (existing.to
      == to) throw 'Port conversion "$designation/$to" has multiple inputs';
    conversionList.push({from: from, to: to});
  }
}
