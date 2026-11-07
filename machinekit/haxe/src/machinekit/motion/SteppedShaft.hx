package machinekit.motion;

import CadKit;
import haxe.Json;
import machinekit.component.ComponentValues;
import machinekit.standard.ParallelKeySpec;
import cadkit.modeling.Axis;
import cadkit.modeling.Part;
import cadkit.modeling.Selection;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.ConnectorRole;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.standard.ParallelKey;
import machinekit.standard.BearingFit;
import machinekit.standard.BearingFit.BearingShaftFit;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

private typedef RecipeKey = {
  var size:String;
  var length:Float;
  var codeOnly:Bool;
  var spec:ParallelKeySpec;
}
private typedef RecipeKeyway = {
  var name:String;
  var z0:Float;
  var key:RecipeKey;
}

/** One constant-diameter length of a stepped shaft, in millimetres. */
typedef ShaftSection = {
  var diameter:Float;
  var length:Float;
}

/** Semantic thread specification for a machined shaft end. Thread flanks are not modelled. */
typedef ShaftThreadEnd = {
  var diameter:Float;
  var pitch:Float;
  var length:Float;
}

/** Detail applied at a section shoulder. The relief diameter is the finished root diameter. */
typedef ShaftShoulderDetail = {
  var z:Float;
  var ?fillet:Float;
  var ?reliefWidth:Float;
  var ?reliefDiameter:Float;
}

/** Optional manufacturing details for a stepped shaft. */
typedef ShaftDetail = {
  var ?inputChamfer:Float;
  var ?outputChamfer:Float;
  var ?inputThread:ShaftThreadEnd;
  var ?outputThread:ShaftThreadEnd;
  var ?shoulders:Array<ShaftShoulderDetail>;
}

/** A keyway cut for `key`, starting at `z0` and running the key's length.
 * Adds a `name` connector at the seat floor, mid-keyway, for mating the key with a fixed joint.
 */
typedef ShaftKeyway = {
  var name:String;
  var z0:Float;
  var key:ParallelKey;
}

/** A full-depth retaining-ring groove from `z0` to `z0 + width`, cut to `diameter`.
 * A `name` connector (axis, groove mid-width) is added when given, for mating a `RetainingRing`.
 */
typedef ShaftGroove = {
  var ?name:String;
  var z0:Float;
  var width:Float;
  var diameter:Float;
}

/** Coaxial cylindrical sections stacked along +Z, producing square shoulders at each diameter
 * change. CAD frame: input end at z=0, output end at z=totalLength.
 * Connectors: `input` (axis, z=0), `output` (shaft, z=totalLength), plus any `namedFaces` (axis,
 * for bearing or collar seats), one connector per named keyway, and one per named groove. Keyways
 * are cut on the shaft's local +Y side, so a `ParallelKey` mated through the keyway's connector
 * sits flush in the slot; groove connectors are axial, for a `RetainingRing`.
 */
class SteppedShaft extends MachineComponent {
  public final sections:Array<ShaftSection>;
  public final totalLength:Float;
  final namedFaces:Array<{name:String, z:Float}>;
  final keyways:Array<ShaftKeyway>;
  final grooves:Array<ShaftGroove>;
  public final detail:ShaftDetail;

  public function new(
    sections:Array<ShaftSection>,
    ?namedFaces:Array<{name:String, z:Float}>,
    ?keyways:Array<ShaftKeyway>,
    ?grooves:Array<ShaftGroove>,
    ?detail:ShaftDetail
  ) {
    if (sections.length == 0) throw "Stepped shaft needs at least one section";
    for (section in sections) if (!(section.diameter > 0)
      || !(section.length > 0)) throw "Stepped shaft section needs a positive diameter and length";
    var total = 0.0;
    for (section in sections) total += section.length;
    var sizes = [for (section in sections) '${Dimension.format(section.diameter)}x${Dimension.format(section.length)}'];
    var resolvedDetail:ShaftDetail = canonicalDetail(detail);
    var resolvedKeyways = keyways == null ? [] : keyways.copy();
    var resolvedGrooves = grooves == null ? [] : grooves.copy();
    var resolvedFaces = namedFaces == null ? [] : namedFaces.copy();
    resolvedFaces.sort((a, b) -> Reflect.compare(Json.stringify(a), Json.stringify(b)));
    resolvedKeyways.sort((a, b) -> Reflect.compare(a.name, b.name));
    resolvedGrooves.sort((a, b) -> Reflect.compare(Json.stringify(a), Json.stringify(b)));
    super(
      geometryNumber(sections, resolvedFaces, resolvedKeyways, resolvedGrooves, resolvedDetail),
      "Stepped shaft " + sizes.join(" / ") + detailDescription(resolvedDetail),
      "steel C45"
    );
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.sections = sections.copy();
    this.namedFaces = resolvedFaces;
    totalLength = total;
    this.detail = resolvedDetail;
    validateDetail();
    addConnector("input", Axis, Solids.axial(0, 0, 0));
    addConnector("output", Shaft, Solids.axial(0, 0, total));
    if (resolvedFaces.length > 0) for (face in resolvedFaces) {
      if (face.z < 0 || face.z > total) throw 'Face "${face.name}" lies outside the shaft';
      addConnector(face.name, Face, Solids.axial(0, 0, face.z));
    }
    this.keyways = resolvedKeyways;
    for (keyway in this.keyways) {
      var end = keyway.z0 + keyway.key.length;
      if (keyway.z0 < 0 || end > total) throw 'Keyway "${keyway.name}" lies outside the shaft';
      if (!withinOneSection(keyway.z0, end)) throw 'Keyway "${keyway.name}" must lie within one shaft section';
      var radius = diameterAt(keyway.z0) / 2;
      if (!(keyway.key.spec.shaftDepth < radius)) throw 'Keyway "${keyway.name}" is deeper than the shaft radius';
      if (!(keyway.key.spec.width < radius)) throw 'Keyway "${keyway.name}" is too wide for the shaft';
      addConnector(
        keyway.name,
        Face,
        Solids.axial(0, radius - keyway.key.spec.shaftDepth, keyway.z0 + keyway.key.length / 2)
      );
    }
    this.grooves = resolvedGrooves;
    for (groove in this.grooves) {
      if (!(groove.width > 0)) throw "Retaining ring groove needs a positive width";
      if (groove.z0 < 0 || groove.z0 + groove.width > total) throw "Retaining ring groove lies outside the shaft";
      if (!withinOneSection(groove.z0,
        groove.z0 + groove.width)) throw "Retaining ring groove must lie within one shaft section";
      if (!(groove.diameter > 0) || !(groove.diameter
        < diameterAt(groove.z0))) throw "Retaining ring groove diameter must be smaller than the shaft";
      if (groove.name != null) addConnector(groove.name, Axis, Solids.axial(0, 0, groove.z0 + groove.width / 2));
    }
    var inputThread = resolvedDetail.inputThread, outputThread = resolvedDetail.outputThread;
    if (inputThread != null) addConnector("inputThread", Shaft, Solids.axial(0, 0, inputThread.length / 2));
    if (outputThread != null) addConnector("outputThread", Shaft, Solids.axial(0, 0, total - outputThread.length / 2));
  }

  /** Diameter of the section containing `z`; a boundary belongs to the following section. */
  public function diameterAt(z:Float):Float {
    if (z < 0 || z > totalLength) throw 'Shaft position $z is outside 0..$totalLength';
    var start = 0.0;
    for (i in 0...sections.length) {
      var section = sections[i];
      var end = start + section.length;
      if (z < end || i == sections.length - 1) return section.diameter;
      start = end;
    }
    throw 'Shaft position $z is outside 0..$totalLength';
  }

  /** Finished journal diameter at `z` for a named shaft fit. Allowance is diametral. */
  public function journalDiameterAt(z:Float,
    fit:BearingShaftFit = Slip):Float return diameterAt(z) + BearingFit.shaftAllowance(
      fit,
      diameterAt(z)
    );

  /** Finished journal diameter at `z` with an explicit diametral allowance. */
  public function journalDiameterAllowance(z:Float, allowance:Float):Float {
    var result = diameterAt(z) + allowance;
    if (!(result > 0) || !Math.isFinite(result)) throw "Shaft journal allowance leaves no shaft diameter";
    return result;
  }

  /** True when no shoulder lies strictly inside z0..z1. */
  function withinOneSection(z0:Float, z1:Float):Bool {
    var boundary = 0.0;
    for (i in 0...sections.length - 1) {
      boundary += sections[i].length;
      if (boundary > z0 + 1e-6 && boundary < z1 - 1e-6) return false;
    }
    return true;
  }

  function validateDetail():Void {
    validateChamfer(optionalFloat(detail.inputChamfer), diameterAt(0), "input");
    validateChamfer(optionalFloat(detail.outputChamfer), diameterAt(totalLength), "output");
    var inputThread = detail.inputThread, outputThread = detail.outputThread;
    if (inputThread != null) validateThread(inputThread, diameterAt(0), sections[0].length, "input");
    if (outputThread != null) validateThread(
      outputThread,
      diameterAt(totalLength),
      sections[sections.length - 1].length,
      "output"
    );
    var shoulders = detail.shoulders == null ? [] : detail.shoulders;
    for (i in 0...shoulders.length) {
      var shoulder = shoulders[i];
      if (shoulderIndex(shoulder.z) < 0) throw 'Shoulder detail z=${shoulder.z} is not a section boundary';
      var prior = diameterAt(shoulder.z - 1e-5), next = diameterAt(shoulder.z + 1e-5);
      var smaller = Math.min(prior, next);
      var stepHeight = Math.abs(prior - next) / 2;
      var fillet = optionalFloat(shoulder.fillet);
      if (shoulder.fillet != null
        && (!(fillet > 0) || fillet >= stepHeight)) throw 'Shoulder fillet at z=${shoulder.z} is too large';
      if (shoulder.reliefWidth != null) {
        var width = optionalFloat(shoulder.reliefWidth);
        if (!(width > 0) || width >= 2 * Math.min(sectionLengthBefore(shoulder.z),
          sectionLengthAfter(shoulder.z))) throw 'Shoulder relief at z=${shoulder.z} has invalid width';
        var reliefDiameter = optionalFloat(shoulder.reliefDiameter);
        if (shoulder.reliefDiameter == null || !(reliefDiameter > 0)
          || reliefDiameter >= smaller) throw 'Shoulder relief at z=${shoulder.z} has invalid diameter';
      }
      for (j in 0...i) if (Math.abs(shoulders[j].z - shoulder.z)
        < 1e-6) throw 'Duplicate shoulder detail at z=${shoulder.z}';
    }
  }

  function validateChamfer(value:Float, diameter:Float, end:String):Void {
    if (value > 0 && value >= diameter / 2) throw 'Shaft $end chamfer must be positive and smaller than its radius';
    if (value < 0 || !Math.isFinite(value)) throw 'Shaft $end chamfer must be positive';
  }

  function validateThread(thread:ShaftThreadEnd, diameter:Float, sectionLength:Float, end:String):Void {
    if (!(thread.diameter > 0) || thread.diameter >= diameter
      || !Math.isFinite(thread.diameter)) throw 'Shaft $end thread diameter must be below the shaft diameter';
    if (!(thread.pitch > 0) || !Math.isFinite(thread.pitch)) throw 'Shaft $end thread pitch must be positive';
    if (!(thread.length > 0) || thread.length > sectionLength
      || !Math.isFinite(thread.length)) throw 'Shaft $end thread must fit within its end section';
  }

  function shoulderIndex(z:Float):Int {
    var boundary = 0.0;
    for (i in 0...sections.length - 1) {
      boundary += sections[i].length;
      if (Math.abs(boundary - z) < 1e-6) return i;
    }
    return -1;
  }

  function sectionLengthBefore(z:Float):Float return sections[shoulderIndex(z)].length;

  function sectionLengthAfter(z:Float):Float return sections[shoulderIndex(z) + 1].length;

  static function geometryNumber(sections:Array<ShaftSection>, faces:Array<{name:String, z:Float}>,
    keyways:Array<ShaftKeyway>, grooves:Array<ShaftGroove>, detail:ShaftDetail):String {
    function number(value:Null<Float>):String return machinekit.component.ComponentNumberKey.encode(value == null ? 0 : value);
    function thread(value:Null<ShaftThreadEnd>):String return value == null ? "none"
      : [number(value.diameter), number(value.pitch), number(value.length)].join(":");
    var features:Array<String> = [];
    for (face in faces) features.push("face:" + face.name.length + ":" + face.name + ":" + number(face.z));
    for (entry in keyways) {
      var spec = entry.key.spec;
      features.push("key:" + entry.name.length + ":" + entry.name + ":" + [number(entry.z0),
        number(entry.key.length), number(spec.minShaft), number(spec.maxShaft), number(spec.width),
        number(spec.height), number(spec.shaftDepth), number(spec.hubDepth)].join(":"));
    }
    for (entry in grooves) features.push("groove:" + (entry.name == null ? "" : entry.name.length + ":" + entry.name)
      + ":" + [number(entry.z0), number(entry.width), number(entry.diameter)].join(":"));
    if (detail.shoulders != null) for (entry in detail.shoulders) features.push("shoulder:" +
      [number(entry.z), number(entry.fillet), number(entry.reliefWidth), number(entry.reliefDiameter)].join(":"));
    var featured = features.length > 0 || optionalFloat(detail.inputChamfer) != 0
      || optionalFloat(detail.outputChamfer) != 0 || detail.inputThread != null || detail.outputThread != null;
    var readable = "SHAFT-" + [for (section in sections)
      Dimension.format(section.diameter) + "x" + Dimension.format(section.length)].join("-");
    if (!featured && readable.length <= machinekit.component.PartNumberLint.MAX_LENGTH) return readable;
    features.sort(Reflect.compare);
    var sectionKeys = [for (section in sections) number(section.diameter) + ":" + number(section.length)];
    var key = haxe.Json.stringify({sections:sectionKeys, features:features, inputChamfer:number(detail.inputChamfer),
      outputChamfer:number(detail.outputChamfer), inputThread:thread(detail.inputThread), outputThread:thread(detail.outputThread)});
    return machinekit.component.PartNumbers.geometry("SHAFT", "STEPPED", key);
  }

  static function canonicalDetail(value:Null<ShaftDetail>):ShaftDetail {
    if (value == null) return emptyDetail();
    var shoulders:Array<ShaftShoulderDetail> = value.shoulders == null ? [] : [for (entry in value.shoulders) {
      z:entry.z, fillet:entry.fillet, reliefWidth:entry.reliefWidth, reliefDiameter:entry.reliefDiameter
    }];
    shoulders.sort((a, b) -> a.z < b.z ? -1 : a.z > b.z ? 1 : 0);
    return {inputChamfer:optionalFloat(value.inputChamfer) == 0 ? null : value.inputChamfer,
      outputChamfer:optionalFloat(value.outputChamfer) == 0 ? null : value.outputChamfer,
      inputThread:value.inputThread, outputThread:value.outputThread, shoulders:shoulders.length == 0 ? null : shoulders};
  }

  static function emptyDetail():ShaftDetail return {
    inputChamfer: null,
    outputChamfer: null,
    inputThread: null,
    outputThread: null,
    shoulders: null
  };

  static function optionalFloat(value:Null<Float>):Float return value == null ? 0.0 : value;

  static function detailDescription(detail:ShaftDetail):String {
    var result = "";
    if (detail.inputChamfer != null || detail.outputChamfer != null) result += ", chamfered ends";
    if (detail.inputThread != null || detail.outputThread != null) result += ", semantic threaded ends";
    if (detail.shoulders != null && detail.shoulders.length > 0) result += ", detailed shoulders";
    return result;
  }

  public function recipeValues():ComponentValues {
    var keyData = [for (entry in keyways) {
      name:entry.name, z0:entry.z0, key: {
        size: '${Dimension.format(entry.key.spec.width)}x${Dimension.format(entry.key.spec.height)}',
        length: entry.key.length,
        codeOnly: entry.key.codeOnly,
        spec: entry.key.spec
      }
    }
    ];
    return new ComponentValues().setToken("sections", Json.stringify(sections)).setToken(
      "faces",
      Json.stringify(namedFaces)
    ).setToken(
      "keyways",
      Json.stringify(keyData)
    ).setToken(
      "grooves",
      Json.stringify(grooves)
    ).setToken(
      "shaftDetail",
      Json.stringify(detail)
    ).setToken(
      "material",
      materialSpec()
    );
  }

  static function rows(text:String):Array<Dynamic> return Json.parse(text);
  static function field(object:Dynamic, name:String):Dynamic return Reflect.field(object, name);

  public static function fromRecipe(values:ComponentValues):SteppedShaft {
    var sections:Array<ShaftSection> = [for (row in rows(values.token("sections"))) {
      diameter:cast field(row, "diameter"), length:cast field(row, "length")
    }
    ];
    var faces:Array<{name:String, z:Float}> = [for (row in rows(values.token("faces"))) {
      name:cast field(row, "name"), z:cast field(row, "z")
    }
    ];
    var rebuilt:Array<ShaftKeyway> = [];
    for (entry in rows(values.token("keyways"))) {
      var keyData = field(entry, "key");
      var specData = field(keyData, "spec");
      var spec:ParallelKeySpec = {
        minShaft: cast field(specData, "minShaft"),
        maxShaft: cast field(specData, "maxShaft"),
        width: cast field(specData, "width"),
        height: cast field(specData, "height"),
        shaftDepth: cast field(specData, "shaftDepth"),
        hubDepth: cast field(specData, "hubDepth")
      };
      var key = field(keyData,
        "codeOnly") == true ? ParallelKey.custom(spec, cast field(keyData, "length")) : ParallelKey.metric(
          cast field(keyData, "size"),
          cast field(keyData, "length")
        );
      rebuilt.push({name: cast field(entry, "name"), z0: cast field(entry, "z0"), key: key});
    }
    var grooves:Array<ShaftGroove> = [for (row in rows(values.token("grooves"))) {
      name:cast field(row, "name"), z0:cast field(row, "z0"), width:cast field(row, "width"), diameter:cast field(
        row,
        "diameter"
      )
    }
    ];
    var raw = Json.parse(values.token("shaftDetail"));
    var shouldersRaw:Array<Dynamic> = cast field(raw, "shoulders");
    var shoulders:Array<ShaftShoulderDetail> = shouldersRaw == null ? null : [for (row in shouldersRaw) {
      z:cast field(row,
        "z"), fillet:cast field(row, "fillet"), reliefWidth:cast field(row, "reliefWidth"), reliefDiameter:cast field(
          row,
          "reliefDiameter"
        )
    }
    ];
    var inputRaw = field(raw, "inputThread"), outputRaw = field(raw, "outputThread");
    var inputThread:ShaftThreadEnd = inputRaw == null ? null : {
      diameter : cast field(inputRaw, "diameter"),
      pitch : cast field(inputRaw, "pitch"),
      length : cast field(inputRaw, "length")
    };
    var outputThread:ShaftThreadEnd = outputRaw == null ? null : {
      diameter : cast field(outputRaw, "diameter"),
      pitch : cast field(outputRaw, "pitch"),
      length : cast field(outputRaw, "length")
    };
    var detail:ShaftDetail = {
      inputChamfer: cast field(raw, "inputChamfer"),
      outputChamfer: cast field(raw, "outputChamfer"),
      inputThread: inputThread,
      outputThread: outputThread,
      shoulders: shoulders
    };
    return new SteppedShaft(sections, faces, rebuilt, grooves, detail);
  }

  static final recipeTypeCache = new machinekit.cache.Once<ComponentType>();

  public static function recipeType():ComponentType {
    return recipeTypeCache.get(() -> new ComponentType(
      "machinekit.motion.stepped-shaft",
      [
        ComponentRecipeSupport.text("sections", '[{"diameter":10,"length":100}]'),
        ComponentRecipeSupport.text("faces", "[]"),
        ComponentRecipeSupport.text("keyways", "[]"),
        ComponentRecipeSupport.text("grooves", "[]"),
        ComponentRecipeSupport.text("shaftDetail", "{}")
      ],
      v -> SteppedShaft.fromRecipe(v),
      true
    ).numbered(v -> fromRecipe(v).designation).procured(_ -> Make("turning and CNC machining")));
  }

  /** Subclasses must declare their own recipe and saved values. */
  override public function componentType():Null<ComponentType> return Std.isExactType(this,
    SteppedShaft) ? recipeType() : null;

  override public function values():machinekit.component.ComponentValues return recipeValues().setToken(
    "material",
    materialSpec()
  );

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var ownedParts:Array<Part> = [];
    return Solids.building(ownedParts, tracked -> {var z = 0.0, sectionsParts: Array<Part> = [];
    for (section in sections) {
      var part = Part.cylinderSpan(section.diameter / 2, z, z + section.length);
      tracked.push(part);
      sectionsParts.push(part);
      z += section.length;
    }
    var body = Solids.union(sectionsParts);
    tracked.push(body);
    if (detail == Envelope) return body;
    body = applyEndDetails(body);
    tracked.push(body);
    var tools:Array<Part> = [];
    for (keyway in keyways) {
      var tool = keywayTool(keyway);
      tracked.push(tool);
      tools.push(tool);
    }
    for (groove in grooves) {
      var tool = grooveTool(groove);
      tracked.push(tool);
      tools.push(tool);
    }
    for (shoulder in (this.detail.shoulders
      == null ? [] : this.detail.shoulders)) if (optionalFloat(shoulder.reliefWidth) > 0) {
      var tool = shoulderReliefTool(shoulder);
      tracked.push(tool);
      tools.push(tool);
    }
    if (tools.length == 0) return body;
    var result = Solids.cut(body, tools);
    tracked.push(result);
    return result;
    });
  }

  function applyEndDetails(body:Part):Part {
    var ownedParts:Array<Part> = [body];
    return Solids.building(ownedParts, tracked -> {var result = body;
    var inputThread = detail.inputThread, outputThread = detail.outputThread;
    if (inputThread != null) {
      result = applyThread(result, inputThread, true);
      tracked.push(result);
    }
    if (outputThread != null) {
      result = applyThread(result, outputThread, false);
      tracked.push(result);
    }
    if (optionalFloat(detail.inputChamfer) > 0) {
      result = finishEdge(result, 0, optionalFloat(detail.inputChamfer), false);
      tracked.push(result);
    }
    if (optionalFloat(detail.outputChamfer) > 0) {
      result = finishEdge(result, totalLength, optionalFloat(detail.outputChamfer), false);
      tracked.push(result);
    }
    for (shoulder in (detail.shoulders == null ? [] : detail.shoulders)) if (optionalFloat(shoulder.fillet) > 0) {
      result = finishEdge(result, shoulder.z, optionalFloat(shoulder.fillet), true);
      tracked.push(result);
    }
    return result;
    });
  }

  function applyThread(body:Part, thread:ShaftThreadEnd, atInput:Bool):Part {
    var z0 = atInput ? 0 : totalLength - thread.length;
    var z1 = atInput ? thread.length : totalLength;
    var outer = diameterAt(atInput ? 0 : totalLength) / 2 + 1;
    var inner = thread.diameter / 2;
    return cutPart(body,
      Solids.cut(Part.cylinderSpan(outer, z0, z1), [Part.cylinderSpan(inner, z0 - 0.05, z1 + 0.05)]));
  }

  function finishEdge(body:Part, z:Float, distance:Float, fillet:Bool):Part {
    var selection = body.edges().curve(CadKit.CurveKind.Circle).filter(function(edge) {
      return Math.abs(Selection.center(edge).z - z) < 1e-6;
    }
    );
    if (selection.count() == 0) {
      selection.close();
      throw 'No circular shaft edge at z=$z for ${fillet ? "fillet" : "chamfer"}';
    }
    try {
      var result = fillet ? body.fillet(selection, distance) : body.chamfer(selection, distance);
      selection.close();
      body.close();
      return result;
    } catch (error:Dynamic) {
      selection.close();
      body.close();
      throw error;
    }
  }

  function cutPart(body:Part, tool:Part):Part {
    try {
      var result = body.subtract(tool);
      body.close();
      tool.close();
      return result;
    } catch (error:Dynamic) {
      body.close();
      tool.close();
      throw error;
    }
  }

  function shoulderReliefTool(shoulder:ShaftShoulderDetail):Part {
    var width = optionalFloat(shoulder.reliefWidth);
    var rawDiameter = shoulder.reliefDiameter;
    if (rawDiameter == null) throw 'Shoulder relief at z=${shoulder.z} needs a diameter';
    var diameter = rawDiameter;
    var adjacent = Math.max(diameterAt(shoulder.z - 1e-5), diameterAt(shoulder.z + 1e-5));
    return Solids.cut(
      Part.cylinderSpan(adjacent / 2 + 1, shoulder.z - width / 2, shoulder.z + width / 2),
      [Part.cylinderSpan(diameter / 2, shoulder.z - width / 2 - 0.05, shoulder.z + width / 2 + 0.05)]
    );
  }

  /** Annulus from the groove diameter out past the shaft surface. */
  function grooveTool(groove:ShaftGroove):Part {
    var inner = groove.diameter / 2, outer = diameterAt(groove.z0) / 2 + 1, z1 = groove.z0 + groove.width;
    return Part.revolve([{r: inner, z: groove.z0}, {r: outer, z: groove.z0}, {r: outer, z: z1}, {r: inner, z: z1}]);
  }

  function keywayTool(keyway:ShaftKeyway):Part {
    var radius = diameterAt(keyway.z0) / 2,
      depth = keyway.key.spec.shaftDepth, width = keyway.key.spec.width, floor = radius - depth;
    return Part.prism(
      [
        new Vector(-width / 2, floor),
        new Vector(width / 2, floor),
        new Vector(width / 2, radius + depth),
        new Vector(-width / 2, radius + depth),
      ],
      keyway.z0,
      keyway.z0 + keyway.key.length
    );
  }
}
