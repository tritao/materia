package machinekit.cover;

import cadkit.modeling.Part;
import machinekit.component.BoltedFace;
import machinekit.component.BoltedFace.PinnedFace;
import machinekit.component.BoltedFace.ThreadMaterial;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentValues;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;

/** A round hole through a cover mount, in the mount's frame: `x` across, `y` up from the surface the cover stands on, `d` its diameter, mm. */
typedef MountBore = {x:Float, y:Float, d:Float};

/** A rectangular cut-out through a cover mount (`y0` of 0 or less cuts to the surface), mm. */
typedef MountCut = {x0:Float, x1:Float, y0:Float, y1:Float};

/** Monolithic added stock in the mounting plate's frame, mm. */
typedef MountStock = {x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float};

/** The machined foot of a mount: its stock and the mating faces that locate and clamp it to the machine. */
typedef MountAttachment = {
  stock:Array<MountStock>,
  bolted:Array<BoltedFace>,
  pinned:Array<PinnedFace>
};

private typedef MountClampRecipe = {
  id:String, frame:materia.assembly.AssemblyRecord.AssemblyFrame,
  holes:Null<Array<machinekit.component.BoltedFace.BoltHole>>, width:Float, height:Float,
  size:Null<String>, clearance:Float, grip:Float, recess:Null<machinekit.component.BoltedFace.BoltRecess>,
  drawn:Bool, washer:Bool, alternative:Null<String>, rated:Bool
};
private typedef MountPinRecipe = {
  id:String, frame:materia.assembly.AssemblyRecord.AssemblyFrame,
  holes:Array<machinekit.component.BoltedFace.PinHole>, diameter:Float, depth:Float,
  through:Bool, drawn:Bool, fit:String
};
private typedef MountAttachmentRecipe = {
  version:Int, stock:Array<MountStock>, bolted:Array<MountClampRecipe>, pinned:Array<MountPinRecipe>
};

/**
 * The plate a way cover's flange is bolted to where the machine has no face of its own: an end wall behind the folded cover or a carrier
 * on the moving member. Frame: the mounting face at z = 0 with the plate behind it (z from `-thickness` to 0), `x` across, `y` up from the
 * surface the cover stands on, the same frame a cover's flange has (`WayCoverSection`), so a mount and the flange it holds are placed with
 * one pose. The face is a tapped, drilled-to-suit receiving face over the whole plate: the flange's holes decide where it is tapped.
 * Bores let a screw journal or a shaft through; cuts clear the rails and blocks a wall stands over.
 */
class WayCoverMount extends MachineComponent {
  public final halfWidth:Float;
  public final height:Float;
  /** How far the plate reaches below the surface the cover stands on (a wall on a bridge lower than the surface), mm. */
  public final below:Float;
  public final thickness:Float;
  public final bores:Array<MountBore>;
  public final cuts:Array<MountCut>;
  public final attachment:Null<MountAttachment>;

  public function new(drawing:String, halfWidth:Float, height:Float, thickness:Float, ?bores:Array<MountBore>, ?cuts:Array<MountCut>, below:Float = 0, ?attachment:MountAttachment) {
    super('WAYCOVER-MOUNT-${Dimension.format(2 * halfWidth)}x${Dimension.format(height)}x${Dimension.format(thickness)}-${cutKey(bores, cuts)}' + (below > 0 ? '-D${Dimension.format(below)}' : '')
      + (attachment == null ? '' : '-F' + attachmentKey(attachment).substr(0, 8)),
      "Way cover mounting plate", "steel", true);
    if (!(halfWidth > 0) || !(height > 0) || !(thickness >= 8)) throw "A cover mount needs a width, a height and at least 8 mm of steel to tap";
    declareDrawingNumber(drawing);
    addFacet(new machinekit.component.ProcurementIntent(machinekit.component.Procurement.Make(attachment == null ? "laser cutting and tapping" : "CNC machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
    this.halfWidth = halfWidth;
    this.height = height;
    this.below = below;
    this.thickness = thickness;
    this.bores = bores == null ? [] : bores.copy();
    this.cuts = cuts == null ? [] : cuts.copy();
    if (attachment != null) {
      if (attachment.stock.length == 0 || attachment.bolted.length == 0) throw "A mounting foot needs stock and clamping faces";
      for (b in attachment.stock) if (!(b.x1 > b.x0) || !(b.y1 > b.y0) || !(b.z1 > b.z0)
        || !Math.isFinite(b.x0 + b.x1 + b.y0 + b.y1 + b.z0 + b.z1)) throw "A mounting foot needs finite, positive stock";
      for (face in attachment.bolted) if (!face.isClamped()) throw "A mounting foot declares its clamped faces; its host owns the receiving faces";
    }
    this.attachment = attachment == null ? null : {
      stock: attachment.stock.copy(), bolted: attachment.bolted.copy(), pinned: attachment.pinned.copy()
    };
  }

  static var recipe:Null<machinekit.component.ComponentType>;
  public static function recipeType():machinekit.component.ComponentType {
    if (recipe == null) recipe = new machinekit.component.ComponentType("machinekit.cover.way-cover-mount", [
      machinekit.component.ComponentRecipeSupport.text("drawing", "WAYCOVER-MOUNT"),
      machinekit.component.ComponentRecipeSupport.length("halfWidth", 50),
      machinekit.component.ComponentRecipeSupport.length("height", 50),
      machinekit.component.ComponentRecipeSupport.length("thickness", 14),
      machinekit.component.ComponentRecipeSupport.length("below", 0),
      machinekit.component.ComponentRecipeSupport.text("bores", "[]"),
      machinekit.component.ComponentRecipeSupport.text("cuts", "[]"),
      machinekit.component.ComponentRecipeSupport.text("attachment", "null")
    ], v -> new WayCoverMount(v.token("drawing"), v.number("halfWidth"), v.number("height"), v.number("thickness"),
      readBores(v.token("bores")), readCuts(v.token("cuts")), v.number("below"),
      readAttachment(v.token("attachment"))), true).numbered(v -> v.token("drawing"));
    return recipe;
  }
  override public function componentType():Null<machinekit.component.ComponentType> return recipeType();
  override public function values():ComponentValues return new ComponentValues()
    .setToken("drawing", drawingNumber).setNumber("halfWidth", halfWidth).setNumber("height", height)
    .setNumber("thickness", thickness).setNumber("below", below).setToken("bores", haxe.Json.stringify(bores))
    .setToken("cuts", haxe.Json.stringify(cuts)).setToken("attachment", writeAttachment(attachment))
    .setToken("material", materialSpec());

  static function writeAttachment(a:Null<MountAttachment>):String {
    if (a == null) return "null";
    var bolts:Array<MountClampRecipe> = [for (f in a.bolted) {
      if (f.thicknessAt != null) throw "A cover mount recipe cannot serialize a stock callback";
      switch f.role {
        case Clamped(size, clearance, grip, recess): {
          id:f.id, frame:f.frame, holes:f.holes, width:f.width, height:f.height, size:size,
          clearance:clearance, grip:grip, recess:recess, drawn:f.drawn, washer:f.washer,
          alternative:f.alternative, rated:f.rated
        };
        case _: throw "A mounting foot declares its clamped faces";
      }
    }];
    var pins:Array<MountPinRecipe> = [for (f in a.pinned) {
      id:f.id, frame:f.frame, holes:f.holes, diameter:f.diameter, depth:f.depth,
      through:f.through, drawn:f.drawn, fit:f.fit
    }];
    var data:MountAttachmentRecipe = {version:1, stock:a.stock, bolted:bolts, pinned:pins};
    return haxe.Json.stringify(data);
  }
  static function readBores(text:String):Array<MountBore> {
    var rows:Array<Dynamic> = cast haxe.Json.parse(text);
    return [for (r in rows) {x:r.x, y:r.y, d:r.d}];
  }
  static function readCuts(text:String):Array<MountCut> {
    var rows:Array<Dynamic> = cast haxe.Json.parse(text);
    return [for (r in rows) {x0:r.x0, x1:r.x1, y0:r.y0, y1:r.y1}];
  }
  static function readFrame(r:Dynamic):materia.assembly.AssemblyRecord.AssemblyFrame {
    var frame:materia.assembly.AssemblyRecord.AssemblyFrame = {x:r.x, y:r.y, z:r.z, qx:r.qx, qy:r.qy, qz:r.qz, qw:r.qw};
    materia.assembly.AssemblyCodec.validateFrame(frame);
    return frame;
  }
  static function readHoles(rows:Null<Array<Dynamic>>):Null<Array<machinekit.component.BoltedFace.BoltHole>>
    return rows == null ? null : [for (r in rows) {x:r.x, y:r.y}];
  static function readAttachment(text:String):Null<MountAttachment> {
    var data:Dynamic = haxe.Json.parse(text);
    if (data == null) return null;
    if (data.version != 1) throw "Unsupported cover mount attachment recipe";
    var stock:Array<MountStock> = [for (b in (cast data.stock:Array<Dynamic>))
      {x0:b.x0, x1:b.x1, y0:b.y0, y1:b.y1, z0:b.z0, z1:b.z1}];
    var bolts:Array<BoltedFace> = [for (f in (cast data.bolted:Array<Dynamic>)) {
      var recess:Null<machinekit.component.BoltedFace.BoltRecess> = f.recess == null ? null
        : {diameter:f.recess.diameter, depth:f.recess.depth};
      new BoltedFace(f.id, readFrame(f.frame), Clamped(f.size, f.clearance, f.grip, recess), readHoles(f.holes),
        f.width, f.height, null, f.drawn, f.washer, f.alternative, f.rated);
    }];
    var pins:Array<PinnedFace> = [for (f in (cast data.pinned:Array<Dynamic>)) {
      var holes = readHoles(f.holes);
      if (holes == null) throw "A locating face needs its hole pattern";
      new PinnedFace(f.id, readFrame(f.frame), holes, f.diameter, f.depth, f.through, f.drawn, f.fit);
    }];
    return {stock:stock, bolted:bolts, pinned:pins};
  }

  static function dimensionsKey(values:Array<Float>):String
    return values.map(v -> machinekit.component.ComponentNumberKey.encode(v)).join(",");

  static function frameKey(f:materia.assembly.AssemblyRecord.AssemblyFrame):String
    return dimensionsKey([f.x, f.y, f.z, f.qx, f.qy, f.qz, f.qw]);

  static function attachmentKey(attachment:MountAttachment):String {
    var stock = [for (b in attachment.stock) dimensionsKey([b.x0, b.x1, b.y0, b.y1, b.z0, b.z1])];
    var bolts = [for (face in attachment.bolted) {
      var role = switch face.role {
        case Clamped(size, clearance, grip, recess): haxe.Json.stringify({size: size,
          dimensions: dimensionsKey([clearance, grip]),
          recess: recess == null ? null : dimensionsKey([recess.diameter, recess.depth])});
        case _: throw "A mounting foot declares its clamped faces";
      };
      {id: face.id, frame: frameKey(face.frame), role: role, footprint: dimensionsKey([face.width, face.height]),
        holes: face.holes == null ? null : [for (h in face.holes) dimensionsKey([h.x, h.y])],
        drawn: face.drawn, washer: face.washer, alternative: face.alternative};
    }];
    var pins = [for (pin in attachment.pinned) {id: pin.id, frame: frameKey(pin.frame),
      holes: [for (h in pin.holes) dimensionsKey([h.x, h.y])], dimensions: dimensionsKey([pin.diameter, pin.depth]),
      through: pin.through, drawn: pin.drawn, fit: pin.fit}];
    return machinekit.component.BomRevision.fingerprint(haxe.Json.stringify({stock: stock, bolts: bolts, pins: pins}), 1);
  }

  static function cutKey(bores:Null<Array<MountBore>>, cuts:Null<Array<MountCut>>):String {
    var parts:Array<String> = [];
    if (bores != null) for (b in bores) parts.push('B${Dimension.format(b.x)},${Dimension.format(b.y)},${Dimension.format(b.d)}');
    if (cuts != null) for (c in cuts) parts.push('C${Dimension.format(c.x0)},${Dimension.format(c.x1)},${Dimension.format(c.y0)},${Dimension.format(c.y1)}');
    return parts.length == 0 ? "plain" : machinekit.component.BomRevision.fingerprint(parts.join("|"), 1).substr(0, 8);
  }

  override public function revisionFingerprint():Null<String> {
    var inputs = [drawingNumber, materialSpec(), Dimension.format(halfWidth), Dimension.format(height), Dimension.format(below), Dimension.format(thickness)];
    for (b in bores) inputs.push('B${Dimension.format(b.x)},${Dimension.format(b.y)},${Dimension.format(b.d)}');
    for (c in cuts) inputs.push('C${Dimension.format(c.x0)},${Dimension.format(c.x1)},${Dimension.format(c.y0)},${Dimension.format(c.y1)}');
    if (attachment != null) inputs.push(attachmentKey(attachment));
    return machinekit.component.BomRevision.fingerprint(inputs.join("|"), 1);
  }

  /** The face the cover's flange is bolted to: the whole front of the plate, tapped to suit the flange's holes. */
  override public function fasteningFaces():Array<BoltedFace> return [BoltedFace.drilledToSuit(
    "cover",
    BoltedFace.frameOf({x: 0, y: (height - below) / 2, z: 0}, {x: 0, y: 0, z: -1}, {x: 1, y: 0, z: 0}),
    2 * halfWidth,
    height + below,
    thickness - 2,
    thickness,
    ThreadMaterial.Steel
  )].concat(attachment == null ? [] : attachment.bolted);

  override public function pinnedFaces():Array<PinnedFace> return attachment == null ? [] : attachment.pinned.copy();

  override public function machinable():Bool return true;

  override public function hasGeometry():Bool return true;

  override public function geometry(detail:ComponentDetail = Preview):Part {
    var body = Part.box(2 * halfWidth, height + below, thickness).translated(new cadkit.modeling.Vector(0, (height - below) / 2, -thickness));
    if (attachment != null) body = Solids.union([body].concat([for (b in attachment.stock)
      Part.box(b.x1 - b.x0, b.y1 - b.y0, b.z1 - b.z0).translated(new cadkit.modeling.Vector(
        (b.x0 + b.x1) / 2, (b.y0 + b.y1) / 2, b.z0
      ))]));
    var tools:Array<Part> = [];
    for (b in bores) tools.push(Part.cylinderSpan(b.d / 2, -thickness - 1, 1, b.x, b.y));
    for (c in cuts) {
      var tool = Part.box(c.x1 - c.x0, c.y1 - c.y0, thickness + 2).translated(new cadkit.modeling.Vector((c.x0 + c.x1) / 2, (c.y0 + c.y1) / 2, -thickness - 1));
      tools.push(tool);
    }
    return machined(Solids.cut(Solids.named(body, "plate"), tools));
  }
}
