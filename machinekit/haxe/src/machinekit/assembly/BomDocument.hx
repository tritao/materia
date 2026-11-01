package machinekit.assembly;

import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.BomItem.BomProperties;
import machinekit.component.BomItem.BomCut;

enum abstract BomUseKind(String) to String {
  var Member = "member";
  var Inventory = "inventory";
}

enum abstract BomUnit(String) to String {
  var Piece = "piece";
  var Metre = "m";
  var SquareMetre = "m2";
  var Kilogram = "kg";
  var Litre = "l";
}

typedef BomQuantity = { var value:Float; var unit:BomUnit; }
typedef BomUse = { var kind:BomUseKind; var path:String; var quantity:BomQuantity; }
typedef BomProcurement = {
  var kind:String;
  var process:Null<String>;
  var manufacturer:Null<String>;
  var mpn:Null<String>;
  var standard:Null<String>;
}
typedef BomRequirement = {
  var name:String;
  var min:Null<String>;
  var max:Null<String>;
  var nominal:Null<String>;
  var unit:String;
}

typedef BomDocumentItem = {
  var partNumber:String;
  var description:String;
  var quantity:BomQuantity;
  var material:Null<String>;
  var typeId:Null<String>;
  var valuesKey:Null<String>;
  var familyId:Null<String>;
  var manufacturingVariant:Null<String>;
  var manufacturingProcess:Null<String>;
  var properties:Null<BomProperties>;
  var cuts:Array<BomCut>;
  var includedIn:Null<String>;
  var procurement:Null<BomProcurement>;
  var maturity:Null<String>;
  var requirements:Array<BomRequirement>;
  var fingerprint:Null<String>;
  var releasedRevision:Null<String>;
  var releaseCurrent:Null<Bool>;
}

typedef BomDocumentLine = { var item:BomDocumentItem; var uses:Array<BomUse>; }

class BomAssemblyNode {
  public final path:String;
  public final lines:Array<BomDocumentLine> = [];
  public final children:Array<BomAssemblyNode> = [];
  public function new(path:String) this.path = path;
}

/** The assembly tree retains all modelled occurrences; purchase lines are derived from it. */
class BomDocument {
  public final schema:String = "machinekit.bom";
  public final version:Int = 1;
  public final machineId:String;
  public final sourceCommit:String;
  public final generatedAt:String;
  public final assembly:BomAssemblyNode;
  public final procurement:Array<BomDocumentLine>;

  public function new(machineId:String, sourceCommit:String, generatedAt:String, assembly:BomAssemblyNode,
    ?releases:machinekit.component.BomReleases) {
    if (machineId == null || machineId == "" || sourceCommit == null || sourceCommit == ""
      || generatedAt == null || generatedAt == "") throw "BOM provenance is required";
    this.machineId = machineId;
    this.sourceCommit = sourceCommit;
    this.generatedAt = generatedAt;
    this.assembly = assembly;
    if (releases != null) applyReleases(assembly, releases);
    this.procurement = procurementView(assembly);
  }

  public static function occurrence(item:BomItem, kind:BomUseKind, path:String, count:Int,
    owner:Null<String>):BomDocumentLine {
    if (item == null || count <= 0 || item.quantity <= 0) throw "BOM occurrence needs positive quantity";
    var properties = Bom.copyPanel(item.properties);
    if (properties != null && item.measure != null) throw "Panel area and explicit measure conflict";
    var unit:BomUnit = properties != null ? SquareMetre : item.measure == null ? Piece : switch item.measure.unit {
      case "piece": Piece;
      case "m": Metre;
      case "m2": SquareMetre;
      case "kg": Kilogram;
      case "l": Litre;
      case _: throw "Unknown BOM quantity unit";
    };
    var perItem = properties != null ? properties.areaM2 : item.measure == null ? 1.0 : item.measure.value;
    if (!Math.isFinite(perItem) || perItem <= 0) throw "BOM measure must be finite and positive";
    var amount = item.quantity * count * perItem;
    var cuts:Array<BomCut> = [], cutLength = 0.0;
    if (item.cuts != null) for (cut in item.cuts) {
      if (unit != Metre || !Math.isFinite(cut.length) || cut.length <= 0 || cut.count <= 0)
        throw "BOM cuts need positive lengths and counts in metres";
      cutLength += cut.length * cut.count;
      cuts.push({length:cut.length, count:cut.count * item.quantity * count});
    }
    if (cutLength > perItem + 0.000000001) throw "BOM cuts exceed the ordered length";
    var quantity:BomQuantity = {value: amount, unit: unit};
    return {item: {partNumber:item.partNumber, description:item.description, quantity:quantity,
      material:item.material, typeId:item.typeId, valuesKey:item.valuesKey, familyId:item.familyId,
      manufacturingVariant:item.manufacturingVariant, manufacturingProcess:item.manufacturingProcess,
      properties:properties, cuts:cuts, includedIn:owner, procurement:null, maturity:null, requirements:[],
      fingerprint:item.fingerprint, releasedRevision:null, releaseCurrent:null},
      uses:[{kind:kind, path:path, quantity:{value:amount, unit:unit}}]};
  }

  static function applyReleases(node:BomAssemblyNode, releases:machinekit.component.BomReleases):Void {
    for (line in node.lines) {
      var release = releases.find(line.item.partNumber);
      if (release != null) {
        line.item.releasedRevision = release.revision;
        line.item.releaseCurrent = line.item.fingerprint != null && line.item.fingerprint == release.fingerprint;
      }
    }
    for (child in node.children) applyReleases(child, releases);
  }

  /** Roll up whole purchases from the retained engineering occurrences. */
  public static function procurementView(root:BomAssemblyNode):Array<BomDocumentLine> {
    var order:Array<BomDocumentLine> = [], byNumber:Map<String, BomDocumentLine> = [], owners:Array<String> = [];
    collect(root, order, byNumber, owners);
    for (owner in owners) if (!byNumber.exists(owner)) throw 'Included purchase owner missing from BOM: $owner';
    return order;
  }

  static function collect(node:BomAssemblyNode, order:Array<BomDocumentLine>, byNumber:Map<String, BomDocumentLine>,
    owners:Array<String>):Void {
    for (line in node.lines) {
      var item = line.item;
      if (item.includedIn != null) { owners.push(item.includedIn); continue; }
      if (item.procurement != null && item.procurement.kind == "NotSupplied") continue;
      var existing = byNumber.get(item.partNumber);
      if (existing == null) {
        var copy:BomDocumentItem = {partNumber:item.partNumber, description:item.description,
          quantity:{value:item.quantity.value, unit:item.quantity.unit}, material:item.material, typeId:item.typeId,
          valuesKey:item.valuesKey, familyId:item.familyId, manufacturingVariant:item.manufacturingVariant,
          manufacturingProcess:item.manufacturingProcess, properties:Bom.copyPanel(item.properties),
          cuts:[for (cut in item.cuts) {length:cut.length, count:cut.count}],
          includedIn:null, procurement:item.procurement, maturity:item.maturity, requirements:item.requirements.copy(),
          fingerprint:item.fingerprint, releasedRevision:item.releasedRevision, releaseCurrent:item.releaseCurrent};
        var added:BomDocumentLine = {item:copy, uses:[for (use in line.uses) copyUse(use)]};
        byNumber.set(item.partNumber, added);
        order.push(added);
      } else {
        var old = existing.item;
        if (old.description != item.description || old.material != item.material || old.typeId != item.typeId
          || old.valuesKey != item.valuesKey || old.familyId != item.familyId
          || old.manufacturingVariant != item.manufacturingVariant || old.manufacturingProcess != item.manufacturingProcess
          || old.quantity.unit != item.quantity.unit || haxe.Json.stringify(old.properties) != haxe.Json.stringify(item.properties)
          || haxe.Json.stringify(old.procurement) != haxe.Json.stringify(item.procurement) || old.maturity != item.maturity
          || haxe.Json.stringify(old.requirements) != haxe.Json.stringify(item.requirements)
          || old.fingerprint != item.fingerprint || old.releasedRevision != item.releasedRevision
          || old.releaseCurrent != item.releaseCurrent)
          throw 'BOM part number "${item.partNumber}" has conflicting metadata or values';
        old.quantity.value += item.quantity.value;
        for (use in line.uses) existing.uses.push(copyUse(use));
        for (cut in item.cuts) old.cuts.push({length:cut.length, count:cut.count});
      }
    }
    for (child in node.children) collect(child, order, byNumber, owners);
  }

  static function copyUse(use:BomUse):BomUse return {
    kind:use.kind, path:use.path, quantity:{value:use.quantity.value, unit:use.quantity.unit}
  };
}
