package machinekit.component;

import machinekit.component.BomItem.BomProperties;
import machinekit.enclosure.LaminatedPanel.PanelProperty;

/** Aggregates bill-of-materials lines by part number. */
class Bom {
  final byPartNumber:Map<String, BomItem> = [];
  final order:Array<String> = [];
  final includedOwners:Array<String> = [];

  public function new() {
  }

  public function add(item:BomItem, quantity:Int = 1):Void {
    if (item.partNumber == null || item.partNumber.length == 0) throw "BOM item needs a part number";
    if (quantity <= 0) throw 'BOM quantity for "${item.partNumber}" must be positive';
    var existing = byPartNumber.get(item.partNumber);
    if (existing == null) {
      byPartNumber.set(
        item.partNumber,
        {
          partNumber: item.partNumber,
          description: item.description,
          quantity: item.quantity * quantity,
          material: item.material,
          familyId: item.familyId,
          manufacturingVariant: item.manufacturingVariant,
          manufacturingProcess: item.manufacturingProcess,
          typeId: item.typeId,
          valuesKey: item.valuesKey,
          fingerprint: item.fingerprint,
          properties: copyPanel(item.properties),
          measure: copyMeasure(item.measure), cuts: copyCuts(item.cuts)
        }
      );
      order.push(item.partNumber);
    } else {
      if (existing.description != item.description
        || existing.material != item.material || existing.typeId != item.typeId || existing.valuesKey != item.valuesKey || existing.fingerprint != item.fingerprint
        || haxe.Json.stringify(existing.properties) != haxe.Json.stringify(item.properties)
        || haxe.Json.stringify(existing.measure) != haxe.Json.stringify(item.measure)
        || haxe.Json.stringify(existing.cuts) != haxe.Json.stringify(item.cuts) || existing.familyId
        != item.familyId || existing.manufacturingVariant != item.manufacturingVariant || existing.manufacturingProcess
        != item.manufacturingProcess) throw 'BOM part number "${item.partNumber}" has conflicting metadata or values';
      existing.quantity += item.quantity * quantity;
    }
  }

  public function addComponent(component:MachineComponent, quantity:Int = 1):Void {
    if (quantity <= 0) throw "BOM component quantity must be positive";
    var included = IncludedInPurchase.of(component);
    if (included == null) add(component.bom, quantity);
    else includedOwners.push(included.ownerPartNumber);
  }

  /** Lines in first-added order. */
  public function lines():Array<BomItem> {
    for (owner in includedOwners) if (!byPartNumber
      .exists(owner)) throw 'Included purchase owner missing from BOM: $owner';
    var result:Array<BomItem> = [];
    for (partNumber in order) {
      var item = byPartNumber.get(partNumber);
      if (item != null) result.push({
        partNumber: item.partNumber,
        description: item.description,
        quantity: item.quantity,
        material: item.material,
        familyId: item.familyId,
        manufacturingVariant: item.manufacturingVariant,
        manufacturingProcess: item.manufacturingProcess,
        typeId: item.typeId,
        valuesKey: item.valuesKey,
        fingerprint: item.fingerprint,
        properties: copyPanel(item.properties),
        measure: copyMeasure(item.measure), cuts: copyCuts(item.cuts)
      });
    }
    return result;
  }

  public function quantity(partNumber:String):Int {
    var item = byPartNumber.get(partNumber);
    return item == null ? 0 : item.quantity;
  }

  public static function copyMeasure(measure:Null<machinekit.component.BomItem.BomMeasure>):Null<machinekit.component.BomItem.BomMeasure>
    return measure == null ? null : {value:measure.value, unit:measure.unit};

  public static function copyCuts(cuts:Null<Array<machinekit.component.BomItem.BomCut>>):Null<Array<machinekit.component.BomItem.BomCut>>
    return cuts == null ? null : [for (cut in cuts) {length:cut.length, count:cut.count}];

  /** Copy nested panel data when inventory and BOM snapshots cross ownership boundaries. */
  public static function copyPanel(panel:Null<BomProperties>):Null<BomProperties> {
    if (panel == null) return null;
    var c = panel.construction, q = c.qualifications;
    return {
      widthMm: panel.widthMm,
      heightMm: panel.heightMm,
      areaM2: panel.areaM2,
      massKg: panel.massKg,
      construction: {
        constructionId: c.constructionId,
        layers: [for (layer in c.layers) {materialId: layer.materialId, thicknessMm: layer.thicknessMm}],
        thicknessMm: c.thicknessMm,
        thicknessBasis: c.thicknessBasis,
        arealMassKgM2: c.arealMassKgM2,
        massBasis: c.massBasis,
        fabrication: c.fabrication,
        jointMethod: c.jointMethod,
        coating: c.coating,
        qualifications: {
          coolantRecord: q.coolantRecord,
          heatLimitC: copyProperty(q.heatLimitC),
          fireClass: q.fireClass,
          fireRecord: q.fireRecord,
          impactRecord: q.impactRecord
        },
        stiffnessPa: copyProperty(c.stiffnessPa),
        costPerM2: copyProperty(c.costPerM2)
      }
    };
  }

  static function copyProperty(value:Null<PanelProperty>):Null<PanelProperty> return value == null ? null : {
    value: value.value, basis: value.basis, source: value.source
  };
}
