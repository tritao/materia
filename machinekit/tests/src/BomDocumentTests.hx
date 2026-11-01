import machinekit.assembly.MachineAssembly;
import machinekit.component.MachineComponent;
import machinekit.component.IncludedInPurchase;

private class BomTestPart extends MachineComponent {
  public function new() {
    super("TEST-PART", "Test part", "steel");
    addFacet(new machinekit.component.ProcurementIntent(Make("machining")));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
  }
}

private class BomIncludedPart extends MachineComponent {
  public function new(owner:String) {
    super("INCLUDED-PIECE", "Included piece", "steel");
    addFacet(new IncludedInPurchase(owner));
    addFacet(new machinekit.component.ProcurementIntent(Buy(null, null, null)));
    addFacet(new machinekit.component.EngineeringMetadata(new machinekit.component.ComponentValues()
      .setToken("ratingsBasis", "Stated").setToken("errorsBasis", "Assumed").setToken("source", "")));
  }
}

class BomDocumentTests {
  public static function main():Void run();
  public static function run():Void {
    var child = new MachineAssembly();
    var source = new machinekit.standard.Bushing(8, 12, 10);
    var fingerprint = source.bom.fingerprint;
    if (fingerprint == null) throw "Recipe fingerprint is missing";
    var releases = machinekit.component.BomReleases.parse(haxe.Json.stringify({version:1, releases:[
      {partNumber:source.bom.partNumber, revision:"A", fingerprint:fingerprint, date:"2026-01-02"}
    ]}));
    var releasedAssembly = new MachineAssembly();
    releasedAssembly.addComponent("bushing", source);
    var nested = new MachineAssembly();
    nested.addBomItem(source.bom, 2);
    releasedAssembly.include("nested", nested);
    var released = releasedAssembly.bomDocument("released", "abc", "2026-01-03T00:00:00Z", releases);
    if (released.procurement[0].item.fingerprint != fingerprint || released.procurement[0].item.releasedRevision != "A"
      || released.procurement[0].item.releaseCurrent != true || released.assembly.lines[0].item.releaseCurrent != true
      || released.procurement[0].item.quantity.value != 3 || released.assembly.children[0].lines[0].item.fingerprint != fingerprint)
      throw "Current release must be visible in engineering and procurement";
    var stale = machinekit.component.BomReleases.parse(haxe.Json.stringify({version:1, releases:[
      {partNumber:source.bom.partNumber, revision:"A", fingerprint:StringTools.lpad("", "0", 64), date:"2026-01-02"}
    ]}));
    if (releasedAssembly.bomDocument("released", "abc", "2026-01-03T00:00:00Z", stale).procurement[0].item.releaseCurrent != false)
      throw "A release pinned to another fingerprint must be stale";
    var history = machinekit.component.BomReleases.parse(haxe.Json.stringify({version:1, releases:[
      {partNumber:source.bom.partNumber, revision:"A", fingerprint:StringTools.lpad("", "0", 64), date:"2026-01-02"},
      {partNumber:source.bom.partNumber, revision:"B", fingerprint:fingerprint, date:"2026-01-03"}
    ]}));
    var latest = releasedAssembly.bomDocument("released", "abc", "2026-01-03T00:00:00Z", history).procurement[0].item;
    if (latest.releasedRevision != "B" || latest.releaseCurrent != true)
      throw "The latest issued release must determine the exported revision";
    var releaseRejected = false;
    try machinekit.component.BomReleases.parse('{"version":2,"releases":[]}') catch (_:Dynamic) releaseRejected = true;
    if (!releaseRejected) throw "Unknown release schema versions must be rejected";
    child.addComponent("first", new BomTestPart());
    child.addBomItem(new BomTestPart().bom, 2);
    child.addBomItem({partNumber: "WIRE", description: "Wire", quantity: 2, material: null,
      procurement:Buy(null, null, null), maturity:"Stated"}, 3);
    var originalPanel = new machinekit.enclosure.EnclosurePanel(100, 200).bom;
    originalPanel.procurement = Buy(null, null, null);
    originalPanel.maturity = "Stated";
    var sourcePanel = originalPanel.properties;
    if (sourcePanel == null) throw "Panel metadata missing";
    var heat:machinekit.enclosure.LaminatedPanel.PanelProperty = {value: 80, basis: Assumed, source: "test"};
    sourcePanel.construction.qualifications.heatLimitC = heat;
    child.addBomItem(originalPanel);
    var root = new MachineAssembly();
    root.addComponent("second", new BomTestPart());
    root.include("child", child);
    var originalThickness = sourcePanel.construction.layers[0].thicknessMm;
    sourcePanel.construction.layers[0].thicknessMm = 999;
    heat.value = 200;
    var doc = root.bomDocument("example", "abc", "2026-01-01T00:00:00Z");
    if (doc.version != 1 || doc.machineId != "example" || doc.assembly.children.length != 1
      || doc.assembly.children[0].path != "child" || doc.procurement.length != 3) throw "BOM document structure";
    var part = doc.procurement[0], wire = doc.procurement[1];
    if (part.item.partNumber != "TEST-PART" || part.item.quantity.value != 4 || part.item.quantity.unit != "piece" || part.uses.length != 3
      || part.uses[0].path != "second" || part.uses[1].path != "child/first"
      || part.uses[0].kind != "member" || part.uses[2].kind != "inventory"
      || part.uses[2].path != "child/@inventory/0" || part.uses[2].quantity.value != 2) throw "BOM member uses";
    if (wire.item.partNumber != "WIRE" || wire.item.quantity.value != 6 || wire.uses.length != 1
      || wire.uses[0].path != "child/@inventory/1" || wire.uses[0].quantity.value != 6) throw "BOM inventory uses";
    part.uses[0].quantity.value = 99;
    if (doc.assembly.lines[0].uses[0].quantity.value != 1)
      throw "Procurement uses must not alias the engineering tree";
    var panel = doc.procurement[2].item.properties;
    if (doc.procurement[2].item.quantity.value != 0.02 || doc.procurement[2].item.quantity.unit != "m2") throw "Panel area quantity";
    if (panel == null || panel.widthMm != 100 || panel.heightMm != 200
      || panel.construction.constructionId != "fr-acm-4") throw "Nested inventory must retain panel metadata";
    if (panel.construction.layers[0].thicknessMm != originalThickness) throw "Included panel must be a snapshot";
    var exportedHeat = panel.construction.qualifications.heatLimitC;
    if (exportedHeat == null || exportedHeat.value != 80) throw "Panel qualification must be a snapshot";
    exportedHeat.value = 300;
    panel.construction.layers[0].thicknessMm = 888;
    var newPanel = root.bomDocument("example", "abc", "2026-01-01T00:00:00Z").procurement[2].item.properties;
    if (newPanel == null || newPanel.construction.layers[0].thicknessMm != originalThickness)
      throw "Exported panel metadata must not mutate the assembly";
    var newHeat = newPanel.construction.qualifications.heatLimitC;
    if (newHeat == null || newHeat.value != 80) throw "Exported qualification must not mutate the assembly";
    var parsed:Dynamic = haxe.Json.parse(haxe.Json.stringify(doc));
    if (parsed.schema != "machinekit.bom" || parsed.version != 1) throw "BOM JSON";
    doc.assembly.children[0].lines[2].item.procurement = {
      kind: "NotSupplied", process: null, manufacturer: null, mpn: null, standard: null
    };
    var procurement = machinekit.assembly.BomDocument.procurementView(doc.assembly);
    if (doc.assembly.children[0].lines.length != 4 || procurement.length != 2)
      throw "Not-supplied occurrences remain in engineering and are excluded from procurement";

    var stock = new MachineAssembly();
    var cable:machinekit.component.BomItem = {partNumber:"CABLE", description:"Cable", quantity:1,
      material:null, procurement:Buy(null, null, null), maturity:"Stated",
      measure:{value:2.5, unit:"m"}, cuts:[{length:1.2, count:2}]};
    stock.addBomItem(cable, 2);
    var stockDoc = stock.bomDocument("stock", "abc", "2026-01-01T00:00:00Z");
    var stockLine = stockDoc.procurement[0].item;
    if (stockLine.quantity.unit != "m" || stockLine.quantity.value != 5 || stockLine.cuts.length != 1
      || stockLine.cuts[0].length != 1.2 || stockLine.cuts[0].count != 4)
      throw "Stock quantities and cuts must retain their units and multiplicity";
    cable.measure.value = 9;
    cable.cuts[0].count = 9;
    if (stockLine.quantity.value != 5 || stockLine.cuts[0].count != 4)
      throw "Exported stock quantities must be snapshots";
    var invalid = new MachineAssembly();
    invalid.addBomItem({partNumber:"BAD", description:"Bad cut", quantity:1, material:null,
      procurement:Buy(null, null, null), maturity:"Stated", measure:{value:1, unit:"m"}, cuts:[{length:2, count:1}]});
    var invalidRejected = false;
    try invalid.bomDocument("bad", "abc", "2026-01-01T00:00:00Z") catch (_:Dynamic) invalidRejected = true;
    if (!invalidRejected) throw "Cuts cannot exceed the ordered stock length";

    var ambiguous = new MachineAssembly();
    ambiguous.addComponentAt(machinekit.assembly.InstancePath.of("@inventory/0"), new BomTestPart());
    ambiguous.addBomItem(new BomTestPart().bom);
    var distinct = ambiguous.bomDocument("ambiguous", "abc", "2026-01-01T00:00:00Z").procurement[0].uses;
    if (distinct.length != 2 || distinct[0].path != distinct[1].path || distinct[0].kind != "member"
      || distinct[1].kind != "inventory") throw "Member and inventory uses must be distinguishable";

    var purchase = new MachineAssembly();
    purchase.addComponent("owner", new BomTestPart());
    purchase.addComponent("piece", new BomIncludedPart("TEST-PART"));
    var purchaseDoc = purchase.bomDocument("purchase", "abc", "2026-01-01T00:00:00Z");
    if (purchaseDoc.assembly.lines.length != 2 || purchaseDoc.assembly.lines[1].item.includedIn != "TEST-PART")
      throw "Engineering tree must retain included purchase pieces";
    if (purchaseDoc.procurement.length != 1 || purchaseDoc.procurement[0].uses.length != 1
      || purchaseDoc.procurement[0].uses[0].path != "owner") throw "Included piece is not a purchase line";
    var orphan = new MachineAssembly();
    orphan.addComponent("piece", new BomIncludedPart("MISSING"));
    var rejected = false;
    try orphan.bomDocument("orphan", "abc", "2026-01-01T00:00:00Z") catch (_:Dynamic) rejected = true;
    if (!rejected) throw "Missing purchase owner must fail BOM export";
    var split = new MachineAssembly(), withOwner = new MachineAssembly(), withoutOwner = new MachineAssembly();
    withOwner.addComponent("owner", new BomTestPart());
    withoutOwner.addComponent("piece", new BomIncludedPart("TEST-PART"));
    split.include("ownerAssembly", withOwner);
    split.include("orphanAssembly", withoutOwner);
    var crossAssemblyRejected = false;
    try split.bomDocument("split", "abc", "2026-01-01T00:00:00Z") catch (_:Dynamic) crossAssemblyRejected = true;
    if (!crossAssemblyRejected) throw "A purchase in another assembly cannot satisfy an orphan";
    var cylinder = new machinekit.pneumatic.PneumaticCylinder("ISO6432", 16, 20, 100);
    var cylinderAssembly = new MachineAssembly();
    cylinderAssembly.addComponent("cylinder", cylinder);
    cylinderAssembly.addComponent("rod", cylinder.movingRod());
    var copiedCylinder = new MachineAssembly();
    copiedCylinder.include("cylinderAssembly", cylinderAssembly);
    var cylinderDoc = copiedCylinder.bomDocument("cylinder", "abc", "2026-01-01T00:00:00Z");
    if (cylinderDoc.procurement.length != 1 || cylinderDoc.assembly.children[0].lines.length != 2
      || cylinderDoc.assembly.children[0].lines[1].item.includedIn != cylinder.bom.partNumber)
      throw "A cylinder rod must remain included after an assembly snapshot";
  }
}
