import machinekit.assembly.MachineAssembly;
import machinekit.component.BomJson;
import sourcing.SourcingPlan;
import sourcing.SupplierOffer;

/** Sourcing invariants exercised through the public version-one document. */
class SourcingChecks {
  static function check(value:Bool, message:String):Void {
    if (!value) throw message;
  }
  static function rejected(action:Void -> Void):Bool {
    try action() catch (_:Dynamic) return true;
    return false;
  }
  public static function main():Void run();
  public static function run():Void {
    var fineAssembly = new MachineAssembly();
    fineAssembly.addComponent("retainingNut", new machinekit.standard.FinePitchNut());
    var fine = BomJson.parse(haxe.Json.stringify(fineAssembly.bomDocument("fine-nut", "test", "2026-01-01T00:00:00Z"))).procurement[0].item;
    check(fine.maturity == "Assumed" && fine.requirements.length == 5, "Fine-pitch nut exports dimensions and property class");
    var assembly = new MachineAssembly();
    var first = new machinekit.standard.Bushing(8, 12, 10);
    var second = new machinekit.standard.Bushing(10, 14, 10);
    assembly.addBomItem(first.bom, 4);
    assembly.addBomItem(second.bom);
    var json = haxe.Json.stringify(assembly.bomDocument("sourcing-check", "test", "2026-01-01T00:00:00Z"));
    var view = BomJson.parse(json).procurement;
    var item = SourcingPlan.find(view, first.bom.partNumber);
    var other = SourcingPlan.find(view, second.bom.partNumber);
    var plan = new SourcingPlan();
    check(plan.lines(view)[0].status == "unselected", "Sourcing must be optional");
    var candidate:SupplierOffer = {
      id: "vallder:example", partNumber: item.partNumber, supplierId: "vallder", supplierSku: "example",
      productUrl: "https://vallder.com/product/example/", observedAt: "2026-10-19T00:00:00Z", currency: "EUR",
      currencyMinorUnit: 2, unitPriceMinor: 26, taxBasis: "unspecified", inStock: false, compatibilityVerified: false
    };
    plan.add(candidate, item);
    candidate.unitPriceMinor = 999;
    check(rejected(() -> plan.add(candidate, item)), "Reject duplicate offer identity");
    check(rejected(() -> plan.select(other, candidate.id)), "Reject offers for a different engineering part");
    plan.select(item, candidate.id);
    var lines = plan.lines(view);
    check(lines[0].status == "review_required", "Nominal candidate must stay unverified");
    check(lines[0].estimatedLinePriceMinor == 104, "Minor-unit price uses BOM quantity and stored snapshot");
    check(lines[1].status == "unselected", "Unmapped part must remain visible");
    lines[0].selected.unitPriceMinor = 800;
    check(plan.lines(view)[0].estimatedLinePriceMinor == 104, "Export must not mutate stored offers");
    candidate.id = "another-supplier:example";
    candidate.supplierId = "another-supplier";
    candidate.unitPriceMinor = 30;
    candidate.compatibilityVerified = true;
    plan.add(candidate, item);
    plan.select(item, candidate.id);
    check(plan.lines(view)[0].status == "unavailable", "Verified out-of-stock offer remains unavailable");
    check(plan.lines(view)[0].offers.length == 2, "Multiple suppliers share one engineering requirement");
    check(json == haxe.Json.stringify(assembly.bomDocument("sourcing-check", "test", "2026-01-01T00:00:00Z")), "Sourcing preserves engineering BOM");
    var changed = BomJson.parse(json).procurement;
    changed[0].item.fingerprint = "different-fingerprint";
    check(rejected(() -> plan.lines(changed)), "Changed fingerprint must orphan the old selection");
    var revised = BomJson.parse(json).procurement;
    revised[0].item.releasedRevision = "B";
    check(rejected(() -> plan.lines(revised)), "Changed release must orphan the old selection");
    plan.clearSelection(item);
    check(plan.lines(changed)[0].offers.length == 0, "Offers must stay bound to their original fingerprint");
    check(plan.lines(revised)[0].offers.length == 0, "Offers must stay bound to their original release");
    check(plan.lines(view)[0].selected == null, "Supplier selection can be removed");
    candidate.id = "available";
    candidate.inStock = true;
    plan.add(candidate, item);
    plan.select(item, candidate.id);
    check(plan.lines(view)[0].status == "selected", "Verified available offer resolves selection");
    check(rejected(() -> plan.lines([])), "Reject stale selection after part removal");
  }
}
