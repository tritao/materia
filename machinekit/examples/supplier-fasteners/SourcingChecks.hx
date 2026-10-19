import machinekit.component.Bom;
import machinekit.component.SourcingPlan;
import machinekit.component.SupplierOffer;

/** Sourcing invariants independent of CAD and live catalogue contents. */
class SourcingChecks {
    static function check(value:Bool, message:String):Void { if (!value) throw message; }
    static function rejected(action:Void->Void):Bool {
        try action() catch (_:Dynamic) return true;
        return false;
    }

    public static function run():Void {
        var bom = new Bom();
        bom.add({partNumber:"ISO4762-M6x20-A2-70", description:"Screw", quantity:1, material:"stainless A2-70"}, 4);
        bom.add({partNumber:"unmapped", description:"Unmapped part", quantity:1, material:null});
        var before = haxe.Json.stringify(bom.lines());
        var plan = new SourcingPlan();
        check(plan.lines(bom)[0].status == "unselected", "Sourcing must be optional");
        var candidate:SupplierOffer = {id:"vallder:example", partNumber:"ISO4762-M6x20-A2-70",
            supplierId:"vallder", supplierSku:"example", productUrl:"https://vallder.com/product/example/",
            observedAt:"2026-10-19T00:00:00Z", currency:"EUR", currencyMinorUnit:2,
            unitPriceMinor:26, taxBasis:"unspecified", inStock:false, compatibilityVerified:false};
        plan.add(candidate);
        candidate.unitPriceMinor = 999;
        check(rejected(() -> plan.add(candidate)), "Reject duplicate offer identity");
        check(rejected(() -> plan.select("unmapped", candidate.id)), "Reject offers for a different engineering part");
        plan.select(candidate.partNumber, candidate.id);
        var lines = plan.lines(bom);
        check(lines[0].status == "review_required", "Nominal candidate must stay unverified");
        check(lines[0].estimatedLinePriceMinor == 104, "Minor-unit price uses BOM quantity and stored snapshot");
        check(lines[1].status == "unselected", "Unmapped part must remain visible");
        lines[0].selected.unitPriceMinor = 800;
        check(plan.lines(bom)[0].estimatedLinePriceMinor == 104, "Export must not mutate stored offers");
        candidate.id = "another-supplier:example";
        candidate.supplierId = "another-supplier";
        candidate.unitPriceMinor = 30;
        candidate.compatibilityVerified = true;
        plan.add(candidate);
        plan.select(candidate.partNumber, candidate.id);
        check(plan.lines(bom)[0].status == "unavailable", "Verified out-of-stock offer remains unavailable");
        check(plan.lines(bom)[0].offers.length == 2, "Multiple suppliers share one engineering requirement");
        check(before == haxe.Json.stringify(bom.lines()), "Sourcing must preserve engineering BOM identity");
        plan.clearSelection(candidate.partNumber);
        check(plan.lines(bom)[0].selected == null, "Supplier selection can be removed");
        candidate.id = "available";
        candidate.inStock = true;
        plan.add(candidate);
        plan.select(candidate.partNumber, candidate.id);
        check(plan.lines(bom)[0].status == "selected", "Verified available offer resolves selection");
        check(rejected(() -> plan.lines(new Bom())), "Reject stale selection after part removal");
    }
}
