package machinekit.component;

typedef SourcedBomLine = {
    var requirement:BomItem;
    var offers:Array<SupplierOffer>;
    var selected:Null<SupplierOffer>;
    /** unselected, review_required, unavailable, price_missing, or selected */
    var status:String;
    /** Snapshot estimate even for a candidate; excludes shipping and unspecified taxes. */
    var estimatedLinePriceMinor:Null<Int>;
}

/** Optional sourcing overlay. Engineering BOM identity and aggregation never depend on supplier. */
class SourcingPlan {
    final offers:Map<String, SupplierOffer> = [];
    final order:Array<String> = [];
    final selections:Map<String, String> = [];

    public function new() {}

    public function add(offer:SupplierOffer):Void {
        if (offer == null || !text(offer.id) || !text(offer.partNumber) || !text(offer.supplierId) ||
            !text(offer.supplierSku) || !text(offer.productUrl) || !text(offer.observedAt) ||
            !~/^[A-Z]{3}$/.match(offer.currency) || offer.currencyMinorUnit < 0 || offer.currencyMinorUnit > 6 ||
            (offer.unitPriceMinor != null && offer.unitPriceMinor < 0) || !text(offer.taxBasis))
            throw "Incomplete supplier offer";
        if (offers.exists(offer.id)) throw 'Duplicate supplier offer "${offer.id}"';
        offers.set(offer.id, copy(offer));
        order.push(offer.id);
    }

    /** Explicit preference, including a provisional candidate; export still reports review_required. */
    public function select(partNumber:String, offerId:String):Void {
        var offer = offers.get(offerId);
        if (offer == null || offer.partNumber != partNumber) throw "Supplier offer does not match BOM part";
        selections.set(partNumber, offerId);
    }

    public function clearSelection(partNumber:String):Void selections.remove(partNumber);

    public function lines(bom:Bom):Array<SourcedBomLine> {
        var requirements = bom.lines(), seen:Map<String, Bool> = [], result:Array<SourcedBomLine> = [];
        for (item in requirements) {
            seen.set(item.partNumber, true);
            var available:Array<SupplierOffer> = [];
            for (id in order) {
                var offer = requireOffer(id);
                if (offer.partNumber == item.partNumber) available.push(copy(offer));
            }
            var selected:Null<SupplierOffer> = selections.exists(item.partNumber) ? copy(requireOffer(selections.get(item.partNumber))) : null;
            var status = selected == null ? "unselected" : !selected.compatibilityVerified ? "review_required" :
                !selected.inStock ? "unavailable" : selected.unitPriceMinor == null ? "price_missing" : "selected";
            var cost:Null<Int> = null;
            if (selected != null && selected.unitPriceMinor != null) {
                var estimate:Float = 1.0 * item.quantity * selected.unitPriceMinor;
                if (estimate > 2147483647 || estimate < 0) throw "Sourcing estimate exceeds integer range";
                cost = Std.int(estimate);
            }
            result.push({requirement:item, offers:available, selected:selected, status:status, estimatedLinePriceMinor:cost});
        }
        for (part in selections.keys()) if (!seen.exists(part)) throw 'Sourcing selection has no BOM requirement "$part"';
        return result;
    }

    static function text(value:String):Bool return value != null && StringTools.trim(value).length > 0;
    function requireOffer(id:String):SupplierOffer {
        var offer = offers.get(id);
        if (offer == null) throw 'Unknown supplier offer "$id"';
        return offer;
    }
    static function copy(offer:SupplierOffer):SupplierOffer return {
        id:offer.id, partNumber:offer.partNumber, supplierId:offer.supplierId, supplierSku:offer.supplierSku,
        productUrl:offer.productUrl, observedAt:offer.observedAt, currency:offer.currency,
        unitPriceMinor:offer.unitPriceMinor, currencyMinorUnit:offer.currencyMinorUnit, taxBasis:offer.taxBasis,
        inStock:offer.inStock, compatibilityVerified:offer.compatibilityVerified
    };
}
