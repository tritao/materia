package sourcing;

import machinekit.bomschema.BomV1.Item;
import machinekit.bomschema.BomV1.Line;

typedef SourcedBomLine = {
  var requirement:Item;
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
  final offerKeys:Map<String, String> = [];
  final selections:Map<String, String> = [];

  public function new() {
  }

  public function add(offer:SupplierOffer, item:Item):Void {
    if (offer == null
      || !text(offer.id) || !text(offer.partNumber) || !text(offer.supplierId) || !text(offer.supplierSku)
      || !text(offer.productUrl) || !text(offer.observedAt) || !~/^[A-Z]{3}$/.match(offer.currency)
      || offer.currencyMinorUnit < 0 || offer.currencyMinorUnit > 6 || (offer.unitPriceMinor != null
      && offer.unitPriceMinor < 0) || !text(offer.taxBasis)) throw "Incomplete supplier offer";
    if (offer.partNumber != item.partNumber) throw "Supplier offer does not match BOM part";
    var identity = key(item);
    if (offers.exists(offer.id)) throw 'Duplicate supplier offer "${offer.id}"';
    offers.set(offer.id, copy(offer));
    offerKeys.set(offer.id, identity);
    order.push(offer.id);
  }

  /** Explicit preference, including a provisional candidate; export still reports review_required. */
  public function select(item:Item, offerId:String):Void {
    var offer = offers.get(offerId);
    if (offer == null || offer.partNumber != item.partNumber || offerKeys.get(offerId) != key(item)) throw "Supplier offer does not match BOM part";
    selections.set(key(item), offerId);
  }

  public function clearSelection(item:Item):Void selections.remove(key(item));

  public function lines(requirements:Array<Line>):Array<SourcedBomLine> {
    var seen:Map<String, Bool> = [], result:Array<SourcedBomLine> = [];
    for (line in requirements) {
      var item = line.item;
      var identity = key(item);
      if (seen.exists(identity)) throw "Duplicate sourcing requirement identity";
      seen.set(identity, true);
      var available:Array<SupplierOffer> = [];
      for (id in order) {
        var offer = requireOffer(id);
        if (offerKeys.get(id) == identity) available.push(copy(offer));
      }
      var selected:Null<SupplierOffer
        > = selections.exists(identity) ? copy(requireOffer(selections.get(identity))) : null;
      var status = selected == null ? "unselected" : !selected.compatibilityVerified ? "review_required"
        : !selected.inStock ? "unavailable" : selected.unitPriceMinor == null ? "price_missing" : "selected";
      var cost:Null<Int> = null;
      if (selected != null && selected.unitPriceMinor != null) {
        var estimate:Float = 1.0 * item.quantity.value * selected.unitPriceMinor;
        if (estimate > 2147483647 || estimate < 0) throw "Sourcing estimate exceeds integer range";
        cost = Std.int(estimate);
      }
      result.push({
        requirement: item,
        offers: available,
        selected: selected,
        status: status,
        estimatedLinePriceMinor: cost
      });
    }
    for (part in selections.keys()) if (!seen.exists(part)) throw 'Sourcing selection has no BOM requirement "$part"';
    return result;
  }

  public static function find(lines:Array<Line>, partNumber:String):Item {
    var found:Null<Item> = null;
    for (line in lines) if (line.item.partNumber == partNumber) {
      if (found != null) throw 'Ambiguous sourcing part "$partNumber"';
      found = line.item;
    }
    if (found == null) throw 'Missing sourcing part "$partNumber"';
    return found;
  }
  static function key(item:Item):String {
    if (item == null || !text(item.partNumber) || (!text(item.fingerprint) && !text(item.releasedRevision)))
      throw "Sourcing requirement needs a part number and fingerprint or released revision";
    return haxe.Json.stringify([item.partNumber, item.fingerprint, item.releasedRevision]);
  }
  static function text(value:String):Bool return value != null && StringTools.trim(value).length > 0;
  function requireOffer(id:String):SupplierOffer {
    var offer = offers.get(id);
    if (offer == null) throw 'Unknown supplier offer "$id"';
    return offer;
  }
  static function copy(offer:SupplierOffer):SupplierOffer return {
    id: offer.id,
    partNumber: offer.partNumber,
    supplierId: offer.supplierId,
    supplierSku: offer.supplierSku,
    productUrl: offer.productUrl,
    observedAt: offer.observedAt,
    currency: offer.currency,
    unitPriceMinor: offer.unitPriceMinor,
    currencyMinorUnit: offer.currencyMinorUnit,
    taxBasis: offer.taxBasis,
    inStock: offer.inStock,
    compatibilityVerified: offer.compatibilityVerified
  };
}
