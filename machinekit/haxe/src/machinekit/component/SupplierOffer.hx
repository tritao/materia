package machinekit.component;

/** One sourcing option for an engineering BOM part. Prices are snapshot minor units.
 * A nominal geometric match must retain compatibilityVerified=false until reviewed.
 */
typedef SupplierOffer = {
    var id:String;
    var partNumber:String;
    var supplierId:String;
    var supplierSku:String;
    var productUrl:String;
    var observedAt:String;
    var currency:String;
    var unitPriceMinor:Null<Int>;
    var currencyMinorUnit:Int;
    var taxBasis:String;
    var inStock:Bool;
    var compatibilityVerified:Bool;
}
