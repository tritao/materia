import machinekit.component.Bom;
import machinekit.component.BomItem;
import machinekit.component.SourcingPlan;
import machinekit.component.SupplierOffer;

/** Tray proposal BOM through the normal BOM/sourcing overlay; not added to the installed machine twice. */
class HobberTrayBom {
  static function csv(value:Dynamic):String {
    var s=value==null?"":Std.string(value);
    return '"'+s.split('"').join('""')+'"';
  }
  static function main():Void {
    var root=Sys.getCwd();
    while(!sys.FileSystem.exists(root+"/machinekit/examples/flexspline-hobber/tray-bom.json")) {
      var parent=haxe.io.Path.directory(root);if(parent==root||parent.length==0)throw "Cannot find tray BOM";root=parent;
    }
    var folder=root+"/machinekit/examples/flexspline-hobber";
    var data:Dynamic=haxe.Json.parse(sys.io.File.getContent(folder+"/tray-bom.json"));
    var sources:Dynamic=haxe.Json.parse(sys.io.File.getContent(folder+"/tray-anchor-sourcing.json"));
    var proposal:Dynamic=haxe.Json.parse(sys.io.File.getContent(folder+"/tray-mounting-proposal.json"));
    if(data.revision!=proposal.revision)throw "Stale tray BOM revision";
    var items:Array<Dynamic> = data.items,bom=new Bom(),ids:Map<String,Bool> = [];
    for(raw in items) {
      var item:BomItem={partNumber:raw.partNumber,description:raw.description,quantity:raw.quantity,material:raw.material};
      if(ids.exists(item.partNumber)||item.quantity<=0)throw "Tray BOM needs unique positive requirements";
      ids.set(item.partNumber,true);bom.add(item);
      if(Reflect.field(data.lineNotes,item.partNumber)==null)throw "BOM quantity needs a unit and review notes";
    }
    var count:Int=proposal.sockets.count;
    for(number in [sources.requirement.partNumber,"HOBBER-TRAY-SCREW-ISO4762-M6X25","HOBBER-TRAY-WASHER-14X6.6X1","HOBBER-TRAY-COMPRESSION-SLEEVE-PROPOSAL"])
      if(bom.quantity(number)!=count)throw 'Hardware quantity mismatch: $number';
    if(bom.quantity("HOBBER-TRAY-SHEET-RECT-V2")!=1||bom.quantity("HOBBER-TRAY-GASKET-TAPE-CANDIDATE")!=proposal.gasket.suggestedPurchaseLengthM)
      throw "Sheet/gasket quantity mismatch";
    var plan=new SourcingPlan(),offers:Array<Dynamic> = sources.offers;
    for(raw in offers){
      var offer:SupplierOffer={id:raw.id,partNumber:raw.partNumber,supplierId:raw.supplierId,supplierSku:raw.supplierSku,
        productUrl:raw.productUrl,observedAt:raw.observedAt,currency:raw.currency,unitPriceMinor:raw.unitPriceMinor,
        currencyMinorUnit:raw.currencyMinorUnit,taxBasis:raw.taxBasis,inStock:raw.inStock,compatibilityVerified:raw.compatibilityVerified};
      if(!ids.exists(offer.partNumber))throw 'Unmapped supplier reference ${offer.id}';plan.add(offer);
    }
    var selections:Array<Dynamic> = sources.selections;
    for(selection in selections)plan.select(selection.partNumber,selection.offerId);
    var rows=plan.lines(bom),sleeve=[for(r in rows)if(r.requirement.partNumber=="HOBBER-TRAY-COMPRESSION-SLEEVE-PROPOSAL")r][0];
    if(sleeve.offers.length!=1||sleeve.offers[0].supplierSku!="007.66.089"||sleeve.offers[0].compatibilityVerified)
      throw "Spacer candidate must retain its SKU and unverified compatibility";
    var out=folder+"/build/tray-mounting";sys.FileSystem.createDirectory(out);
    sys.io.File.saveContent(out+"/tray-bom-sourced.json",haxe.Json.stringify({scope:data.scope,revision:data.revision,status:data.status,
      lines:rows,lineNotes:data.lineNotes,offerNotes:sources.offerNotes,supplierLeads:sources.supplierLeads,
      openItems:data.openItems,availabilityPolicy:sources.availabilityPolicy},null,"  "));
    var table=["part_number,description,quantity,unit,material,sourcing_status,supplier_references,source_urls,review_status"];
    for(row in rows) {
      var item=row.requirement,notes:Dynamic=Reflect.field(data.lineNotes,item.partNumber);
      table.push([for(v in [item.partNumber,item.description,Std.string(item.quantity),Std.string(notes.unit),item.material,row.status,
        [for(o in row.offers)o.supplierId+":"+o.supplierSku].join(" | "),[for(o in row.offers)o.productUrl].join(" | "),Std.string(notes.status)])csv(v)].join(","));
    }
    sys.io.File.saveContent(out+"/tray-bom.csv",table.join("\n")+"\n");
    Sys.println('Tray BOM: ${rows.length} requirements, ${offers.length} supplier candidates; hardware quantities, units and unverified spacer mapping checked.');
  }
}
