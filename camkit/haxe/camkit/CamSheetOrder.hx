package camkit;

/** Operation precedence is explicit; final releases additionally depend on every operation on that part. */
typedef CamSheetOperation = {
  id:String,part:String,area:Float,kind:String,requires:Array<String>,releases:Bool
}
typedef CamSheetOrderResult = {operations:Array<CamSheetOperation>,deferred:Array<String>};

class CamSheetOrder {
  /**
   * `admit` tentatively simulates the candidate and must roll it back on refusal.
   * A successful candidate advances that simulation's state. This keeps StockKit
   * and vacuum assumptions in the caller rather than duplicating them in CamKit.
   */
  public static function order(operations:Array<CamSheetOperation>,?admit:CamSheetOperation->Bool):CamSheetOrderResult {
    var ids=new Map<String,Bool>();
    for(op in operations) {
      if(op.id==null||op.id.length==0||ids.exists(op.id)||op.part==null||op.part.length==0||
        !Math.isFinite(op.area)||op.area<=0||op.requires==null||["inside","outside","drill","surface","pocket"].indexOf(op.kind)<0)
        throw "Invalid sheet operation identity, area or kind";
      ids.set(op.id,true);
    }
    for(op in operations)for(id in op.requires)if(!ids.exists(id)||id==op.id)throw "Invalid sheet operation prerequisite";
    var result:Array<CamSheetOperation> = [],done=new Map<String,Bool>(),deferred:Array<String> = [];
    while(result.length<operations.length) {
      var ready:Array<CamSheetOperation> = [];
      for(op in operations)if(!done.exists(op.id)) {
        var blocked=false;
        for(id in op.requires)if(!done.exists(id))blocked=true;
        if(op.releases)for(other in operations)if(other.part==op.part&&!other.releases&&other.id!=op.id&&!done.exists(other.id))blocked=true;
        if(op.kind=="outside")for(other in operations)if(other.part==op.part&&other.kind!="outside"&&!done.exists(other.id))blocked=true;
        if(!blocked)ready.push(op);
      }
      ready.sort((a,b)->a.releases!=b.releases?(a.releases?1:-1):(a.kind=="outside")!=(b.kind=="outside")?(a.kind=="outside"?1:-1):a.area!=b.area?(a.area<b.area?-1:1):Reflect.compare(a.id,b.id));
      var selected:Null<CamSheetOperation> = null;
      for(op in ready) {
        if(admit==null||admit(op)){selected=op;break;}
        if(deferred.indexOf(op.id)<0)deferred.push(op.id);
      }
      if(selected==null)throw ready.length==0?"Cyclic sheet operation prerequisites":"No remaining sheet operation passes the freed-part holding check";
      result.push(selected);done.set(selected.id,true);
    }
    return {operations:result,deferred:deferred};
  }
}
