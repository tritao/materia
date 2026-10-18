package machinekit.component;

/** Declared manufacturing intent, not a certificate of process or load capability. */
class ManufacturingVariant {
  public final id:String;
  public final process:String;
  public final material:String;
  public final status:String;
  final requiredHardware:Array<{partNumber:String,quantity:Int,description:String,material:String}>;
  final pending:Array<String>;
  public function new(id:String,process:String,material:String,status:String,
      hardware:Array<{partNumber:String,quantity:Int,description:String,material:String}>,pendingChecks:Array<String>) {
    if(id==null || id.length==0 || process==null || process.length==0 || material==null || material.length==0 ||
        (status!="prototype" && status!="validated")) throw "Invalid manufacturing variant";
    if(hardware==null || pendingChecks==null || (status=="validated" && pendingChecks.length>0))
      throw "Inconsistent manufacturing validation status";
    for(item in hardware) {
      if(item.partNumber.length==0 || item.quantity<=0) throw "Invalid variant hardware";
      materia.project.MaterialLibrary.fromSpec(item.material);
    }
    this.id=id;this.process=process;this.material=material;this.status=status;
    requiredHardware=[for(item in hardware) {partNumber:item.partNumber,quantity:item.quantity,description:item.description,material:item.material}];
    pending=pendingChecks.copy();
  }
  public function hardware():Array<{partNumber:String,quantity:Int,description:String,material:String}>
    return [for(item in requiredHardware) {partNumber:item.partNumber,quantity:item.quantity,description:item.description,material:item.material}];
  public function pendingChecks():Array<String> return pending.copy();
}
