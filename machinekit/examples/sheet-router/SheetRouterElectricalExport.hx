import machinekit.component.AssumedCatalogComponent;
class SheetRouterElectricalExport {
 static function main(){
  var beds:Array<Dynamic> = [];
  for(bed in [2600.0,3150.0]){
   var machine=new SheetRouter(new SheetRouterSpec(bed)),channels:Array<{id:String,kind:String}> = [];
   for(member in machine.components())if(Std.isOfType(member.component,AssumedCatalogComponent)){
    var part:AssumedCatalogComponent=cast member.component,io=part.io();
    for(name in io.inputs)channels.push({id:member.id+"."+name,kind:member.id=="setter"&&name=="trigger"?"fast-input":"input"});
    for(name in io.outputs)channels.push({id:member.id+"."+name,kind:"output"});
    for(name in io.analogOutputs)channels.push({id:member.id+"."+name,kind:"analog"});
   }
   var definition=machine.definition();
   if(definition.switches!=null)for(contact in definition.switches)channels.push({id:contact.id,kind:"fast-input"});
   channels.sort((a,b)->Reflect.compare(a.id,b.id));beds.push({bed:bed,channels:channels});
  }
  sys.io.File.saveContent(Sys.args()[0],haxe.Json.stringify({beds:beds},null,"  ")+"\n");
 }
}
