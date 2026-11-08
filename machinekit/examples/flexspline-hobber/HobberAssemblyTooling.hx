import sys.io.File;

typedef HobberToolRow = {
  id:String, category:String, description:String, quantity:Int,
  unitPriceEur:Float, basis:String
};
typedef HobberToolStep = {id:String, parts:Array<String>, tools:Array<String>, measurement:String};

/** Assembly tooling is a separate BOM: it adds neither machine members nor
 * machine mass. Prices are planning assumptions, never supplier quotations. */
class HobberAssemblyTooling {
  static function row(id:String,description:String,price:Float,quantity:Int=1):HobberToolRow return {
    id:id,category:"Tooling",description:description,quantity:quantity,unitPriceEur:price,
    basis:"ASSUMED EUR planning allowance; obtain a quote and qualify the tool before use"
  };
  public static function rows():Array<HobberToolRow> return [
    row("precision-level","Precision machine level; datum P in two directions",80),
    row("straightedge-granite","Straightedge or granite reference for guide straightness",180),
    row("dial-indicator","Dial indicator; resolution/uncertainty suitable for ALIGNMENT.md acceptance",35,2),
    row("magnetic-base","Adjustable magnetic indicator base",25,2),
    row("block-transfer-sleeves","Matched rail-block transfer sleeves",20),
    row("x-drive-arbor","X drive alignment arbor; journals and length to the selected screw drawing",120),
    row("z-drive-arbor","Z drive alignment arbor; journals and length to the selected screw drawing",120),
    row("drive-alignment-arbor","Y module drive alignment arbor to the module screw profile",120),
    row("nut-bore-height-reference","Carriage nut-bore height reference for support-seat replication",100),
    row("replication-kit","Replication epoxy, release agent and dams; selected product/range per RAIL_MOUNTING.md",120),
    row("block-pad-master","Common master for the four replicated X block pads",150),
    row("c-test-mandrel","C datum test mandrel to the selected table interface; qualify runout",100),
    row("square-granite","Precision square or granite reference for squareness",180),
    row("spindle-test-arbor","Ground spindle test arbor for B/outboard indication",100),
    row("torque-driver","Calibrated torque driver with the required metric hex bits",80),
    row("feeler-gauges","Feeler gauges for scale gaps and guard clearances",15),
    row("af4-long-key","AF4 hex key, at least400 mm reach for X nut-flange service (P5)",25),
    row("af3-low-profile-key","AF3 key: qualified6 mm stub, D3.5 envelope,90 mm reach",12),
    row("af2.5-key","AF2.5 key, at least25 mm clear driver reach for M3 bellows fixings",8),
    row("af5.5-long-spanner","AF5.5 spanner: at least140 mm reach; qualified motor-bay envelope",18),
    row("air-fitting-socket","Socket for the sealed-scale purge-air fitting",8),
    row("bellows-support-straps","Temporary straps supporting released Z bellows ends; qualify retention",5,4),
    row("shop-hoist","Shop hoist; procure rated capacity>=250 kg and qualify the lifting arrangement",300),
    row("qualified-slings","Slings for the saddle/head lifting fixture; rated capacity>=250 kg required",25,2),
    row("pin-spanner","Pin spanner to the outboard gland drawing",20),
    row("af22-thin-socket","AF22 socket, outside diameter<=28 mm for outboard retaining-nut service",20),
    row("bearing-puller","Bearing puller to the qualified outboard shaft fit",80)
  ];
  public static function zSteps(machine:FlexsplineHobber):Array<HobberToolStep> return [
    {id:"Z1 guides",parts:["railZLeft","railZRight"],tools:["straightedge-granite","dial-indicator","magnetic-base","torque-driver"],measurement:"Reference edges; indicate straightness/parallelism to ALIGNMENT.md before the upper bellows brackets obstruct the rail drivers."},
    {id:"Z2 drive-seat fixture",parts:["Zfixed","Zfloating","ZnutBlock"],tools:["z-drive-arbor","nut-bore-height-reference","replication-kit","dial-indicator","magnetic-base"],measurement:"Required fixture-replicated seat process: arbor coaxial and parallel to rails, height from the carriage nut bore; transfer support pins after cure. XA replacement of the existing ground bar remains an OPEN release gate."},
    {id:"Z3 saddle on bench",parts:["Zplate"].concat(machine.zBellowsMounts.filter(id->id.indexOf("Moving")>=0)),tools:["torque-driver","af3-low-profile-key"],measurement:"Fit moving bellows brackets before lowering the saddle onto its blocks. Their rear screws are deliberately serviced on the bench."},
    {id:"Z4 column relation",parts:["Zplate","column/casting"],tools:["c-test-mandrel","square-granite","dial-indicator","magnetic-base","replication-kit"],measurement:"A6: indicate Z parallel to the C mandrel in two planes. Replicated sole/match-pin qualification remains OPEN; no pin or bond credit in slip/stiffness."},
    {id:"Z5 bellows",parts:[for(c in machine.zBellows)for(id in [c.fixedMember,c.movingMember])id],tools:["af2.5-key","torque-driver","feeler-gauges","bellows-support-straps"],measurement:"Park Z high for lower end screws and low for upper screws. Check>=2 mm external clearance through overtravel. For service support released ends, remove saddle, then withdraw complete covers along their checked paths."}
  ];
  public static function steps(machine:FlexsplineHobber):Array<HobberToolStep> {
    var out:Array<HobberToolStep>=[];
    for(s in HobberXAssembly.steps())out.push({id:"X/"+s.id,parts:s.parts,tools:s.tools,measurement:s.measurement});
    for(i=>s in machine.yModule.assemblySteps())out.push({id:"Y/"+(i+1),parts:[for(id in s.parts)"Ymodule/"+id],tools:s.tools,measurement:s.measurement});
    out=out.concat(zSteps(machine));
    out.push({id:"C/B references",parts:["table/platen","mandrel","spindleSeat","outboard"],tools:["c-test-mandrel","spindle-test-arbor","square-granite","dial-indicator","magnetic-base","pin-spanner","af22-thin-socket","bearing-puller"],measurement:"Use the datum and runout measurements in ALIGNMENT.md and b-align-outboard; qualify mandrel/arbor uncertainty before alignment."});
    out.push({id:"Head service lift",parts:["Zplate","swivel/housing"],tools:["shop-hoist","qualified-slings","bellows-support-straps"],measurement:"Qualify the saddle/head lifting arrangement and balance before releasing its joints; support the bellows ends independently."});
    return out;
  }
  public static function check(machine:FlexsplineHobber):Int {
    var catalog=new Map<String,HobberToolRow>(),count=0,members=[for(m in machine.components())m.id];
    for(r in rows()) {
      if(catalog.exists(r.id)||r.category!="Tooling"||r.quantity<=0||r.unitPriceEur<=0||r.basis.indexOf("ASSUMED")<0)throw "Invalid tooling BOM row: "+r.id;
      catalog.set(r.id,r);
    }
    for(s in steps(machine)) {
      if(s.tools.length==0||StringTools.trim(s.measurement)=="")throw "Assembly step lacks tools/measurement: "+s.id;
      for(id in s.parts)if(!members.contains(id))throw "Unknown assembly step part: "+s.id+" / "+id;
      for(id in s.tools){if(!catalog.exists(id))throw "Assembly step names undeclared tooling: "+s.id+" / "+id;count++;}
    }
    Sys.println('Assembly tooling: ${rows().length} separate BOM rows; $count checked step references; all prices ASSUMED EUR.');
    return count;
  }
  static function csv(value:String):String return '"'+value.split('"').join('""')+'"';
  public static function writeCsv(path:String,machine:FlexsplineHobber):Void {
    check(machine);
    var lines=["category,tool_id,description,quantity,unit_price_eur,basis,used_by_step"],declarations=steps(machine);
    for(r in rows())lines.push([r.category,r.id,r.description,Std.string(r.quantity),Std.string(r.unitPriceEur),r.basis,[for(s in declarations)if(s.tools.contains(r.id))s.id].join("; ")].map(csv).join(","));
    File.saveContent(path,lines.join("\n")+"\n");
  }
}
