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
    row("c-test-mandrel","C datum test mandrel to the selected table interface; qualify runout",100),
    row("square-granite","Precision square or granite reference for squareness",180),
    row("spindle-test-arbor","Ground spindle test arbor for B/outboard indication",100),
    row("torque-driver","Calibrated torque driver with the required metric hex bits",80),
    row("feeler-gauges","Feeler gauges for scale gaps and guard clearances",15),
    row("af4-long-key","AF4 hex key, at least400 mm reach for X nut-flange service (P5)",25),
    row("af3-low-profile-key","AF3 key: qualified6 mm stub, D3.5 envelope,90 mm reach",12),
    row("af2.5-key","AF2.5 key, at least100 mm reach for lowered X guard feet and M3 bellows fixings; D3.5 envelope ASSUMED",8),
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
    {id:"Z2 ground bars and supports",parts:["ZdriveFixedBar","ZdriveFloatingBar","Zfixed","Zfloating","ZnutBlock"],tools:["dial-indicator","magnetic-base","torque-driver"],measurement:"Fit matched-thickness short ground bars, tightening their recessed screws first. Indicate supports to the actual screw and rails at both travel ends, then match-pin to the bars. No epoxy/arbor fixture. Covered bar screws are reached after support removal for rare bar replacement."},
    {id:"Z3 saddle on bench",parts:["Zplate"].concat(machine.zBellowsMounts.filter(id->id.indexOf("Moving")>=0)),tools:["torque-driver","af3-low-profile-key"],measurement:"Fit moving bellows brackets before lowering the saddle onto its blocks. Their rear screws are deliberately serviced on the bench."},
    {id:"Z4 column relation",parts:["Zplate","column/casting"],tools:["c-test-mandrel","square-granite","dial-indicator","magnetic-base"],measurement:"A6: indicate Z parallel to the C mandrel in two planes. Ground sole matching/indication and match-pin qualification remains OPEN; no pin or bond credit in slip/stiffness."},
    {id:"Z5 bellows",parts:[for(c in machine.zBellows)for(id in [c.fixedMember,c.movingMember])id],tools:["af2.5-key","torque-driver","feeler-gauges","bellows-support-straps"],measurement:"Park Z high for lower end screws and low for upper screws. Check>=2 mm external clearance through overtravel. For service support released ends, remove saddle, then withdraw complete covers along their checked paths."}
  ];
  public static function ySteps(machine:FlexsplineHobber):Array<HobberToolStep> {
    var out:Array<HobberToolStep>=[];
    for(i=>s in machine.yModule.assemblySteps()) {
      var measurement=s.measurement;
      if(s.parts.contains("coverUpper"))measurement+=" For this host, park X at its retract limit and Z at its upper limit, centre Y and A, and clear the workholding mandrel before handling the lower guard. Withdraw the upper half upward; move the lower half down only by its declared profile-derived distance, then forward. Reverse these checked paths for installation. Both screw supports remain bolted and pinned. Release the clip-in PU lips clear of projecting head-interface fasteners before sliding the steel guards; qualify the assumed lip peel/clip process with the selected seal profile. The rigid-path and retained-hardware checks do not qualify elastomer strain. Remove both guards before unbolting a stop or reaching the wipers.";
      out.push({id:"Y/"+(i+1),parts:[for(id in s.parts)"Ymodule/"+id],tools:s.tools,measurement:measurement});
    }
    return out;
  }
  public static function steps(machine:FlexsplineHobber):Array<HobberToolStep> {
    var out:Array<HobberToolStep>=[];
    for(s in HobberXAssembly.steps())out.push({id:"X/"+s.id,parts:s.parts,tools:s.tools,measurement:s.measurement});
    out=out.concat(ySteps(machine));
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
