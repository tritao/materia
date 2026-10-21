import camkit.*;
import camkit.CamSheetNester.CamNestResult;
import materia.sheet.*;
import materia.sheet.SheetPolygon.SheetShape;
import sys.io.File;

class SheetNestingChecks {
  static var assertions=0;
  static var outputDirectory:Null<String> = null;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function rejects(action:Void->Void,label:String):Void {var failed=false;try action()catch(_:Dynamic)failed=true;check(failed,label);}
  static function stock(w:Float,h:Float):SheetStockSpec return {id:'stock-$w-$h',revision:1,materialId:"mdf",lengthUnit:"mm",width:w,height:h,thickness:18};
  static function req(id:String,w:Float,h:Float,quantity:Int=1,?shape:SheetShape):SheetPartRequirement return {
    id:id,revision:1,partId:id,partRevision:1,quantity:quantity,materialId:"mdf",lengthUnit:"mm",thickness:18,thicknessToleranceMinus:0.5,thicknessTolerancePlus:0.5,
    blankWidth:w,blankHeight:h,widthToleranceMinus:0,widthTolerancePlus:0,heightToleranceMinus:0,heightTolerancePlus:0,
    allowedRotations:[0,90],directionConstraint:"free",shape:shape};
  static function saveLayout(result:CamNestResult,stock:SheetStockSpec,name:String):Void {
    if(outputDirectory==null)return;
    var plan=result.plans[0],checked=SheetPlanValidator.validate(plan,stock,result.requirements);
    File.saveContent(outputDirectory+"/"+name+".svg",SheetPlanExporter.svg(plan,stock,result.requirements,checked));
  }
  static function legacyFingerprint():Void {
    var legacy=req("legacy-panel",700,750,3);legacy.partId="panel";legacy.partRevision=2;
    // Evaluated against the unchanged version-1 implementation at b38630144.
    var before=SheetPlanValidator.requirementFingerprint(legacy);
    check(before=="eafdd1f2dbaa647e64bbe1eb72dddb78b484a629265a51509137c84ebed5c7fc","legacy rectangular fingerprint remains byte-compatible");
    legacy.shape=SheetPolygon.rectangle(0,0,700,750);
    var shapeHash=SheetPlanValidator.requirementFingerprint(legacy);
    check(shapeHash!=before,"polygon capture participates in staleness fingerprint");
    legacy.allowedAngles=[0.0,90.0,180.0,270.0];
    check(SheetPlanValidator.requirementFingerprint(legacy)!=shapeHash,"free-form angle policy participates in staleness fingerprint");
  }
  static function basic():Void {
    var s=stock(100,100),r=CamSheetNester.nest([req("panel",45,100,2)],[{stock:s,quantity:2}],{toolDiameter:6,gap:4,edgeMargin:0,seed:31,minimumRemnantSide:10});
    check(r.plans.length==1&&r.guillotineSheets==1,"tight rectangles use at least the guillotine baseline density");
    check(Math.abs(r.utilisation-0.9)<1e-10,"true material utilisation");
    CamSheetNester.validateBatch(r,[s]);
    var encoded=SheetProjectCodec.empty();encoded.stockSpecs=[s];encoded.requirements=r.requirements;encoded.plans=r.plans;
    var restored=SheetProjectCodec.decode(SheetProjectCodec.encode(encoded));
    check(restored.plans[0].schemaVersion==2&&SheetFreeLayout.placements(restored.plans[0]).length==2,"free-form plan codec preserves placements");
    var angles:Array<Float> = cast restored.requirements[0].allowedAngles;
    check(restored.requirements[0].shape!=null&&angles.length==4,"codec preserves polygon and angle snapshots");
    var checked=SheetPlanValidator.validate(restored.plans[0],s,restored.requirements);check(checked.valid,"independent free-form validation after round trip");
    check(SheetPlanExporter.svg(restored.plans[0],s,restored.requirements,checked).indexOf("fill-rule")>=0,"free-form SVG exports polygon geometry");
    check(SheetPlanExporter.csv(restored.plans[0],s,restored.requirements,checked).indexOf("polygon_loops_json")>=0,"free-form CSV preserves geometry instead of claiming guillotine cuts");
    var first=SheetFreeLayout.placements(restored.plans[0])[0],second=SheetFreeLayout.placements(restored.plans[0])[1];
    second.x=first.x;second.y=first.y;second.angle=first.angle;second.loops=first.loops;
    check(!SheetPlanValidator.validate(restored.plans[0],s,restored.requirements).valid,"overlap is rejected independently of the nesting engine");
    var moved=SheetProjectCodec.decode(SheetProjectCodec.encode(encoded));
    var outside=SheetFreeLayout.placements(moved.plans[0])[0];outside.x-=1;for(loop in outside.loops)for(p in loop)p.x-=1;
    check(!SheetPlanValidator.validate(moved.plans[0],s,moved.requirements).valid,"actual edge margin violation is rejected");
    moved=SheetProjectCodec.decode(SheetProjectCodec.encode(encoded));
    var close=SheetFreeLayout.placements(moved.plans[0])[1];close.x-=1;for(loop in close.loops)for(p in loop)p.x-=1;
    check(!SheetPolygon.overlap(SheetFreeLayout.placements(moved.plans[0])[0].loops,close.loops)&&
      !SheetPlanValidator.validate(moved.plans[0],s,moved.requirements).valid,"nonoverlapping parts still fail insufficient tool/gap clearance");
    var unsupported=SheetProjectCodec.decode(SheetProjectCodec.encode(encoded));
    var unknown:SheetStockSpec={id:s.id,revision:s.revision,materialId:"missing-material",lengthUnit:s.lengthUnit,width:s.width,height:s.height,thickness:s.thickness};
    unsupported.requirements[0].materialId=unknown.materialId;
    unsupported.plans[0].stockSpecFingerprint=SheetPlanValidator.stockFingerprint(unknown);
    unsupported.plans[0].requirements=SheetPlanValidator.captureRequirements(unsupported.requirements);
    check(!SheetPlanValidator.validate(unsupported.plans[0],unknown,unsupported.requirements).valid,"independent validator rejects uncatalogued material even when fingerprints match");
    s.width=99;check(!SheetPlanValidator.validate(r.plans[0],s,r.requirements).valid,"stock change invalidates captured free-form plan");
    var bow:SheetShape=[[{x:0.0,y:0.0},{x:10.0,y:10.0},{x:0.0,y:10.0},{x:10.0,y:0.0}]];
    rejects(()->SheetPolygon.validate(bow),"crossing input outline refused");
  }
  static function shapes():Void {
    var l:SheetShape=[[{x:0.0,y:0.0},{x:80.0,y:0.0},{x:80.0,y:20.0},{x:20.0,y:20.0},{x:20.0,y:80.0},{x:0.0,y:80.0}]];
    var s=stock(105,105),r=CamSheetNester.nest([req("L",80,80,2,l)],[{stock:s,quantity:1}],{toolDiameter:4,gap:1,edgeMargin:0,seed:31,minimumRemnantSide:10});
    check(r.plans.length==1&&SheetFreeLayout.placements(r.plans[0]).length==2,"concave L parts interlock on one sheet");
    var a=SheetFreeLayout.placements(r.plans[0])[0],b=SheetFreeLayout.placements(r.plans[0])[1],x=SheetPolygon.bounds(a.loops),y=SheetPolygon.bounds(b.loops);
    check(x.maxX>y.minX&&y.maxX>x.minX&&x.maxY>y.minY&&y.maxY>x.minY,"true-shape nest overlaps bounding boxes");
    check(!SheetPolygon.overlap(a.loops,b.loops)&&SheetPolygon.distance(a.loops,b.loops)>=5-1e-7,"interlocking polygons have exact edge/containment check and 5 mm clearance");
    saveLayout(r,s,"interlocking-l");
    var ring=SheetPolygon.rectangle(0,0,200,200);ring.push(SheetPolygon.rectangle(40,40,120,120)[0]);
    s=stock(200,200);r=CamSheetNester.nest([req("ring",200,200,1,ring),req("plug",80,80)],[{stock:s,quantity:1}],{toolDiameter:6,gap:1,edgeMargin:0,seed:31,minimumRemnantSide:10});
    check(r.plans.length==1&&r.stats.decompositions>0,"material decomposition honors a real hole");
    var ringPlacement=SheetFreeLayout.placements(r.plans[0])[0],plug=SheetFreeLayout.placements(r.plans[0])[1];
    check(SheetPolygon.inside(ringPlacement.loops[1],plug.loops[0][0])&&!SheetPolygon.overlap(ringPlacement.loops,plug.loops),"small part nests inside the parent's hole");
    check(SheetFreeLayout.remnants(r.plans[0]).length>0,"conservative reusable rectangles are returned from remaining material");
    CamSheetNester.validateBatch(r,[s]);
    saveLayout(r,s,"hole-nest");
    var mapped=CamSheetProfiles.freePart(r.plans[0],ringPlacement.id,0.018);
    check(mapped.holes.length==1&&mapped.outer.vertices.length==4,"free-form placement maps into SR11 CAM outer and hole boundaries");
  }
  static function rotationAndSheets():Void {
    var locked=req("grain",90,50);locked.directionConstraint="width-aligned";
    var s=stock(60,100);
    rejects(()->CamSheetNester.nest([locked],[{stock:s,quantity:1}],{toolDiameter:6,gap:1,edgeMargin:0}),"grain lock does not rotate a part just to fit");
    locked.directionConstraint="free";
    var r=CamSheetNester.nest([locked],[{stock:s,quantity:1}],{toolDiameter:6,gap:1,edgeMargin:0});
    check(SheetFreeLayout.placements(r.plans[0])[0].angle==90,"permitted quarter turn fits the narrow sheet");
    locked.directionConstraint="width-aligned";s=stock(120,120);
    r=CamSheetNester.nest([locked],[{stock:s,quantity:1}],{toolDiameter:6,gap:1,edgeMargin:5});
    check(SheetFreeLayout.placements(r.plans[0])[0].angle==0,"grain-locked part keeps orientation");
    var free=req("angle",20,70);free.allowedAngles=[45.0];s=stock(100,100);
    r=CamSheetNester.nest([free],[{stock:s,quantity:1}],{toolDiameter:6,gap:1,edgeMargin:5});
    check(SheetFreeLayout.placements(r.plans[0])[0].angle==45&&SheetPlanValidator.validate(r.plans[0],s,r.requirements).valid,"explicit non-quarter angle has the same grid transform in packing and validation");
    var many=req("many",80,80,4);r=CamSheetNester.nest([many],[{stock:s,quantity:4}],{toolDiameter:6,gap:1,edgeMargin:5});
    check(r.plans.length==4,"batch spans several available sheets");
    rejects(()->CamSheetNester.nest([many],[{stock:s,quantity:3}],{toolDiameter:6,gap:1,edgeMargin:5}),"insufficient sheets do not return a silently incomplete plan");
    var inventoryRecord=SheetProjectCodec.empty();inventoryRecord.stockSpecs=[s];inventoryRecord.requirements=r.requirements;inventoryRecord.plans=r.plans;
    var inventory=new SheetInventory(inventoryRecord,null);inventory.registerSheet("physical",s.id);inventory.allocate("physical",r.plans[0].id);
    rejects(()->inventory.execute(r.plans[0].id,"physical","run",true),"layout does not consume inventory without a qualified free-form cut sequence");
    check(inventory.piece("physical").state=="allocated","refused layout-only execution preserves inventory");
  }
  static function cabinet():Array<Dynamic> {
    var results:Array<Dynamic> = [];
    for(length in [2600.0,3150.0]) {
      var count=length>2800?8:6,s=stock(1600,length),requirements=[req("panel",700,750,count),req("small",50,50,6)];
      var options:camkit.CamSheetNester.CamNestOptions={toolDiameter:6,gap:1,edgeMargin:10,seed:31,minimumRemnantSide:100};
      var a=CamSheetNester.nest(requirements,[{stock:s,quantity:3}],options),b=CamSheetNester.nest(requirements,[{stock:s,quantity:3}],options);
      check(haxe.Json.stringify(a.plans)==haxe.Json.stringify(b.plans),"same seed yields identical complete polygon plans");
      check(a.guillotineSheets!=null&&a.plans.length<=a.guillotineSheets,"cabinet rectangles are no worse than guillotine candidate");
      check(a.plans.length==1&&a.utilisation>0.75,"cabinet set fills one sheet with useful material yield");
      CamSheetNester.validateBatch(a,[s]);
      saveLayout(a,s,'cabinet-${Std.int(length)}');
      results.push({workY:length,partCount:count+6,sheets:a.plans.length,guillotineSheets:a.guillotineSheets,utilisation:a.utilisation,
        seconds:a.elapsedSeconds,engine:a.engine,stats:a.stats,plans:a.plans,requirements:a.requirements});
      Sys.println('SR12 bed=$length parts=${count+6}, sheets=${a.plans.length}, utilisation=${a.utilisation}, seconds=${a.elapsedSeconds}');
    }
    return results;
  }
  static function main():Void {
    var args=Sys.args();if(args.length>0)outputDirectory=haxe.io.Path.directory(args[0]);
    legacyFingerprint();basic();shapes();rotationAndSheets();var results=cabinet();
    if(args.length>0)File.saveContent(args[0],haxe.Json.stringify({assertions:assertions,cabinet:results},null,"  ")+"\n");
    Sys.println('Sheet nesting checks passed ($assertions assertions)');
  }
}
