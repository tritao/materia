package camkit;

import camkit.CamSheetNfp.CamNestVariant;
import camkit.CamSheetNfp.CamNestPlaced;
import camkit.CamSheetNfp.CamNfpStats;
import materia.sheet.*;
import materia.sheet.SheetPolygon.SheetPoint;
import materia.sheet.SheetPolygon.SheetShape;
import materia.sheet.SheetPolygon.SheetRemnant;

typedef CamNestStock = {stock:SheetStockSpec,quantity:Int};
typedef CamNestOptions = {
  toolDiameter:Float,gap:Float,edgeMargin:Float,
  @:optional var seed:Int;
  @:optional var rotationStepDegrees:Float;
  @:optional var minimumRemnantSide:Float;
  @:optional var batchId:String;
}
typedef CamNestResult = {
  plans:Array<SheetCutPlan>,requirements:Array<SheetPartRequirement>,utilisation:Float,elapsedSeconds:Float,
  engine:String,guillotineSheets:Null<Int>,stats:CamNfpStats,seed:Int
}
private typedef Slot = {stock:SheetStockSpec,index:Int,width:Float,height:Float,placed:Array<CamNestPlaced>};
private typedef Instance = {id:String,requirement:SheetPartRequirement,area:Float,variants:Array<CamNestVariant>,tie:Int};
private typedef FreeRect = {x:Float,y:Float,width:Float,height:Float};

/** Deterministic bottom-left no-fit nesting over material polygons, including holes and concavity. Dimensions are mm. */
class CamSheetNester {
  static function hash(text:String,seed:Int):Int {var h=seed;for(i in 0...text.length)h=(h*31+text.charCodeAt(i))|0;return h;}
  static function compatible(req:SheetPartRequirement,stock:SheetStockSpec):Bool {
    var thickness=SheetPlanValidator.millimetres(stock.thickness,stock.lengthUnit);
    return req.materialId==stock.materialId&&thickness>=req.thickness-req.thicknessToleranceMinus-1e-7&&thickness<=req.thickness+req.thicknessTolerancePlus+1e-7;
  }
  static function slots(stocks:Array<CamNestStock>):Array<Slot> {
    var result:Array<Slot> = [];
    for(item in stocks) {
      if(item.quantity<1||item.quantity>100||result.length+item.quantity>1000)throw "Nesting stock quantity must be 1..100, with at most 1000 source sheets";
      for(i in 0...item.quantity) {
        var w=SheetPlanValidator.millimetres(item.stock.width,item.stock.lengthUnit),h=SheetPlanValidator.millimetres(item.stock.height,item.stock.lengthUnit);
        if(w<=0||h<=0||w>10000||h>10000)throw "Invalid nesting sheet dimensions";
        result.push({stock:item.stock,index:result.length,width:Math.floor(w/SheetPolygon.QUANTUM)*SheetPolygon.QUANTUM,
          height:Math.floor(h/SheetPolygon.QUANTUM)*SheetPolygon.QUANTUM,placed:[]});
      }
    }
    return result;
  }
  static function capture(req:SheetPartRequirement,step:Float):SheetPartRequirement {
    var scale=SheetPlanValidator.millimetres(1,req.lengthUnit);
    var shape:SheetShape=req.shape==null?SheetPolygon.rectangle(0,0,req.blankWidth*scale,req.blankHeight*scale):
      [for(loop in req.shape)[for(p in loop)SheetPolygon.point(p.x*scale,p.y*scale)]];
    shape=SheetPolygon.normalized(shape);
    var angles:Array<Float> = req.allowedAngles==null?[for(i in 0...Std.int(Math.ceil(360/step)))i*step]:req.allowedAngles.copy();
    if(req.directionConstraint=="width-aligned")angles=[for(a in angles)if(a==0)a];
    if(angles.length==0)throw 'Nesting grain lock conflicts with rotations for ${req.id}';
    return {id:req.id,revision:req.revision,partId:req.partId,partRevision:req.partRevision,quantity:req.quantity,materialId:req.materialId,
      lengthUnit:"mm",thickness:req.thickness*scale,thicknessToleranceMinus:req.thicknessToleranceMinus*scale,thicknessTolerancePlus:req.thicknessTolerancePlus*scale,
      blankWidth:req.blankWidth*scale,blankHeight:req.blankHeight*scale,widthToleranceMinus:req.widthToleranceMinus*scale,widthTolerancePlus:req.widthTolerancePlus*scale,
      heightToleranceMinus:req.heightToleranceMinus*scale,heightTolerancePlus:req.heightTolerancePlus*scale,
      allowedRotations:req.allowedRotations.copy(),directionConstraint:req.directionConstraint,shape:shape,allowedAngles:angles};
  }
  static function pack(instances:Array<Instance>,stock:Array<CamNestStock>,margin:Float,spacing:Float,stats:CamNfpStats):Null<Array<Slot>> {
    var sheets=slots(stock);
    for(item in instances) {
      var assigned=false;
      for(sheet in sheets) {
        if(!compatible(item.requirement,sheet.stock))continue;
        var best:Null<CamNestPlaced> = null;
        for(variant in item.variants) {
          for(p in CamSheetNfp.candidates(sheet.placed,variant,sheet.width,sheet.height,margin,spacing,stats)) {
            var shape=SheetPolygon.transform(variant.shape,0,p.x,p.y),valid=true;
            for(other in sheet.placed)if(!SheetPolygon.separated(shape,other.shape,spacing)){valid=false;break;}
            if(!valid)continue;
            var candidate=CamSheetNfp.place(item.id,item.requirement.id,variant,p);
            if(best==null||candidate.y<best.y-SheetPolygon.EPS||Math.abs(candidate.y-best.y)<SheetPolygon.EPS&&candidate.x<best.x-SheetPolygon.EPS||
              Math.abs(candidate.y-best.y)<SheetPolygon.EPS&&Math.abs(candidate.x-best.x)<SheetPolygon.EPS&&variant.angle<best.variant.angle)best=candidate;
            break;
          }
        }
        if(best!=null){sheet.placed.push(best);assigned=true;break;}
      }
      if(!assigned)return null;
    }
    return sheets;
  }
  static function rectangle(shape:SheetShape):Bool {
    if(shape.length!=1||shape[0].length!=4)return false;
    var loop=shape[0];for(i in 0...4){var a=loop[i],b=loop[(i+1)%4];if(a.x!=b.x&&a.y!=b.y)return false;}return true;
  }
  /** A real guillotine partition is a competing candidate, so rectangular inputs cannot regress against it. */
  static function guillotine(instances:Array<Instance>,stock:Array<CamNestStock>,margin:Float,spacing:Float):Null<Array<Slot>> {
    for(item in instances)if(!rectangle(item.requirement.shape))return null;
    spacing=Math.ceil(spacing/SheetPolygon.QUANTUM)*SheetPolygon.QUANTUM;
    var sheets=slots(stock),free:Array<Array<FreeRect>> = [for(s in sheets)[{x:margin,y:margin,width:s.width-2*margin+spacing,height:s.height-2*margin+spacing}]];
    for(item in instances) {
      var found=false;
      for(index in 0...sheets.length) {
        var sheet=sheets[index];if(!compatible(item.requirement,sheet.stock))continue;
        var selected=-1,variant:Null<CamNestVariant> = null,waste=Math.POSITIVE_INFINITY;
        for(i in 0...free[index].length)for(v in item.variants) {
          if(v.angle%90!=0)continue;
          var b=SheetPolygon.bounds(v.shape),w=b.maxX-b.minX+spacing,h=b.maxY-b.minY+spacing,f=free[index][i];
          if(w<=f.width+SheetPolygon.EPS&&h<=f.height+SheetPolygon.EPS&&f.width*f.height-w*h<waste) {
            selected=i;variant=v;waste=f.width*f.height-w*h;
          }
        }
        if(selected<0||variant==null)continue;
        var f=free[index].splice(selected,1)[0],b=SheetPolygon.bounds(variant.shape),w=b.maxX-b.minX+spacing,h=b.maxY-b.minY+spacing;
        sheet.placed.push(CamSheetNfp.place(item.id,item.requirement.id,variant,SheetPolygon.point(f.x-b.minX,f.y-b.minY)));
        if(f.width-w>f.height-h) {
          if(f.width-w>SheetPolygon.EPS)free[index].push({x:f.x+w,y:f.y,width:f.width-w,height:f.height});
          if(f.height-h>SheetPolygon.EPS)free[index].push({x:f.x,y:f.y+h,width:w,height:f.height-h});
        } else {
          if(f.width-w>SheetPolygon.EPS)free[index].push({x:f.x+w,y:f.y,width:f.width-w,height:h});
          if(f.height-h>SheetPolygon.EPS)free[index].push({x:f.x,y:f.y+h,width:f.width,height:f.height-h});
        }
        found=true;break;
      }
      if(!found)return null;
    }
    return sheets;
  }
  static function sourceArea(sheets:Array<Slot>):Float {var area=0.0;for(s in sheets)if(s.placed.length>0)area+=s.width*s.height;return area;}
  static function remnantRects(sheet:Slot,spacing:Float,minimum:Float):Array<SheetRemnant> {
    var xs=[0.0,sheet.width],ys=[0.0,sheet.height];
    for(part in sheet.placed)for(loop in part.shape)for(p in loop) {
      for(x in [p.x-spacing,p.x+spacing])if(x>0&&x<sheet.width)xs.push(SheetPolygon.snap(x));
      for(y in [p.y-spacing,p.y+spacing])if(y>0&&y<sheet.height)ys.push(SheetPolygon.snap(y));
    }
    function axis(values:Array<Float>):Array<Float> {
      values.sort((a,b)->a<b?-1:a>b?1:0);var unique:Array<Float> = [];
      for(v in values)if(unique.length==0||v!=unique[unique.length-1])unique.push(v);
      if(unique.length<=64)return unique;
      return [for(i in 0...64)unique[Std.int((unique.length-1)*i/63)]];
    }
    xs=axis(xs);ys=axis(ys);var rectangles:Array<SheetRemnant> = [];
    for(row in 0...ys.length-1) {
      var runs:Array<SheetRemnant> = [];
      for(column in 0...xs.length-1) {
        var x=xs[column],y=ys[row],w=xs[column+1]-x,h=ys[row+1]-y;
        if(w<0.0005||h<0.0005)continue;
        var shape=SheetPolygon.rectangle(x,y,w,h),valid=true;
        for(p in sheet.placed)if(!SheetPolygon.separated(shape,p.shape,spacing)){valid=false;break;}
        if(!valid)continue;
        if(runs.length>0&&Math.abs(runs[runs.length-1].x+runs[runs.length-1].width-x)<SheetPolygon.EPS)runs[runs.length-1].width+=w;
        else runs.push({id:"",x:x,y:y,width:w,height:h});
      }
      for(run in runs) {
        var extended=false;
        for(r in rectangles)if(Math.abs(r.x-run.x)<SheetPolygon.EPS&&Math.abs(r.width-run.width)<SheetPolygon.EPS&&Math.abs(r.y+r.height-run.y)<SheetPolygon.EPS) {
          r.height+=run.height;extended=true;break;
        }
        if(!extended)rectangles.push(run);
      }
    }
    var retained=[for(r in rectangles)if(r.width>=minimum&&r.height>=minimum)r];
    for(i in 0...retained.length)retained[i].id='remnant-${sheet.index}-${i+1}';return retained;
  }
  public static function nest(requirements:Array<SheetPartRequirement>,stock:Array<CamNestStock>,options:CamNestOptions):CamNestResult {
    var started=Sys.time();
    if(requirements==null||requirements.length==0||stock==null||stock.length==0||options==null||!Math.isFinite(options.toolDiameter)||options.toolDiameter<=0||
      !Math.isFinite(options.gap)||options.gap<0||!Math.isFinite(options.edgeMargin)||options.edgeMargin<0)throw "Invalid sheet nesting inputs";
    var step:Float=options.rotationStepDegrees==null?90.0:options.rotationStepDegrees,seed=options.seed==null?1:options.seed;
    if(!Math.isFinite(step)||step<1||step>360)throw "Rotation step must be 1..360 degrees; explicit angle lists may contain arbitrary angles";
    var captured=[for(req in requirements)capture(req,step)],record=SheetProjectCodec.empty();
    record.stockSpecs=[for(s in stock)s.stock];record.requirements=captured;SheetProjectCodec.validate(record);
    var instanceCount=0;
    for(req in captured){if(req.quantity>1000-instanceCount)throw "Sheet nesting is limited to 1000 instances per batch";instanceCount+=req.quantity;}
    var stats:CamNfpStats={pairs:0,candidates:0,decompositions:0},instances:Array<Instance> = [];
    for(req in captured) {
      var base=CamSheetNfp.decompose(req.shape,stats),variants:Array<CamNestVariant> = [];
      for(angle in req.allowedAngles)variants.push({angle:angle,shape:SheetPolygon.transform(req.shape,angle,0,0),
        pieces:[for(piece in base)SheetPolygon.transform([piece],angle,0,0)[0]]});
      for(i in 0...req.quantity){var id='${req.id}-${i+1}';instances.push({id:id,requirement:req,area:SheetPolygon.materialArea(req.shape),variants:variants,tie:hash(id,seed)});}
    }
    instances.sort((a,b)->a.area!=b.area?(a.area>b.area?-1:1):a.tie!=b.tie?(a.tie<b.tie?-1:1):Reflect.compare(a.id,b.id));
    var margin=Math.ceil(options.edgeMargin/SheetPolygon.QUANTUM)*SheetPolygon.QUANTUM,spacing=options.toolDiameter+options.gap;
    var baseline=guillotine(instances,stock,margin,spacing),sheets=pack(instances,stock,margin,spacing,stats),engine="no-fit";
    if(baseline!=null&&(sheets==null||sourceArea(baseline)<sourceArea(sheets)-SheetPolygon.EPS)){sheets=baseline;engine="guillotine-candidate";}
    if(sheets==null)throw "No complete layout found with the requested sheets, rotations and deterministic search order";
    var plans:Array<SheetCutPlan> = [],total=0.0;
    var minimum:Float=options.minimumRemnantSide==null?100.0:options.minimumRemnantSide;
    if(!Math.isFinite(minimum)||minimum<=0)throw "Minimum remnant side must be positive";
    var batch=options.batchId==null?'nest-$seed':options.batchId;
    for(sheet in sheets)if(sheet.placed.length>0) {
      var plan:SheetCutPlan={schemaVersion:2,id:'$batch-sheet-${sheet.index+1}',revision:1,stockSpecId:sheet.stock.id,
        stockSpecRevision:sheet.stock.revision,stockSpecFingerprint:SheetPlanValidator.stockFingerprint(sheet.stock),lengthUnit:"mm",
        kerf:options.toolDiameter,edgeMargin:options.edgeMargin,requirements:SheetPlanValidator.captureRequirements(captured),placements:[],cuts:[],retainedRegionIds:[],
        layout:"freeform",batchId:batch,sheetIndex:sheet.index,nestGap:options.gap,
        freePlacements:[for(p in sheet.placed){id:p.id,requirementId:p.requirementId,x:p.x,y:p.y,angle:p.variant.angle,loops:p.shape}],
        remnants:remnantRects(sheet,spacing,minimum)};
      var checked=SheetPlanValidator.validate(plan,sheet.stock,captured);
      if(!checked.valid)throw 'Nesting independent validation failed: ${checked.messages.join("; ")}';
      total+=checked.partAreaMm2;plans.push(plan);
    }
    var baselineCount:Null<Int> = null;if(baseline!=null){var count=0;for(s in baseline)if(s.placed.length>0)count++;baselineCount=count;}
    var result:CamNestResult={plans:plans,requirements:captured,utilisation:total/sourceArea(sheets),elapsedSeconds:Sys.time()-started,
      engine:engine,guillotineSheets:baselineCount,stats:stats,seed:seed};
    validateBatch(result,[for(s in stock)s.stock]);return result;
  }
  public static function validateBatch(result:CamNestResult,stock:Array<SheetStockSpec>):Void {
    var counts=new Map<String,Int>(),ids=new Map<String,Bool>(),batch:Null<String> = null;
    for(plan in result.plans) {
      if(ids.exists(plan.id)||batch!=null&&plan.batchId!=batch)throw "Mixed or duplicate nesting batch plans";
      ids.set(plan.id,true);batch=plan.batchId;
      var found:Null<SheetStockSpec> = null;for(s in stock)if(s.id==plan.stockSpecId)found=s;
      var checked=SheetPlanValidator.validate(plan,found,result.requirements);if(!checked.valid)throw checked.messages.join("; ");
      for(p in plan.freePlacements){if(ids.exists(p.id))throw "Duplicate nesting batch placement";ids.set(p.id,true);var count=counts.exists(p.requirementId)?counts.get(p.requirementId):0;counts.set(p.requirementId,count+1);}
    }
    for(req in result.requirements)if(!counts.exists(req.id)||counts.get(req.id)!=req.quantity)throw "Nesting batch does not meet its exact quantities";
  }
}
