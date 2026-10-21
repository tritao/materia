package materia.sheet;

import materia.sheet.SheetPolygon.SheetShape;
import materia.sheet.SheetPolygon.SheetShapePlacement;
import materia.sheet.SheetPolygon.SheetRemnant;
import materia.project.MaterialDef;

/** Independent polygon validation for version-2 free-form layouts; no nesting heuristic is trusted here. */
class SheetFreeLayout {
  public static function placements(plan:SheetCutPlan):Array<SheetShapePlacement> {
    if(plan.schemaVersion!=2||plan.freePlacements==null)throw "Not a free-form layout";
    return cast plan.freePlacements;
  }
  public static function remnants(plan:SheetCutPlan):Array<SheetRemnant> {
    if(plan.schemaVersion!=2||plan.remnants==null)throw "Missing free-form remnants";
    return cast plan.remnants;
  }
  public static function checkData(plan:SheetCutPlan):Void {
    if(plan.schemaVersion!=2||plan.id==null||StringTools.trim(plan.id)==""||plan.revision<1||plan.layout!="freeform"||plan.lengthUnit!="mm"||plan.batchId==null||StringTools.trim(plan.batchId)==""||
      plan.sheetIndex==null||plan.sheetIndex<0||plan.nestGap==null||!Math.isFinite(plan.nestGap)||plan.nestGap<0||
      plan.freePlacements==null||plan.freePlacements.length==0||plan.freePlacements.length>10000||plan.remnants==null||
      plan.placements==null||plan.placements.length!=0||plan.cuts==null||plan.cuts.length!=0||plan.retainedRegionIds==null||plan.retainedRegionIds.length!=0)
      throw "Invalid free-form sheet layout schema or mixed guillotine data";
    for(p in placements(plan)) {
      if(p.id==null||p.id.length==0||p.requirementId==null||!Math.isFinite(p.x)||!Math.isFinite(p.y)||!Math.isFinite(p.angle)||p.angle<0||p.angle>=360)
        throw "Invalid free-form placement identity or pose";
      SheetPolygon.validate(p.loops);
    }
    for(r in remnants(plan))if(r.id==null||r.id.length==0||!Math.isFinite(r.x)||!Math.isFinite(r.y)||!Math.isFinite(r.width)||
      !Math.isFinite(r.height)||r.width<=0||r.height<=0)throw "Invalid free-form rectangular remnant";
  }
  static function xml(text:String):String return StringTools.replace(StringTools.replace(StringTools.replace(StringTools.replace(text,"&","&amp;"),"<","&lt;"),">","&gt;"),'"',"&quot;");
  public static function csv(plan:SheetCutPlan):String {
    function quote(value:Dynamic):String return '"'+StringTools.replace(Std.string(value),'"','""')+'"';
    function row(values:Array<Dynamic>):String return [for(v in values)quote(v)].join(",");
    var rows=["record_type,id,requirement_id,x_mm,y_mm,angle_deg,material_area_mm2,polygon_loops_json"];
    for(p in placements(plan))rows.push(row(["part",p.id,p.requirementId,p.x,p.y,p.angle,SheetPolygon.materialArea(p.loops),haxe.Json.stringify(p.loops)]));
    for(r in remnants(plan))rows.push(row(["remnant",r.id,"",r.x,r.y,0,r.width*r.height,haxe.Json.stringify(SheetPolygon.rectangle(r.x,r.y,r.width,r.height))]));
    return rows.join("\n")+"\n";
  }
  public static function svg(plan:SheetCutPlan,width:Float,height:Float):String {
    var out=new StringBuf();
    out.add('<svg xmlns="http://www.w3.org/2000/svg" width="${width}mm" height="${height}mm" viewBox="0 0 $width $height"><title>${xml(plan.id)} · free-form planning layout</title>');
    out.add('<g transform="translate(0,$height) scale(1,-1)"><rect width="$width" height="$height" fill="#eee"/>');
    for(p in placements(plan)) {
      var path=new StringBuf();for(loop in p.loops){path.add('M ${loop[0].x} ${loop[0].y}');for(i in 1...loop.length)path.add(' L ${loop[i].x} ${loop[i].y}');path.add(" Z ");}
      out.add('<path d="${path.toString()}" fill="#b9d8ec" fill-rule="evenodd" stroke="#234" stroke-width="0.2"><title>${xml(p.id)}</title></path>');
    }
    for(r in remnants(plan))out.add('<rect x="${r.x}" y="${r.y}" width="${r.width}" height="${r.height}" fill="#cce6c2" stroke="#463" stroke-width="0.2"/>');
    out.add("</g></svg>\n");return out.toString();
  }
  public static function validate(plan:SheetCutPlan,stock:SheetStockSpec,requirements:Array<SheetPartRequirement>,?source:StockPiece,
      ?customMaterials:Array<MaterialDef>):SheetPlanValidation {
    var result:SheetPlanValidation={valid:false,messages:[],regions:[],cutLines:[],kerfLossAreaMm2:0,marginWasteAreaMm2:0,
      discardedAreaMm2:0,retainedAreaMm2:0,partAreaMm2:0,sourceAreaMm2:0,areaBalanceErrorMm2:0};
    var errors=result.messages;
    try {
      checkData(plan);
      var placed=placements(plan),remaining=remnants(plan);
      if(stock==null||requirements==null||!Math.isFinite(plan.kerf)||plan.kerf<0||!Math.isFinite(plan.edgeMargin)||plan.edgeMargin<0)
        throw "Missing free-form stock/requirements or invalid spacing";
      if(plan.stockSpecId!=stock.id||plan.stockSpecRevision!=stock.revision||plan.stockSpecFingerprint!=SheetPlanValidator.stockFingerprint(stock))
        throw "Free-form plan is stale: stock specification changed";
      var known=false;
      try known=materia.project.MaterialLibrary.get(stock.materialId)!=null catch(_:Dynamic){}
      if(customMaterials!=null)for(material in customMaterials)if(material.id==stock.materialId)known=true;
      if(!known)throw "Unknown free-form stock material";
      var width=SheetPlanValidator.millimetres(source==null?stock.width:source.width,source==null?stock.lengthUnit:source.lengthUnit);
      var height=SheetPlanValidator.millimetres(source==null?stock.height:source.height,source==null?stock.lengthUnit:source.lengthUnit);
      var thickness=SheetPlanValidator.millimetres(source==null?stock.thickness:source.thickness,source==null?stock.lengthUnit:source.lengthUnit);
      if(width<=0||height<=0||thickness<=0||!Math.isFinite(width)||!Math.isFinite(height)||!Math.isFinite(thickness))throw "Invalid free-form source dimensions";
      if(source!=null&&(source.stockSpecId!=stock.id||source.materialId!=stock.materialId))throw "Free-form source does not match stock";
      result.sourceAreaMm2=width*height;
      var byId=new Map<String,SheetPartRequirement>(),refs=new Map<String,Bool>(),seen=new Map<String,Bool>(),counts=new Map<String,Int>();
      for(req in requirements){if(byId.exists(req.id))throw "Duplicate free-form requirement";byId.set(req.id,req);}
      if(plan.requirements==null||plan.requirements.length!=requirements.length)throw "Free-form requirement snapshots do not match";
      for(reference in plan.requirements) {
        var req=byId.get(reference.id);
        if(req==null||refs.exists(reference.id)||reference.revision!=req.revision||reference.fingerprint!=SheetPlanValidator.requirementFingerprint(req))
          throw "Free-form plan is stale: a requirement changed";
        refs.set(reference.id,true);
      }
      for(p in placed) {
        var req=byId.get(p.requirementId);
        if(req==null||seen.exists(p.id))throw "Unknown requirement or duplicate placement";seen.set(p.id,true);
        var scale=SheetPlanValidator.millimetres(1,req.lengthUnit),nominal=req.thickness*scale;
        if(req.materialId!=stock.materialId||thickness<nominal-req.thicknessToleranceMinus*scale-SheetPolygon.EPS||
          thickness>nominal+req.thicknessTolerancePlus*scale+SheetPolygon.EPS)throw "Free-form part material/thickness mismatch";
        if(req.shape==null)throw "Free-form part lacks its captured polygon outline";
        var angles:Array<Float> = req.allowedAngles==null?[for(a in req.allowedRotations)(a:Float)]:req.allowedAngles;
        if(angles.indexOf(p.angle)<0||req.directionConstraint=="width-aligned"&&p.angle!=0)throw "Free-form part violates its allowed rotation/grain lock";
        var original:SheetShape=[for(loop in req.shape)[for(v in loop)SheetPolygon.point(v.x*scale,v.y*scale)]];
        SheetPolygon.validate(original);
        var expected=SheetPolygon.transform(SheetPolygon.transform(original,p.angle,0,0),0,p.x,p.y);
        if(expected.length!=p.loops.length)throw "Placed outline differs from its requirement";
        for(i in 0...expected.length) {
          if(expected[i].length!=p.loops[i].length)throw "Placed outline vertex count differs from its requirement";
          for(j in 0...expected[i].length)if(Math.abs(expected[i][j].x-p.loops[i][j].x)>SheetPolygon.EPS||
            Math.abs(expected[i][j].y-p.loops[i][j].y)>SheetPolygon.EPS)throw "Placed outline differs from its requirement pose";
        }
        var bounds=SheetPolygon.bounds(p.loops);
        if(bounds.minX<plan.edgeMargin-SheetPolygon.EPS||bounds.minY<plan.edgeMargin-SheetPolygon.EPS||
          bounds.maxX>width-plan.edgeMargin+SheetPolygon.EPS||bounds.maxY>height-plan.edgeMargin+SheetPolygon.EPS)throw "Free-form edge margin violation";
        var count=counts.exists(req.id)?counts.get(req.id):0;counts.set(req.id,count+1);
        if(count+1>req.quantity)throw "Free-form sheet overproduces a requirement";
        result.partAreaMm2+=SheetPolygon.materialArea(p.loops);
      }
      var gap:Float=cast plan.nestGap;
      var spacing=plan.kerf+gap;
      for(i in 0...placed.length)for(j in i+1...placed.length)
        if(!SheetPolygon.separated(placed[i].loops,placed[j].loops,spacing))throw "Free-form parts overlap or violate tool/gap spacing";
      for(i in 0...remaining.length) {
        var r=remaining[i];if(seen.exists(r.id))throw "Duplicate remnant identity";seen.set(r.id,true);
        if(r.x<0||r.y<0||r.x+r.width>width+SheetPolygon.EPS||r.y+r.height>height+SheetPolygon.EPS)throw "Remnant lies outside source";
        var shape=SheetPolygon.rectangle(r.x,r.y,r.width,r.height);
        for(p in placed)if(!SheetPolygon.separated(shape,p.loops,spacing))throw "Remnant intersects a part or its cutting allowance";
        for(j in 0...i) {
          var b=remaining[j];if(r.x<b.x+b.width-SheetPolygon.EPS&&b.x<r.x+r.width-SheetPolygon.EPS&&
            r.y<b.y+b.height-SheetPolygon.EPS&&b.y<r.y+r.height-SheetPolygon.EPS)throw "Remnants overlap";
        }
        result.retainedAreaMm2+=r.width*r.height;
      }
      // Layout geometry cannot assign kerf vs unretained waste until a cutting sequence exists.
      result.discardedAreaMm2=result.sourceAreaMm2-result.partAreaMm2-result.retainedAreaMm2;
      if(result.discardedAreaMm2 < -Math.max(1e-6,result.sourceAreaMm2*1e-9))throw "Free-form area accounting exceeds its sheet";
      result.discardedAreaMm2=Math.max(0,result.discardedAreaMm2);
      result.areaBalanceErrorMm2=result.sourceAreaMm2-result.partAreaMm2-result.retainedAreaMm2-result.discardedAreaMm2;
    } catch(error:Dynamic) errors.push(Std.string(error));
    result.valid=errors.length==0;return result;
  }
}
