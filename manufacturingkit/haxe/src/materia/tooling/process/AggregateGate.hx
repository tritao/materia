package materia.tooling.process;

import materia.casting.Box3;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.tooling.ToolingCodec;
import materia.tooling.process.CastingProcessSpec.CastabilityLimits;

enum abstract AggregateGapKind(String) to String {
  var Wall = "wall-or-rib";
  var FlowPassage = "flow-passage";
  var SteelCover = "steel-cover";
  var SteelSpacing = "steel-clear-spacing";
  var WithinAssembly = "within-insert-assembly";
}

typedef AggregateBoundary = {
  var ids:Array<String>;
  var featureIds:Array<String>;
  var assemblyIds:Array<String>;
  var insert:Bool;
  var tooling:Bool;
}

typedef AggregateGap = {
  var id:String;
  var kind:AggregateGapKind;
  var first:AggregateBoundary;
  var second:AggregateBoundary;
  var featurePair:Array<String>;
  var sourceIds:Array<String>;
  var measuredMm:Float;
  var axis:Null<OrthogonalSections.SectionAxis>;
  /** Location envelope, not a fabricated closest-point witness or solid. */
  var region:Box3;
  var method:String;
  var witness:Null<{first:{x:Float,y:Float,z:Float},second:{x:Float,y:Float,z:Float}}>;
}

typedef AggregateFinding = {var gap:AggregateGap;var requiredMm:Float;}
typedef AggregateGroup = {
  var kind:AggregateGapKind;
  var featurePair:Array<String>;
  var sourceIds:Array<String>;
  var minimumMeasuredMm:Float;
  var requiredMm:Null<Float>;
  var witnessCount:Int;
  var region:Box3;
  var status:ToolingCheckStatus;
  var criterion:String;
}
typedef AggregateReport = {
  var sourceFingerprint:String;
  var limits:CastabilityLimits;
  var gaps:Array<AggregateGap>;
  var failures:Array<AggregateFinding>;
  var groups:Array<AggregateGroup>;
  var validation:Array<ToolingValidation>;
}

/** Aggregate-related recipe terms; these are not full reinforced-concrete code checks. */
class AggregateGate {
  public static function kinds():Array<AggregateGapKind>
    return [Wall,FlowPassage,SteelCover,SteelSpacing,WithinAssembly];

  public static function classes(a:AggregateBoundary,b:AggregateBoundary):Array<AggregateGapKind> {
    if(a.insert && b.insert) {
      var owners=a.assemblyIds.copy();
      for(id in b.assemblyIds)if(owners.indexOf(id)<0)owners.push(id);
      return owners.length==1 ? [WithinAssembly] : [SteelSpacing];
    }
    if(a.insert || b.insert)return [SteelCover];
    // A permanent resin wall also carries a process clearance when bounded by
				// forming tooling. The flow threshold never weakens the structural wall threshold.
    return a.tooling || b.tooling ? [Wall,FlowPassage] : [Wall];
  }

  public static function required(kind:AggregateGapKind,limits:CastabilityLimits):Null<Float> {
    var dg=limits.maximumAggregateSizeMm;
    if(dg==null)return null;
    return switch kind {
      case Wall:dg*limits.minimumWallAggregateRatio;
      case FlowPassage:limits.minimumPassageAggregateRatio==null ? null : dg*limits.minimumPassageAggregateRatio;
      case SteelCover:dg+limits.steelAggregateMarginMm;
      case SteelSpacing:Math.max(dg+limits.steelAggregateMarginMm,dg*limits.steelSpacingAggregateRatio);
      case WithinAssembly:null;
      default:throw "Unknown aggregate rule class";
    };
  }

  public static function captureLimit(limits:CastabilityLimits):Float {
    var result=0.0;
    for(kind in kinds()) {var value=required(kind,limits);if(value!=null)result=Math.max(result,value);}
    return result;
  }

  public static function withFill(sourceId:String,report:AggregateReport,fill:Null<FillAnalysis.FillReport>):AggregateReport {
    var coverage=report.validation.filter(row->StringTools.startsWith(row.rule,"process.castability.aggregate-coverage."));
    var result=evaluate(sourceId,report.sourceFingerprint,report.limits,report.gaps,fill,coverage);
    return result;
  }

  public static function rule(kind:AggregateGapKind):String return switch kind {
    case Wall:"process.castability.minimum-wall";
    case FlowPassage:"process.castability.minimum-passage";
    case SteelCover:"process.castability.steel-cover";
    case SteelSpacing:"process.castability.steel-spacing";
    case WithinAssembly:"process.castability.assembly-recess";
    default:throw "Unknown aggregate rule class";
  };

  public static function pairs(a:AggregateBoundary,b:AggregateBoundary):Array<Array<String>> {
    var result:Array<Array<String>>=[],seen=new Map<String,Bool>();
    for(first in a.featureIds)for(second in b.featureIds) {
      var pair=[first,second];pair.sort(Reflect.compare);
      var key=ToolingCodec.canonical(pair);
      if(!seen.exists(key)){seen.set(key,true);result.push(pair);}
    }
    result.sort((a,b)->Reflect.compare(ToolingCodec.canonical(a),ToolingCodec.canonical(b)));
    return result;
  }

  public static function fromSections(document:materia.tooling.ToolingDocument,sourceFingerprint:String,
      limits:CastabilityLimits,sections:OrthogonalSections.OrthogonalSectionReport):AggregateReport {
    function boundary(ids:Array<String>):AggregateBoundary {
      var owners:Array<String>=[],steel=false,tooling=false;
      for(feature in document.input.features)if(ids.indexOf(feature.id)>=0 && feature.kind!=materia.casting.CastingFeature.CastingFeatureKind.InsertRecess)tooling=true;
      for(feature in document.input.features)if(ids.indexOf(feature.id)>=0 && feature.kind==materia.casting.CastingFeature.CastingFeatureKind.InsertRecess) {
        steel=true;if(feature.owner!=null && owners.indexOf(feature.owner)<0)owners.push(feature.owner);
      }
      return {ids:ids.copy(),featureIds:ids.copy(),assemblyIds:owners,insert:steel,tooling:tooling};
    }
    var gaps:Array<AggregateGap>=[];
    for(span in sections.spans)for(region in materia.casting.BoxAlgebra.coalesce(span.strips)) {
      var a=boundary(span.lowerSourceIds),b=boundary(span.upperSourceIds),ids=a.ids.copy();
      for(id in b.ids)if(ids.indexOf(id)<0)ids.push(id);
      ids.sort(Reflect.compare);
      for(kind in classes(a,b))for(featurePair in pairs(a,b)) {
        var gap:AggregateGap={id:"gap",kind:kind,first:a,second:b,featurePair:featurePair,sourceIds:ids,
          measuredMm:span.thicknessMm,axis:span.axis,region:region,method:"Exact merged opposing-face material span",witness:null};
        gap.id="aggregate-gap-"+haxe.crypto.Sha256.encode(ToolingCodec.canonical(gap)).substr(0,24);gaps.push(gap);
      }
    }
    return evaluate(document.input.id,sourceFingerprint,limits,gaps);
  }

  static function envelope(a:Box3,b:Box3):Box3 {
    var x=Math.min(a.x,b.x),y=Math.min(a.y,b.y),z=Math.min(a.z,b.z);
    return {x:x,y:y,z:z,width:Math.max(a.x+a.width,b.x+b.width)-x,
      depth:Math.max(a.y+a.depth,b.y+b.depth)-y,height:Math.max(a.z+a.height,b.z+b.height)-z};
  }

  /** Same-assembly gaps have no spacing floor. Their recess access uses the
   * existing exact fill topology, including owner-attributed trapped pockets. */
  public static function evaluate(sourceId:String,sourceFingerprint:String,limits:CastabilityLimits,
      gaps:Array<AggregateGap>,?fill:FillAnalysis.FillReport,?coverage:Array<ToolingValidation>):AggregateReport {
    var sorted=gaps.copy();sorted.sort((a,b)->Reflect.compare(a.id,b.id));
    var grouped=new Map<String,AggregateGroup>(),failures:Array<AggregateFinding>=[];
    for(gap in sorted) {
      var key=ToolingCodec.canonical([gap.kind,gap.featurePair]);
      var group=grouped.get(key),minimum=required(gap.kind,limits);
      if(group==null) {
        group={kind:gap.kind,featurePair:gap.featurePair.copy(),sourceIds:[],minimumMeasuredMm:gap.measuredMm,
          requiredMm:minimum,witnessCount:0,region:gap.region,status:Pass,
          criterion:gap.kind==WithinAssembly ? "Open access to bulk resin; narrow recess criterion "+(limits.maximumAggregateSizeMm==null ? "unresolved" : Std.string(limits.maximumAggregateSizeMm))+" mm; no spacing floor on an open assembly gap" : "Minimum finished-resin clearance"};
        grouped.set(key,group);
      }
      group.minimumMeasuredMm=Math.min(group.minimumMeasuredMm,gap.measuredMm);
      group.witnessCount++;group.region=envelope(group.region,gap.region);
      for(id in gap.sourceIds)if(group.sourceIds.indexOf(id)<0)group.sourceIds.push(id);
      if(minimum!=null && gap.measuredMm<minimum-1e-7)failures.push({gap:gap,requiredMm:minimum});
      if(gap.kind==WithinAssembly) {
        if(fill==null)group.status=Pending;
        else {
          var owners=gap.first.assemblyIds.concat(gap.second.assemblyIds).concat(gap.first.featureIds).concat(gap.second.featureIds);
          if(fill.trappedPockets.filter(pocket->pocket.sourceIds.filter(id->owners.indexOf(id)>=0).length>0).length>0)group.status=Fail;
        }
      } else if(minimum==null)group.status=Pending;
      else if(gap.measuredMm<minimum-1e-7)group.status=Fail;
    }
    var groups=[for(group in grouped)group];
    if(coverage==null)coverage=[];
    for(group in groups)if(group.status!=Fail && coverage.filter(check->check.affectedIds.filter(id->group.sourceIds.indexOf(id)>=0).length>0).length>0)
      group.status=Unsupported;
    for(group in groups)group.sourceIds.sort(Reflect.compare);
    groups.sort((a,b)->Reflect.compare(ToolingCodec.canonical([a.kind,a.featurePair]),ToolingCodec.canonical([b.kind,b.featurePair])));
    var checks:Array<ToolingValidation>=[];
    for(kind in kinds()) {
      var rows=groups.filter(group->group.kind==kind),failed=rows.filter(group->group.status==Fail);
      var pending=rows.filter(group->group.status==Pending),unsupported=rows.filter(group->group.status==Unsupported),ids:Array<String>=[];
      for(group in failed)for(id in group.sourceIds)if(ids.indexOf(id)<0)ids.push(id);
      ids.sort(Reflect.compare);
      var status=failed.length>0 ? Fail : unsupported.length>0 ? Unsupported : pending.length>0 ? Pending : Pass;
      if(kind!=WithinAssembly && required(kind,limits)==null)status=Pending;
      if(kind==WithinAssembly && fill==null && rows.length>0)status=Pending;
      checks.push(FillAnalysis.check(rule(kind),ids.length==0 ? [sourceId] : ids,status,
        failed.length+" failing feature pairs; "+rows.length+" measured pairs for "+kind+". "
								+(kind==WithinAssembly ? "No aggregate flow-passage floor applies within one insert assembly; exact fill topology checks bulk access."
          : "Required "+required(kind,limits)+" mm. See aggregate-groups.csv; per-witness data remains attached."),limits.evidence));
    }
    return {sourceFingerprint:sourceFingerprint,limits:limits,gaps:sorted,failures:failures,groups:groups,validation:checks.concat(coverage)};
  }
}
