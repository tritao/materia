package materia.tooling.casting;

import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.casting.MoldPlan.MoldBox;

/** Strict authored process inputs. Missing physical evidence stays explicit, never inferred. */
class CastingMoldOptionsCodec {
    public static function snapshot(o:CastingMoldOptions):CastingMoldOptions return read(o);
    public static function read(o:Dynamic):CastingMoldOptions {
        obj(o,["groutedJoints","precision","compaction","anchorage","requiresFlip","liftingEvidence"]);
        var joints:Array<GroutedJoint> = [];
        for(j in arr(o,"groutedJoints")) {
            obj(j,["id","insertIds","gapMm","jackingReference","boltSleeveReference","cureBeforeTorqueEvidence"]);
            joints.push({id:text(j,"id"),insertIds:strings(j,"insertIds"),gapMm:nullableNumber(j,"gapMm"),jackingReference:nullableText(j,"jackingReference"),boltSleeveReference:nullableText(j,"boltSleeveReference"),cureBeforeTorqueEvidence:strings(j,"cureBeforeTorqueEvidence")});
        }
        var precision:Array<PrecisionInterface> = [];
        for(p in arr(o,"precision")) {
            obj(p,["insertId","route","allowance","masterReference","masterFlatnessMm","releaseSpec","damGeometry","holeProtectionReference","pourGroup","evidence"]);
            var route:PrecisionRoute=switch text(p,"route"){case "replicated":Replicated;case "machined":Machined;case "as-cast":AsCast;default:throw "Invalid precision route";};
            precision.push({insertId:text(p,"insertId"),route:route,allowance:nullableNumber(p,"allowance"),masterReference:nullableText(p,"masterReference"),masterFlatnessMm:nullableNumber(p,"masterFlatnessMm"),releaseSpec:nullableText(p,"releaseSpec"),damGeometry:boxes(p,"damGeometry"),holeProtectionReference:nullableText(p,"holeProtectionReference"),pourGroup:nullableText(p,"pourGroup"),evidence:strings(p,"evidence")});
        }
        var c=Reflect.field(o,"compaction");obj(c,["method","bedInterfaceReference","frequencyHz","amplitudeMm","durationSeconds","evidence"]);
        var method:CompactionMethod=switch text(c,"method"){case "undeclared":Undeclared;case "vibration-table":VibrationTable;case "external-vibrators":ExternalVibrators;case "internal-poker":InternalPoker;case "none":NoCompaction;default:throw "Invalid compaction method";};
        var anchorage:Array<InsertAnchorage> = [];
        for(a in arr(o,"anchorage")) {
            obj(a,["insertId","kind","geometry","pulloutN","shearN","evidence"]);
            var kind:AnchorageKind=switch text(a,"kind"){case "pending":Pending;case "studs":Studs;case "undercut":Undercut;case "bonded-keys":BondedKeys;default:throw "Invalid anchorage kind";};
            var geometry=boxes(a,"geometry");if(geometry.length>0)throw "Anchorage geometry needs a corresponding source adapter revision before this template can consume it";
            anchorage.push({insertId:text(a,"insertId"),kind:kind,geometry:geometry,pulloutN:nullableNumber(a,"pulloutN"),shearN:nullableNumber(a,"shearN"),evidence:strings(a,"evidence")});
        }
        var flip:Dynamic=Reflect.field(o,"requiresFlip");if(!Std.isOfType(flip,Bool))throw "Expected handling flip flag";
        precision.sort((a,b)->Reflect.compare(a.insertId,b.insertId));anchorage.sort((a,b)->Reflect.compare(a.insertId,b.insertId));
        return {groutedJoints:joints,precision:precision,compaction:{method:method,bedInterfaceReference:nullableText(c,"bedInterfaceReference"),frequencyHz:nullableNumber(c,"frequencyHz"),amplitudeMm:nullableNumber(c,"amplitudeMm"),durationSeconds:nullableNumber(c,"durationSeconds"),evidence:strings(c,"evidence")},anchorage:anchorage,requiresFlip:flip,liftingEvidence:strings(o,"liftingEvidence")};
    }
    static function boxes(o:Dynamic,k:String):Array<MoldBox> {
        var out:Array<MoldBox> = [];
        for(b in arr(o,k)) {
            obj(b,["x","y","z","width","depth","height"]);
            var n:MoldBox={x:number(b,"x"),y:number(b,"y"),z:number(b,"z"),width:number(b,"width"),depth:number(b,"depth"),height:number(b,"height")};
            if(n.width<=0 || n.depth<=0 || n.height<=0)throw "Invalid process geometry";out.push(n);
        }return out;
    }
    static function obj(o:Dynamic,keys:Array<String>):Void {
        if(o==null || !Reflect.isObject(o) || Std.isOfType(o,Array) || Reflect.fields(o).length!=keys.length)throw "Invalid mold process record";
        for(k in keys)if(!Reflect.hasField(o,k))throw "Missing mold process field "+k;
    }
    static function arr(o:Dynamic,k:String):Array<Dynamic> {
        var v:Dynamic=Reflect.field(o,k);if(!Std.isOfType(v,Array))throw "Expected mold process array";var a:Array<Dynamic> = cast v;if(a.length>512)throw "Too many mold process records";return a;
    }
    static function strings(o:Dynamic,k:String):Array<String> {var out:Array<String> = [];for(v in arr(o,k)){if(!Std.isOfType(v,String))throw "Expected evidence reference";var s:String=cast v;if(StringTools.trim(s).length==0 || s.length>2048)throw "Invalid evidence reference";out.push(s);}return out;}
    static function text(o:Dynamic,k:String):String {var v:Dynamic=Reflect.field(o,k);if(!Std.isOfType(v,String))throw "Expected mold process text";var s:String=cast v;if(StringTools.trim(s).length==0 || s.length>2048)throw "Invalid mold process text";return s;}
    static function nullableText(o:Dynamic,k:String):Null<String> return Reflect.field(o,k)==null?null:text(o,k);
    static function number(o:Dynamic,k:String):Float {var v:Dynamic=Reflect.field(o,k);if((!Std.isOfType(v,Float) && !Std.isOfType(v,Int)) || !Math.isFinite(v))throw "Expected finite process number";return v;}
    static function nullableNumber(o:Dynamic,k:String):Null<Float> {if(Reflect.field(o,k)==null)return null;var n=number(o,k);if(n<0)throw "Negative process dimension/capacity";return n;}
}
