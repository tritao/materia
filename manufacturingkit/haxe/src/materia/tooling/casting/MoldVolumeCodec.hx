package materia.tooling.casting;

import materia.tooling.ToolingCodec;
import materia.tooling.panel.PanelCodec;
import materia.tooling.casting.CastingMoldPlan.MoldVolume;
import materia.tooling.casting.CastingMoldPlan.MoldPartMaterial;
import materia.tooling.casting.MoldPlan.MoldBox;

class MoldVolumeCodec {
    public static function encode(v:MoldVolume):String return ToolingCodec.canonical({format:"materia.mold-volume",version:1,volume:snapshot(v)});
    public static function decode(s:String):MoldVolume {
        if(s==null || haxe.io.Bytes.ofString(s).length>524288)throw "Mold volume exceeds limit";
        var r:Dynamic=haxe.Json.parse(s);
        if(r==null || Reflect.fields(r).length!=3 || Reflect.field(r,"format")!="materia.mold-volume" || Reflect.field(r,"version")!=1)throw "Unsupported mold volume format";
        return read(Reflect.field(r,"volume"));
    }
    public static function snapshot(v:MoldVolume):MoldVolume return read(v);
    static function read(r:Dynamic):MoldVolume {
        if(r==null || Reflect.fields(r).length!=3)throw "Invalid mold volume";
        var id:Dynamic=Reflect.field(r,"id"),token:Dynamic=Reflect.field(r,"material"),raw:Dynamic=Reflect.field(r,"pieces");
        if(!Std.isOfType(id,String) || !Std.isOfType(token,String) || !Std.isOfType(raw,Array))throw "Invalid mold volume fields";
        var name:String=cast id,text:String=cast token;PanelCodec.identifier(name);
        var material:MoldPartMaterial=switch text {case "eps":EPSStock;case "steel":SteelStock;case "former":FormerStock;default:throw "Invalid mold volume material";};
        var pieces:Array<MoldBox> = [],items:Array<Dynamic> = cast raw;
        if(items.length==0 || items.length>2048)throw "Invalid mold volume pieces";
        for(b in items) {
            if(b==null || Reflect.fields(b).length!=6)throw "Invalid mold box";
            function n(k:String):Float {
                var v:Dynamic=Reflect.field(b,k);
                if((!Std.isOfType(v,Float) && !Std.isOfType(v,Int)) || !Math.isFinite(v) || Math.abs(v)>1000000)throw "Invalid mold box number";return v;
            }
            var box:MoldBox={x:n("x"),y:n("y"),z:n("z"),width:n("width"),depth:n("depth"),height:n("height")};
            if(box.width<=0 || box.depth<=0 || box.height<=0)throw "Invalid mold box dimensions";pieces.push(box);
        }
        return {id:name,material:material,pieces:pieces};
    }
}
