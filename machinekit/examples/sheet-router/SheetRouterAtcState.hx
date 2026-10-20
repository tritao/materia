import cnckit.CncController;
import sys.FileSystem;
import sys.io.File;

/** Atomic saved tool inventory and measured H values. The host compiles the next
 * cutting segment after this checkpoint, using the updated controller table.
 */
class SheetRouterAtcState {
    static function applyLengths(data:Dynamic,controller:CncController):Void {
        var tools=SheetRouterAtc.decodeTools(data.tools);
        for(t in tools)if(t.measuredGaugeTipMm!=null) {
            var length:Float=cast t.measuredGaugeTipMm;
            if(!Math.isFinite(length)||length<=0)throw "Invalid measured ATC length";
            var old:Null<toolpathkit.tool.Tool> = try controller.toolLibrary.tool(t.number) catch(error:Dynamic)null;
            if(old==null)controller.toolLibrary.set(new toolpathkit.tool.Tool(t.number,length/1000,t.diameterMm/1000));
            else {
                if(Math.abs(old.diameter-t.diameterMm/1000)>1e-9)throw "CNC/ATC tool diameters disagree";
                controller.setToolLength(t.number,length/1000);
            }
        }
    }
    public static function checkpoint(path:String,controller:CncController):String->Void {
        return json->{
            var data:Dynamic=haxe.Json.parse(json);
            if(data.version!=1)throw "Unsupported ATC checkpoint";
            applyLengths(data,controller);
            var temporary=path+".pending";
            if(FileSystem.exists(temporary))throw "Unresolved ATC checkpoint file requires recovery";
            File.saveContent(temporary,json+"\n");FileSystem.rename(temporary,path);
        };
    }
    public static function load(machine:SheetRouter,path:String,controller:CncController):SheetRouterAtc {
        var writer=checkpoint(path,controller),json=File.getContent(path);
        var state=SheetRouterAtc.fromJson(machine,json,writer);
        // Rehydrate H values before any program segment is compiled. Do not
        // clear the persisted in-progress/recovery marker.
        applyLengths(haxe.Json.parse(json),controller);
        return state;
    }
}
