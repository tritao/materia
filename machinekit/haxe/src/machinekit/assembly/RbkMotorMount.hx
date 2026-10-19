package machinekit.assembly;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.modeling.Plane;
import cadkit.modeling.Location;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.SupplierCadComponent;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Pinned RBK assembly geometry, normalized to X across, Y up, Z motor-to-bearing.
 * Motor face Z=0; base bearing plane Y=0. The flange underhang needs base relief.
 * Axis frames do not establish shaft seating, bearing preload or load capacity.
 */
class RbkMotorMount extends MachineComponent {
    public final model:String;
    public final revision:String;
    public final assetSha256:String;
    public final shaftHeight:Float;
    public final pilotDiameter:Float;
    public final underhang:Float;
    final spec:Dynamic;
    static var recipe:Null<ComponentType>;
    static var supplierDirectory:Null<String>;

    public static function models():Array<String> return ["RBK12-57","RBK15-57","RBK15-86","RBK17-86"];
    /** Asset location is deployment configuration; saved recipes pin identity, never an absolute path. */
    public static function setSupplierDirectory(path:String):Void {
        if(path==null || path.length==0) throw "RBK supplier asset directory is empty";
        supplierDirectory=path;
    }
    public static function assetDirectory():String {
        if(supplierDirectory!=null) return supplierDirectory;
        var configured=Sys.getEnv("MATERIA_VALLDER_ROOT");
        if(configured!=null && configured.length>0) return configured;
        var root=Sys.getEnv("MATERIA_ROOT");
        return root==null ? "build/suppliers/vallder" : root+"/build/suppliers/vallder";
    }
    public function new(model:String="RBK12-57",revision:String="vallder-rbk-v1") {
        var data:Dynamic=haxe.Json.parse(RbkMountData.text()),entries:Array<Dynamic> = data.interfaces;
        var selected:Dynamic=null;
        for(entry in entries) if(entry.model==model && entry.revision==revision) selected=entry;
        if(selected==null) throw "Unknown pinned RBK interface revision";
        spec=selected;
        super('VALLDER-$model-$revision', 'Pinned $model motor/bearing mount', "unspecified");
        lockManufacturingMaterial("unspecified");
        this.model=model;this.revision=revision;assetSha256=spec.asset_sha256;
        shaftHeight=spec.shaft_height_mm;pilotDiameter=spec.pilot_diameter_mm;underhang=spec.underhang_mm;
        addConnector("base",Mount,AssemblyFrames.alongY(0,0,0,0,1,0));
        addConnector("motor",Mount,Solids.axial(0,shaftHeight,0));
        addConnector("shaftAxis",Shaft,Solids.axial(0,shaftHeight,0));
        var motorBolts:Array<Array<Float>> = spec.motor_bolts_xy;
        for(i in 0...motorBolts.length) addConnector('motorBolt${i+1}',Face,Solids.axial(motorBolts[i][0],motorBolts[i][1],0));
        var baseBolts:Array<Dynamic> = spec.base_holes;
        for(i in 0...baseBolts.length) addConnector('baseBolt${i+1}',Face,AssemblyFrames.alongY(baseBolts[i].x,0,baseBolts[i].z,0,1,0));
        var auxiliary:Array<Dynamic> = spec.auxiliary_holes;
        for(i in 0...auxiliary.length) addConnector('baseAux${i+1}',Face,AssemblyFrames.alongY(auxiliary[i].x,0,auxiliary[i].z,0,1,0));
    }
    /** Defensive evidence copy: nominal drawing dimensions, measured CAD bolt centres and limitations. */
    public function interfaceMetadata():Dynamic return haxe.Json.parse(haxe.Json.stringify(spec));
    public function motorThread():String return spec.motor_thread;
    /** Drawing value only; null where absent. Not a measured effective thread engagement. */
    public function motorThreadDepth():Null<Float> return spec.motor_thread_depth_mm;
    override public function hasGeometry():Bool return true;
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var path=assetDirectory()+"/cad/assets/"+assetSha256+".step";
        if(!sys.FileSystem.exists(path)) throw 'Missing pinned RBK CAD asset $assetSha256 in ${assetDirectory()}';
        var imported=new SupplierCadComponent(path,assetSha256,designation,description);
        try {
            var raw=imported.geometry();
            try {
                var first=spec.raw_orientation=="x_shaft_y_up";
                var location=new Location(new Plane(new Vector(0,first ? -spec.raw_base_y : 0,-spec.raw_motor_face_x),
                    new Vector(0,0,1),first ? new Vector(-1,0,0) : new Vector(0,1,0)));
                var normalized=raw.placed(location);
                raw.close();imported.close();
                return normalized;
            } catch(error:Dynamic) {raw.close();throw error;}
        } catch(error:Dynamic) {imported.close();throw error;}
    }
    public static function recipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.assembly.rbk-motor-mount",[
            P.choice("model",models(),"RBK12-57"),P.choice("revision",["vallder-rbk-v1"],"vallder-rbk-v1")
        ],v -> new RbkMotorMount(v.token("model"),v.token("revision")),true,false,null,false);
        return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function values():ComponentValues return new ComponentValues().setToken("model",model).setToken("revision",revision);
}
