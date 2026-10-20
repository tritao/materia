package machinekit.tooling;

import cadkit.modeling.Part;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentRecipeSupport as P;
import materia.assembly.AssemblyFrames as F;

/** Nominal locating pins and stepped bore masks; supplier fits need qualification. */
class ToolingPin extends MachineComponent {
    static var recipe:Null<ComponentType>;
    static function parameters():Array<machinekit.component.ComponentParameter> return [
        P.length("diameter",6), P.length("length",40), P.length("headDiameter",0), P.length("headLength",0)
    ];
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.tooling.plain-pin",parameters(),v->new ToolingPin(v),false,false,null,false);
        return recipe;
    }
    public function new(?input:ComponentValues) {
        var v=P.resolve(parameters(),input);
        if(v.number("diameter")<=0 || v.number("length")<=0)throw "Pin dimensions must be positive";
        var headDiameter = v.number("headDiameter"), headLength = v.number("headLength");
        if (headDiameter < 0 || headLength < 0 || (headDiameter == 0) != (headLength == 0)
            || headDiameter > 0 && (headDiameter <= v.number("diameter") || headLength >= v.number("length")))
            throw "Invalid stepped tooling pin head";
        if(v.get("material")!=null && v.token("material")!="steel")throw "Tooling pin requires steel";
        var suffix = headDiameter == 0 ? "" : "-HEAD-" + headDiameter + "x" + headLength;
        var description = headDiameter == 0 ? "Nominal plain locating pin" : "Nominal stepped bore mask";
        super("TOOLING-PIN-"+v.number("diameter")+"x"+v.number("length")+suffix,description,"steel",false,v);
        addConnector("mount",Mount,F.identity());
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function hasGeometry():Bool return true;
    override public function setMaterial(spec:String):Void {if(spec!="steel")throw "Tooling pin requires steel";super.setMaterial(spec);}
    public static function boreMask(hole:materia.casting.InsertHole, ?plugLength:Float):MachineComponent {
        if (hole.thread != null) {
            if (plugLength == null)
                throw "Threaded bore protection requires a selected plug length: " + hole.id;
            if (hole.counterbore != null)
                throw "Threaded counterbore protection needs a sealing cap: " + hole.id;
            return machinekit.standard.SocketSetScrew.forThread(machinekit.structural.InsertHoles.threadSpec(hole.thread).mating(), cast plugLength);
        }
        return new ToolingPin(new ComponentValues().setNumber("diameter",hole.diameter).setNumber("length",hole.depth)
            .setNumber("headDiameter",hole.counterbore == null ? 0 : hole.counterbore.diameter)
            .setNumber("headLength",hole.counterbore == null ? 0 : hole.counterbore.depth));
    }

    override public function geometry(detail:ComponentDetail=Preview):Part {
        var shaft = Part.cylinder(parameterNumber("diameter")/2,parameterNumber("length"));
        if (parameterNumber("headDiameter") == 0)
            return shaft;
        return machinekit.component.Solids.building([shaft], parts -> {
            parts.push(Part.cylinder(parameterNumber("headDiameter")/2,parameterNumber("headLength")));
            return machinekit.component.Solids.union(parts);
        });
    }
}
