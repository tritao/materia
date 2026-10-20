package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.assembly.PosedParts;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import materia.casting.CastingFeature;
import materia.casting.CastingPrism;
import materia.casting.CastingPatterns;
import materia.casting.CastingEnvelope;
import materia.casting.CastingCavities;
import materia.casting.BoxAlgebra;

/** Source design cuts and openings; finished cavities have one neutral derivation. */
class MineralCasting extends MachineComponent {
	public final collisionHullQualification:CastingCollisionQualification;
    final features:Array<CastingFeature>;
	final envelope:Array<CastingPrism>;
    static var recipe:Null<ComponentType>;
    public static function parameters():Array<ComponentParameter> return [
        P.length("width",650),P.length("depth",550),P.length("height",120),
        P.length("footWidth",0),P.length("footHeight",0),P.length("footShoulder",0),P.length("cornerChamfer",0),
        P.text("features",'{"version":2,"features":[]}')];
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.structural.mineral-casting",parameters(),v->new MineralCasting(v),false,false,null,false);
        return recipe;
    }
    public function new(?overrides:ComponentValues) {
        var v=P.resolve(parameters(),overrides),cuts=CastingPatterns.decode(v.token("features"));
        v.setToken("features",CastingPatterns.encode(cuts));
        for(name in ["width","depth","height"])if(v.number(name)<=0)throw "Casting dimensions must be positive";
        if(v.get("material")!=null&&v.token("material")!="mineral cast")throw "Casting requires mineral cast";
        if(v.number("cornerChamfer")<0||2*v.number("cornerChamfer")>=Math.min(v.number("width"),v.number("depth")))throw "Casting chamfer consumes body";
        if(v.number("footWidth")==0&&(v.number("footHeight")!=0||v.number("footShoulder")!=0))throw "Incomplete casting foot";
        if(v.number("footWidth")!=0&&(v.number("footWidth")<=v.number("width")||v.number("footShoulder")<=0||v.number("footHeight")<=v.number("footShoulder")||v.number("footHeight")>=v.number("height")))throw "Invalid flared casting foot";
        super("MINERAL-CASTING","Mineral casting (purchase by casting mass, kg)","mineral cast",false,v);
        features=cuts;
		envelope = CastingEnvelope.build(v.number("width"), v.number("depth"), v.number("height"), v.number("cornerChamfer"),
			v.number("footWidth"), v.number("footHeight"), v.number("footShoulder"));
        var cavities=finishedCavities();
        if(parameterNumber("cornerChamfer")==0&&parameterNumber("footWidth")==0) {
            var material=materialBoxes();
            addFacet(CollisionHullFacet.fromBoxes([for(b in BoxAlgebra.compact(material))BoxAlgebra.limits(b)]));
			collisionHullQualification = new CastingCollisionQualification(Supported, "Exact disjoint-box material hulls");
		} else {
			collisionHullQualification = new CastingCollisionQualification(Unsupported,
				"Flared or chamfered envelopes require non-box material decomposition");
        }
		addFacet(collisionHullQualification);
        addConnector("mount",Mount,Solids.axial(0,0,0));addConnector("top",Mount,Solids.axial(0,0,v.number("height")));
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function hasGeometry():Bool return true;
    override public function setMaterial(spec:String):Void {if(spec!="mineral cast")throw "Casting requires mineral cast";super.setMaterial(spec);}
    static function prism(p:CastingPrism):Part {
        var local=Part.prism([for(v in p.profile)new Vector(v.x,v.y)],0,p.height);
        try {var placed=PosedParts.moved(local,p.pose);local.close();return placed;}catch(e:Dynamic){local.close();throw e;}
    }
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var parts:Array<Part> = [];
        try {
            for(p in castingEnvelope())parts.push(prism(p));
            var body=Solids.union(parts);parts=[];
            return Solids.building([body],owned->{
                for(c in finishedCavities())for(p in c.pieces)owned.push(Solids.named(prism(p),c.featureId));
                return Solids.cut(owned[0],owned.slice(1));
            });
        } catch(e:Dynamic){for(p in parts)p.close();throw e;}
    }
    public function castingFeatures():Array<CastingFeature> return materia.casting.CastingFeatureCodec.snapshot(features);
    public function castingEnvelope():Array<CastingPrism>
		return [for (prism in envelope) {
			id: prism.id,
			profile: [for (point in prism.profile) {x: point.x, y: point.y}],
			height: prism.height,
			pose: materia.casting.CastingRecordCodec.frame(prism.pose)
		}];
    public function finishedCavities():Array<materia.casting.CastingCavities.FinishedCastingCavity> return CastingCavities.derive(castingEnvelope(),features);
    public function materialBoxes():Array<materia.casting.Box3>
        return BoxAlgebra.material(castingEnvelope(), features);
    public function minimumSectionIy(x0:Float,x1:Float):Float {
        return BoxAlgebra.minimumIy(materialBoxes(),x0,x1);
    }
    public function castingByWeight():{material:String,kg:Float} return {material:materialSpec(),kg:massProperties().mass};
}
