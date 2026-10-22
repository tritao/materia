package machinekit.transmission;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.InertiaTensor;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.transmission.GearRackSpec.GearRackCatalog;
import machinekit.assembly.AxisBuilder;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;

/** One catalogue section; the host mount derives its holes from mount1…N.
 * Nominal tooth drawing; supplier mass/holes/ratings. No rack seat is generated.
 * CAD axes match HelicalRack: length +Z, teeth +Y, face width +X.
 */
class GearRack extends MachineComponent {
    public final spec:GearRackSpec;
    static var recipe:Null<ComponentType>;
    static function inputs():Array<ComponentParameter> return [P.choice("designation",GearRackCatalog.catalog().designations(),"29-15-105")];
    public static function catalogue(designation:String):GearRack return new GearRack(new ComponentValues().setToken("designation",designation));
    public function new(?values:ComponentValues) {
        var v=P.resolve(inputs(),values),s=GearRackCatalog.catalog().get(v.token("designation"));
        super("ATLANTA-"+s.designation,"ATLANTA HPR helical rack "+s.designation,"steel",true,v);spec=s;
        for(i in 0...spec.holeCount)addConnector('mount${i+1}',Mount,
            AxisBuilder.orient(spec.faceWidthMm,-spec.pitchHeightMm+spec.holeHeightMm,spec.holeEndMm+i*spec.holePitchMm,[-1.0,0,0],[0.0,0,1]));
        for(i in 0...2)addConnector('pinPilot${i+1}',Mount,
            AxisBuilder.orient(spec.faceWidthMm,-spec.pitchHeightMm+spec.holeHeightMm,i==0?spec.pinEndMm:spec.lengthMm-spec.pinEndMm,[-1.0,0,0],[0.0,0,1]));
        addConnector("back",Mount,F.translation(0,-spec.pitchHeightMm,0));
        addConnector("referenceSide",Mount,AxisBuilder.orient(spec.faceWidthMm,0,0,[-1.0,0,0],[0.0,0,1]));
        addConnector("pitch",Pitch,Solids.axial(spec.faceWidthMm/2,0,0));
        addConnector("end",Mount,Solids.axial(spec.faceWidthMm/2,0,spec.lengthMm));
        // Datasheet mass; envelope inertia is an explicit nominal approximation.
        var m=spec.massKg,b=spec.faceWidthMm,h=spec.overallHeightMm,l=spec.lengthMm;
        declareMass(m,new Vector(b/2,-spec.pitchHeightMm+h/2,l/2),
            new InertiaTensor(m*(h*h+l*l)/12,0,0,m*(b*b+l*l)/12,0,m*(b*b+h*h)/12));
    }
    public static function recipeType():ComponentType {
        if(recipe==null)recipe=new ComponentType("machinekit.transmission.gear-rack",inputs(),v->new GearRack(v),true,false);return recipe;
    }
    override public function componentType():Null<ComponentType> return recipeType();
    override public function presentationView():Null<ComponentView> return ComponentView.lengthAlongZ();
    override public function hasGeometry():Bool return true;
    public function helixAngle():Float return Math.acos(Math.PI*spec.normalModule/(spec.lengthMm/spec.teeth));
    public function mountingConnectors():Array<Connector> return [for(i in 0...spec.holeCount)connector('mount${i+1}')];
    public function mountingHole():{diameter:Float,counterboreDiameter:Float,counterboreDepth:Float,reference:String}
        return {diameter:spec.boreMm,counterboreDiameter:spec.counterboreMm,counterboreDepth:spec.counterboreDepthMm,reference:GearRackCatalog.SOURCE+"#page=76"};
    public function mountingThread():cadkit.threads.ThreadSpec return machinekit.standard.FastenerThreads.metric(6,1).mating();
    /** Published perfect-condition rating, not automatically admissible for an unverified pinion.
     * ATLANTA materials; case-hardened 16MnCr5 pinion; v=1.5 m/s; Sb=fn=KHb=1;
     * correct alignment/grease; check screws separately. Application factors still apply.
     */
    public function publishedFeedForceN(pinionTeeth:Int):Float {
        var force=switch pinionTeeth {case 15:spec.force15N;case 20:spec.force20N;case _:throw "No reviewed feed-force table row for this pinion";};
        return force*spec.sectionForceFactor;
    }
    /** Engineering relation fields (stiffness/backlash/efficiency) remain assumptions,
     * separate from the catalogue capacity and dimensional data above. */
    public function nominalRack():HelicalRack return new HelicalRack(new ComponentValues()
        .setNumber("normalModule",spec.normalModule).setInteger("teeth",spec.teeth).setNumber("faceWidth",spec.faceWidthMm)
        .setNumber("helixAngle",helixAngle()).setNumber("barHeight",spec.pitchHeightMm-1.25*spec.normalModule).setToken("hand","right"));
    static function bore(radius:Float,start:Float,end:Float,y:Float,z:Float):Part {
        var raw=Part.cylinder(radius,end-start);
        try {var result=PosedParts.moved(raw,AxisBuilder.orient(start,y,z,[0.0,1,0],[1.0,0,0]));raw.close();return result;}
        catch(e:Dynamic){raw.close();throw e;}
    }
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var body=nominalRack().geometry(detail),tools:Array<Part> = [];
        try {
            var y=-spec.pitchHeightMm+spec.holeHeightMm;
            for(i in 0...spec.holeCount) {
                var z=spec.holeEndMm+i*spec.holePitchMm;
                tools.push(bore(spec.boreMm/2,-0.1,spec.faceWidthMm+0.1,y,z));
                tools.push(bore(spec.counterboreMm/2,-0.1,spec.counterboreDepthMm,y,z));
            }
            for(z in [spec.pinEndMm,spec.lengthMm-spec.pinEndMm])tools.push(bore(spec.pinPilotMm/2,-0.1,spec.faceWidthMm+0.1,y,z));
        }catch(e:Dynamic){body.close();for(t in tools)t.close();throw e;}
        return Solids.cut(body,tools);
    }
}
