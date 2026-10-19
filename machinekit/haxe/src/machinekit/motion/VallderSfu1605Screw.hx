package machinekit.motion;
import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Nominal Vallder SFU1605 end machining, products-sfu1605.png.
 * Input: D10x15, M12x1x14 including 2 mm relief, D12x25; floating D10x11.
 * Thread flanks are semantic; small end chamfers/shoulder fillets are omitted.
 * Lock thread uses its root envelope. No custom end machining is requested.
 */
class VallderSfu1605Screw extends LeadScrew {
    public static inline final FIXED_END:Float=54;
    public static inline final FLOAT_END:Float=11;
    static var recipe:Null<ComponentType>;
    public function new(length:Float=400) {
        if(!Math.isFinite(length) || length<=111 || length>4000) throw "SFU1605 length must contain both machined ends and its 46 mm nut";
        super(BallScrew.race(),length);
        addConnector("raceStart",Mount,Solids.axial(0,0,54));
        addConnector("fixedJournalStart",Axis,Solids.axial(0,0,29));
        addConnector("floatingSeat",Axis,Solids.axial(0,0,length-11));
        addConnector("outputRing",Mount,Solids.axial(0,0,length-3+1.15/2));
    }
    override public function geometry(detail:ComponentDetail=Preview):Part {
        var l=totalLength;
        var body=Solids.union([
            Solids.named(Part.cylinderSpan(5,0,15),"coupling-end"),
            Solids.named(Part.cylinderSpan(5.4,15,27),"M12x1-root"),
            Solids.named(Part.cylinderSpan(5,27,29),"thread-relief"),
            Solids.named(Part.cylinderSpan(6,29,54),"fixed-journal"),
            Solids.named(Part.cylinderSpan(8,54,l-11),"race"),
            Solids.named(Part.cylinderSpan(5,l-11,l-3),"floating-journal"),
            Solids.named(Part.cylinderSpan(4.8,l-3,l-1.85),"ring-groove"),
            Solids.named(Part.cylinderSpan(5,l-1.85,l),"tip")]);
        try {
            var threaded=new Part(cadkit.threads.Threads.attach(body.shape,"race:cyl.side",thread.threadSpec(),AxisMinimum));
            body.close();return threaded;
        } catch(e:Dynamic) {body.close();throw e;}
    }
    override public function axialRigidityAt(position:Float):Float {
        var diameter=position<15 ? 10.0 : position<27 ? 10.8 : position<29 ? 10.0 : position<54 ? 12.0 :
            position<totalLength-11 ? thread.rootDiameter() : position>=totalLength-3 && position<=totalLength-1.85 ? 9.6 : 10.0;
        return YOUNGS_MODULUS/1e6*Math.PI*diameter*diameter/4;
    }
    override public function axialComplianceBetween(from:Float,to:Float):Float {
        if(!Math.isFinite(from+to) || Math.min(from,to)<0 || Math.max(from,to)>totalLength) throw "Invalid screw compliance interval";
        var points=[0.0,15,27,29,54,totalLength-11,totalLength-3,totalLength-1.85,totalLength],sum=0.0;
        for(i in 0...points.length-1) {var a=Math.max(Math.min(from,to),points[i]),b=Math.min(Math.max(from,to),points[i+1]);if(b>a)sum+=(b-a)/axialRigidityAt((a+b)/2);}
        return sum;
    }
    public static function drawingRecipeType():ComponentType {
        if(recipe==null) recipe=new ComponentType("machinekit.motion.vallder-sfu1605-screw",[P.length("length",400)],v -> new VallderSfu1605Screw(v.number("length")));
        return recipe;
    }
    override public function componentType():Null<ComponentType> return drawingRecipeType();
    override public function values():ComponentValues return new ComponentValues().setNumber("length",totalLength).setToken("material",materialSpec());
}
