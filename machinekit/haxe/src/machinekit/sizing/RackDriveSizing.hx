package machinekit.sizing;

import machinekit.transmission.HelicalPinion;
import machinekit.transmission.HelicalRack;
import machinekit.transmission.PreloadedPinionMount;
import machinekit.motion.PlanetaryReducer;

typedef RackCompliance = {
    /** Metres per newton, referred to the rack travel. */
    var meshMPerN:Float;
    var reducerMPerN:Float;
    var hingeMPerN:Float;
    var beltMPerN:Float;
    var stiffnessNPerM:Float;
    var backlashM:Float;
}

/** Explicit screening assumptions; excludes tooth root fillet/life factors and full ISO/JGMA certification. */
class RackDriveSizing {
    public static function compliance(pinion:HelicalPinion,rack:HelicalRack,?reducer:PlanetaryReducer,
            ?preload:PreloadedPinionMount,beltMPerN:Float=0):RackCompliance {
        rack.relation(pinion);
        if(!Math.isFinite(beltMPerN)||beltMPerN<0)throw "Invalid belt compliance";
        var radius=pinion.pitchDiameter()/2000;
        var mesh=1/(rack.n("meshStiffness")*1000);
        var gear=reducer==null?0:radius*radius/reducer.n("torsionalStiffness");
        // Radial displacement of a spring hinge maps to tangential lost travel by tan(alpha_t).
        var radial=preload==null?0:preload.n("springRate")*1000*Math.pow(preload.n("springArm")/preload.n("leverArm"),2);
        var hinge=preload==null?0:Math.pow(Math.tan(pinion.transversePressureAngle()),2)/radial;
        var backlash=(rack.n("backlash")+pinion.n("backlash"))/1000+(reducer==null?0:radius*reducer.backlashRadians());
        return {meshMPerN:mesh,reducerMPerN:gear,hingeMPerN:hinge,beltMPerN:beltMPerN,
            stiffnessNPerM:1/(mesh+gear+hinge+beltMPerN),backlashM:backlash};
    }
    /** Two equally loaded straight belt spans: k_theta = 2 EA R_driven² / span.
     * EA is N, span and driven pitch radius are metres; result is travel m/N.
     */
    public static function beltCompliance(radiusM:Float,drivenRadiusM:Float,spanM:Float,axialRigidityN:Float):Float {
        for(value in [radiusM,drivenRadiusM,spanM,axialRigidityN])if(!Math.isFinite(value)||value<=0)throw "Invalid reduction belt span";
        return radiusM*radiusM*spanM/(2*axialRigidityN*drivenRadiusM*drivenRadiusM);
    }
    /** Conservative one-tooth Lewis bending and Hertz line-contact screens.
     * Geometry factor uses virtual tooth count; no credit for overlap/contact ratio.
     * Steel E=210 GPa, nu=.3; 1.5 load factor; assumed root/contact limits 200/700 MPa.
     */
    public static function strength(pinion:HelicalPinion,tangentialN:Float,loadFactor:Float=1.5,preloadN:Float=0):{
            bendingMPa:Float,contactMPa:Float,separatingN:Float,axialN:Float,radialBearingN:Float,flankNormalN:Float,failures:Array<String>} {
        if(!Math.isFinite(tangentialN)||tangentialN<0||!Math.isFinite(loadFactor)||loadFactor<1)throw "Invalid tooth load";
        if(!Math.isFinite(preloadN)||preloadN<0)throw "Invalid tooth preload";
        var beta=pinion.n("helixAngle"),normal=pinion.n("pressureAngle"),angle=pinion.transversePressureAngle();
        var z=pinion.parameterCount("teeth")/Math.pow(Math.cos(beta),3);
        var lewis=Math.PI*(0.154-0.912/z),width=pinion.n("faceWidth"),moduleSize=pinion.n("normalModule");
        // The empirical Lewis factor here is valid only for the assumed 20-degree full-depth system.
        if(Math.abs(normal-20*Math.PI/180)>1e-8||lewis<=0)throw "Lewis screen supports 20 degree full-depth teeth";
        var drivenNormal=tangentialN/(Math.cos(angle)*Math.cos(beta));
        // With radial preload and closed opposing flanks, N+/- satisfy radial
        // equilibrium and their tangential difference transmits the drive force.
        // Without enough preload only the driving flank remains in contact.
        var normalN=Math.max(drivenNormal,(preloadN/(Math.sin(angle)*Math.cos(beta))+drivenNormal)/2);
        var flankTangential=normalN*Math.cos(angle)*Math.cos(beta);
        var bending=loadFactor*flankTangential/(width*moduleSize*lewis);
        var curvature=pinion.pitchDiameter()/2*Math.sin(angle); // rack flank is a plane
        var reducedE=210000/(2*(1-0.3*0.3));
        var contact=Math.sqrt(loadFactor*normalN*reducedE/(Math.PI*width*curvature));
        var separating=Math.abs(pinion.separatingForce(tangentialN)),axial=Math.abs(pinion.axialThrust(tangentialN));
        var failures:Array<String> = [];
        if(bending>200)failures.push("pinion: Lewis bending exceeds assumed 200 MPa");
        if(contact>700)failures.push("pinion: Hertz contact exceeds assumed 700 MPa");
        return {bendingMPa:bending,contactMPa:contact,separatingN:separating,axialN:axial,flankNormalN:normalN,
            radialBearingN:Math.sqrt(tangentialN*tangentialN+separating*separating),failures:failures};
    }
}
