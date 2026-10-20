import machinekit.transmission.HelicalPinion;
import machinekit.transmission.HelicalRack;

/** Purchasing requirement and catalogue screen; does not qualify an arbitrary steel rack. */
class SheetRouterRackRating {
    public static function check(pinion:HelicalPinion,rack:HelicalRack,reservedForceN:Float,flankNormalN:Float):Dynamic {
        rack.relation(pinion);
        if(!Math.isFinite(reservedForceN)||!Math.isFinite(flankNormalN)||reservedForceN<0||flankNormalN<0)
            throw "Rack rating requires finite non-negative demand";
        if(Math.min(pinion.n("faceWidth"),rack.n("faceWidth"))<19 || Math.abs(pinion.n("pressureAngle")-Math.PI/9)>1e-8)
            throw "Manufacturer rating requires at least 19 mm engagement and 20 degree normal pressure angle";
        if(pinion.parameterCount("teeth")!=20||Math.abs(pinion.n("normalModule")-1.5)>1e-9||
                Math.abs(pinion.n("helixAngle")-Math.acos(3*Math.PI/10))>1e-8)
            throw "No manufacturer feed-force rating for this rack/pinion geometry";
        // Catalogue feed force is a single-driving-flank rating. Include the extra
        // opposing-flank preload by referring the most loaded flank back to feed force.
        var loadedFlankN=flankNormalN*Math.cos(pinion.transversePressureAngle())*Math.cos(pinion.n("helixAngle"));
        var demand=Math.max(reservedForceN,loadedFlankN),rating=6000.0;
        // Conservatively retain the 50% mounting derating for short rack segments.
        var usable=rating*0.5;
        return {permissibleFeedN:rating,mountingDeratedFeedN:usable,demandN:demand,utilization:demand/usable,
            passes:demand<=usable,source:"https://www.atlantadrives.com/pdf/ads_rackratings-h.pdf",
            catalogue:"Module 1.5, 20 teeth, HPR quality 6; case-hardened 16MnCr5 pinion, hardened rack",
            standardRackWidthMm:19,selectedEnvelopeWidthMm:rack.n("faceWidth"),
            sourceBasis:"Manufacturer maximum feed force; grease lubrication, speed 1.5 m/s, uniform tooth loading, SB=1",
            qualification:"Candidate sourcing requirement only. Actual heat treatment, part drawing, face engagement, duty factors and mounting bolts must match before purchase."};
    }
}
