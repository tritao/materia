package machinekit.assembly;

/** Fabrication layout for the supported twin HGR15 / SFU16 / RBK12 interface.
 * Coordinates are millimetres: X across, Y off the base, Z along travel.
 * Lengths and spacing vary; cross sections remain an explicitly checked interface family.
 */
class ScrewAxisLayout {
    public final travel:Float;
    public final screwLength:Float;
    public final railLength:Float;
    public final railSpacing:Float;
    public final railHoles:Array<Float>;
    public function new(travel:Float=200,screwLength:Float=400,railLength:Float=300,railSpacing:Float=134) {
        for(v in [travel,screwLength,railLength,railSpacing])
            if(!Math.isFinite(v) || v<=0)throw "Axis layout needs finite positive dimensions";
        if(screwLength<300 || screwLength>4000)throw "Axis screw length must be 300..4000 mm";
        if(railSpacing<134)throw "Axis rail spacing must clear the support and carriage hardware (134 mm minimum)";
        // Carriage spans -40..40 about its origin, which starts 40 mm into each rail.
        if(railLength<80+travel)throw "Axis travel puts the carriage beyond its rails";
        if(100+railLength>screwLength+23.5)throw "Axis rails would overlap the floating support hardware";
        // Nut origin is 140; race starts at 94.5 and ends at screwLength+29.5.
        if(140+travel+46>screwLength+29.5)throw "Axis travel puts the nut beyond the screw race";
        this.travel=travel;this.screwLength=screwLength;this.railLength=railLength;this.railSpacing=railSpacing;
        railHoles=[];var z=20.0;while(z<=railLength-20+1e-9){railHoles.push(z);z+=60;}
    }
    public function halfSpacing():Float return railSpacing/2;
    public function floatingZ():Float return screwLength+33.5;
    public function riserBolts():Array<Float> return [110.0,100+railLength-10];
    public function baseWidth():Float return railSpacing+66;
    public function carriageWidth():Float return railSpacing+46;
}
