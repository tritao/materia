package machinekit.assembly;
import machinekit.motion.Vallder15SupportSeats;
import machinekit.motion.SfuFamilyData;

/** Drawing-checked SFU20/BK15/BF15/DSG20/HG20 layout, mm. No purchased part is resized. */
class Sfu20AxisLayout {
 public final travel:Float;
 public final screwLength:Float;
 public final railLength:Float;
 public final railSpacing:Float;
 public static inline var SHAFT_Y:Float=28;
 public static inline var SCREW_Z:Float=40;
 public static inline var RAIL_Z:Float=140;
 public static inline var CARRIAGE_Z:Float=200;
 public static inline var CARRIAGE_Y:Float=50;
 public static inline var PLATE:Float=14;
 public function new(travel:Float=300,screwLength:Float=600,railLength:Float=450,railSpacing:Float=180) {
  for(v in [travel,screwLength,railLength,railSpacing])if(!Math.isFinite(v) || v<=0)throw "SFU20 axis needs finite positive dimensions";
  if(screwLength>4000 || screwLength<400)throw "Unsupported SFU20 screw length";
  if(railSpacing<150)throw "SFU20 rails must clear the DSG20 housing and hardware";
  if(CARRIAGE_Z+travel+50>RAIL_Z+railLength)throw "SFU20 carriage runs beyond rails";
  if(CARRIAGE_Z+travel+50>SCREW_Z+screwLength-18.5)throw "SFU20 carriage reaches floating support";
  if(RAIL_Z+railLength>SCREW_Z+screwLength-18.5)throw "SFU20 rails reach floating support";
  this.travel=travel;this.screwLength=screwLength;this.railLength=railLength;this.railSpacing=railSpacing;
 }
 public function fixedCentreZ():Float return SCREW_Z+SfuFamilyData.ends("SFU2005").fixedEnd+Vallder15SupportSeats.FIXED_SHOULDER_Z;
 public function floatingCentreZ():Float return SCREW_Z+screwLength-SfuFamilyData.ends("SFU2005").floatingEnd-Vallder15SupportSeats.FLOATING_FRONT_Z;
 public function baseLength():Float return SCREW_Z+screwLength+30-6;
 public function width():Float return railSpacing+64;
 public function railHoles():Array<Float> {var r:Array<Float> = [],z=20.0;while(z<=railLength-20+1e-9){r.push(z);z+=60;}return r;}
}
