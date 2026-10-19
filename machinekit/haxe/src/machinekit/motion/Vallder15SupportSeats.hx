package machinekit.motion;

/** Audited BK15/BF15 inner rings and seal sleeves, raw shaft frame. No preload inferred.
 * BK15 bearing inner rings: solids 11/7, Z=-7.5..1.5 / 1.5..10.5.
 * Rear sleeve solid25: -14..-7.5; front sleeve solid26: 10.5..20.
 * BF15 bearing inner ring solid7: -4.5..4.5. Re-audit if supplier bytes change.
 */
class Vallder15SupportSeats {
 public static inline var BK_HASH="4bf21bcf811deb78fac8c841b85de437c651159b853b34170e8f0a56f4aa6a80";
 public static inline var BF_HASH="ee201fb90dedf7814587e7471a753d4cc4da496806839f8f7bd154b8c1690e91";
 public static inline var FIXED_SHOULDER_Z:Float=-14;
 public static inline var FLOATING_FRONT_Z:Float=-4.5;
 public static function datums(model:String,hash:String):Array<{name:String,z:Float}> {
  return switch model {
   case "BK15":
    if(hash!=BK_HASH)throw "BK15 seating needs a fresh CAD audit";
    [{name:"raceShoulder",z:FIXED_SHOULDER_Z},{name:"retainingNutSeat",z:20.0},{name:"bearingRear",z:-7.5},{name:"bearingFront",z:10.5}];
   case "BF15":
    if(hash!=BF_HASH)throw "BF15 seating needs a fresh CAD audit";
    [{name:"bearingFront",z:FLOATING_FRONT_Z},{name:"bearingBack",z:4.5}];
   default:[];
  };
 }
}
