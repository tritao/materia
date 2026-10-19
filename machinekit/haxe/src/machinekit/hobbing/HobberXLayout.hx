package machinekit.hobbing;
import machinekit.motion.*;

typedef HobberXDimensions={railStart:Float,railLength:Float,screwStart:Float,screwLength:Float,nutStation:Float,motorFace:Float,motorRear:Float,floatingCentre:Float,deckStart:Float,bedRear:Float,bedLength:Float,bedCentre:Float,footRear:Float,footSpan:Float};

/** Reviewed design dimensions, regenerated/verified by HobberLoadingSizing.
    Loading retracts are rounded up from exact-solid corridor analysis, not
    inferred from part diameter alone. Changing the head/gripper requires that
    acceptance again; the mechanical envelope follows these dimensions. */
class HobberXLayout {
  public static inline final MIN=25.0;
  public static inline final RETRACT=110.0; // required 108.54, round up to 5 mm
  public static inline final TOP_RETRACT=145.0; // required 142.33, evaluated alternative
  public static inline final OVERTRAVEL=5.0;
  public static inline final END_ALLOWANCE=10.0;
  public static inline final BLOCK_FRONT=102.0;
  public static inline final BLOCK_REAR=300.0;
  public static inline final NUT_DATUM=250.0;
  public static inline final COLUMN_REAR=386.0;
  public static inline final BED_FRONT=-300.0;
  public static inline final FOOT_FRONT=-180.0;
  public static inline final FRONT_LIFT=35.0; // max 20 mm band + 8 mm sleeve + 2 mm cap + 5 mm clearance
  public static inline final FRONT_APPROACH=250.0; // robot transfer distance, not X-axis stroke
  static var frontCache:Null<HobberXDimensions>;
  public static function front():HobberXDimensions {if(frontCache==null)frontCache=dimensions(RETRACT);return frontCache;}
  static function up10(v:Float):Float return Math.ceil(v/10)*10;
  public static function dimensions(retract:Float):HobberXDimensions {
    var guide=LinearRailSystem.catalog().get("HGR25"),nut=new BallNut(),motor=ServoMotor.model("GENERIC-HV-SERVO-400W").rating;
    var railStart=MIN+BLOCK_FRONT-guide.blockLength/2-guide.railEndMargin-OVERTRAVEL;
    var railLength=up10(retract+BLOCK_REAR+guide.blockLength/2+guide.railEndMargin+OVERTRAVEL-railStart);
    var start=up10(NUT_DATUM+retract+nut.bodyLength/2+BallScrew.INPUT_JOURNAL_LENGTH+OVERTRAVEL+END_ALLOWANCE);
    var raceEnd=NUT_DATUM+MIN-nut.bodyLength/2-nut.flangeThickness-OVERTRAVEL-END_ALLOWANCE-BallScrew.JOURNAL_LENGTH;
    var carrierFront=NUT_DATUM+MIN-OVERTRAVEL-nut.bodyLength/2-nut.flangeThickness-20;
    var supportEnd=carrierFront-12.5-ScrewSupportUnit.bf12().length/2-END_ALLOWANCE;
    var length=up10(start-Math.min(raceEnd,supportEnd));
    var face=start+motor.shaftLength,rear=face+motor.bodyLength,floating=start-length+12.5;
    var bedRear=up10(Math.max(retract+COLUMN_REAR+OVERTRAVEL+35,Math.max(railStart+railLength+25,rear+20)));
    return {railStart:railStart,railLength:railLength,screwStart:start,screwLength:length,nutStation:start-NUT_DATUM,motorFace:face,motorRear:rear,
      floatingCentre:floating,deckStart:floating-10,bedRear:bedRear,bedLength:bedRear-BED_FRONT,bedCentre:(bedRear+BED_FRONT)/2,footRear:bedRear-120,footSpan:bedRear-120-FOOT_FRONT};
  }
}
