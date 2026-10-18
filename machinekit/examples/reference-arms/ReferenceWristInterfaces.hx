import haxe.io.Bytes;
import materia.assembly.AssemblyRecord.AssemblyFrame;

@:wire typedef WristPin = {
  @:id(1) var number:Int;
  @:id(2) var functions:Array<String>;
}
/** An observed CAD insert face, not a qualified cable seating/key frame. */
@:wire typedef WristGeometryBinding = {
  @:id(1) var sourceStep:String;
  @:id(2) var sourceStepSha256:String;
  @:id(3) var solidIndex:Int;
  @:id(4) var insertFaceIndex:Int;
  @:id(5) var contactFaces:Array<Int>;
  @:id(6) var stepMetresPerUnit:Float;
  @:id(7) var insertFrameStepMm:AssemblyFrame;
  @:id(8) var stepMetresToParent:AssemblyFrame;
  @:id(9) var observedFrame:AssemblyFrame;
  @:id(10) var parentLink:String;
  @:id(11) var parentUrdfSha256:String;
  @:id(12) var registrationStatus:String;
  @:id(13) var clockReference:String;
  @:id(14) var limitations:Array<String>;
}
@:wire typedef WristPort = {
  @:id(1) var id:String;
  @:id(2) var thread:String;
  @:id(3) var sex:String;
  @:id(4) var contactCount:Int;
  @:id(5) var coding:Null<String>;
  @:id(6) var pins:Array<WristPin>;
  @:id(7) var sharedPowerGroup:String;
  @:id(8) var geometryStatus:String;
  @:id(9) var matingFrame:Null<AssemblyFrame>;
  @:id(10) var pinoutSource:String;
  @:id(11) var pinoutSourceSha256:String;
  @:id(12) var parentLink:Null<String>;
  @:id(13) var placementSource:Null<String>;
  @:id(14) var observedGeometry:Null<WristGeometryBinding>;
}
@:wire typedef WristPowerEvidence = {
  @:id(1) var id:String;
  @:id(2) var singlePinContinuousA:Float;
  @:id(3) var dualPinContinuousA:Null<Float>;
  @:id(4) var peakA:Null<Float>;
  @:id(5) var peakDurationS:Null<Float>;
  @:id(6) var peakDutyFraction:Null<Float>;
  @:id(7) var averageWindowS:Null<Float>;
  @:id(8) var maxCapacitanceUf:Null<Float>;
  @:id(9) var source:String;
  @:id(10) var applicability:String;
}
/** Catalogue compatibility is distinct from an identified, dimensioned cable. */
@:wire typedef WristCableEvidence = {
  @:id(1) var id:String;
  @:id(2) var manufacturer:String;
  @:id(3) var selector:String;
  @:id(4) var selectorKind:String;
  @:id(5) var portId:String;
  @:id(6) var source:String;
  @:id(7) var exactPartNumber:Null<String>;
  @:id(8) var drawingSource:Null<String>;
  @:id(9) var geometrySha256:Null<String>;
  @:id(10) var seatingOffsetMm:Null<Float>;
  @:id(11) var keyClockingRad:Null<Float>;
  @:id(12) var cableDiameterMm:Null<Float>;
  @:id(13) var minBendRadiusMm:Null<Float>;
  @:id(14) var fixedLeadMm:Null<Float>;
  @:id(15) var limitations:Array<String>;
}
/** Design preference; never a certificate of physical mating or cable suitability. */
@:wire typedef WristLayoutSelection = {
  @:id(1) var cableId:String;
  @:id(2) var connectorExitDeg:Float;
  @:id(3) var preferredStudyClockDeg:Float;
  @:id(4) var status:String;
  @:id(5) var source:String;
}
@:wire typedef WristProfile = {
  @:id(1) var id:String;
  @:id(2) var modelId:String;
  @:id(3) var manufacturer:String;
  @:id(4) var hardwareRevision:String;
  @:id(5) var applicability:String;
  @:id(6) var voltageModes:Array<Float>;
  @:id(7) var ports:Array<WristPort>;
  @:id(8) var ratings:Array<WristPowerEvidence>;
  @:id(9) var limitations:Array<String>;
  @:id(10) var cables:Array<WristCableEvidence>;
  @:id(11) var layoutSelection:Null<WristLayoutSelection>;
}
@:wire typedef WristInterfaceCatalog = {
  @:id(1) var schema:Int;
  @:id(2) var sourceSha256:String;
  @:id(3) var captured:String;
  @:id(4) var profiles:Array<WristProfile>;
}
typedef WristUse = {
  var powerPort:String;
  var signalPort:String;
  var voltage:Float;
  var powerMode:String;
  var signalMode:String;
  var continuousA:Float;
  var peakA:Float;
  var peakDurationS:Float;
  var peakDutyFraction:Float;
  var averageWindowA:Float;
  var capacitanceUf:Float;
  var requestedSignals:Array<String>;
}

/** Documented vendor interfaces, independent of the custom 5 A service module.
    Electrical facts can be used without inventing a mating location in a source mesh. */
class ReferenceWristInterfaces {
  static function number(value:Null<Float>):Float {if(value==null) throw "Missing documented numeric limit";return cast value;}
  public static function digest(bytes:Bytes):String {
    var hash=haxe.crypto.Sha256.make(bytes),out=new StringBuf();
    for(i in 0...hash.length) out.add(StringTools.hex(hash.get(i),2).toLowerCase());return out.toString();
  }
  public static function load(directory:String,expected:String,model:String):Array<WristProfile> {
    var bytes=sys.io.File.getBytes(haxe.io.Path.directory(directory)+"/assets/wrist-interfaces.wire.json");
    if(digest(bytes)!=expected) throw "Changed manufacturer wrist-interface catalog; prepare it again";
    var catalog:WristInterfaceCatalog=haxeon.wire.JsonWire.decode(bytes.toString());
    if(catalog.schema!=1) throw "Unsupported wrist-interface catalog";
    var selected=[for(p in catalog.profiles) if(p.modelId==model) p];
    for(p in selected) {
      validate(p);
      for(port in p.ports) if(port.observedGeometry!=null) {
        var binding:WristGeometryBinding=port.observedGeometry;
        if(digest(sys.io.File.getBytes(directory+"/robot.urdf"))!=binding.parentUrdfSha256)
          throw "Changed wrist observation parent URDF; registration must be reviewed";
      }
    }
    return selected;
  }
  /** Serial numbers stay strings: the published cutover exceeds a 32-bit integer. */
  public static function ur5eForSerial(profiles:Array<WristProfile>,serial:String):WristProfile {
    if(serial==null || !~/^[0-9]{11}$/.match(serial)) throw "UR5e needs its documented 11-digit serial number";
    var id=serial<"20245501103" ? "ur5e-m8-male" : "ur5e-m8-female";
    for(p in profiles) if(p.id==id && p.modelId=="ur5e") return p;
    throw "UR5e connector revision is not in the prepared catalog";
  }
  public static function port(profile:WristProfile,id:String):WristPort {
    for(p in profile.ports) if(p.id==id) return p;throw 'Unknown wrist port $id';
  }
  public static function pin(profile:WristProfile,portId:String,functionName:String):Int {
    for(p in port(profile,portId).pins) if(p.functions.indexOf(functionName)>=0) return p.number;
    throw 'Function $functionName is not on $portId';
  }
  public static function validate(profile:WristProfile):Void {
    if(profile.id.length==0 || profile.ratings.length==0 || profile.ports.length==0) throw "Incomplete manufacturer wrist profile";
    var ids=new Map<String,Bool>();
    for(p in profile.ports) {
      if(ids.exists(p.id) || p.pins.length!=p.contactCount || p.sharedPowerGroup.length==0) throw "Invalid wrist connector inventory";
      ids.set(p.id,true);var pins=new Map<Int,Bool>();
      for(pin in p.pins) {
        if(pin.number<1 || pin.number>p.contactCount || pins.exists(pin.number) || pin.functions.length==0) throw "Invalid wrist pin numbering";
        pins.set(pin.number,true);
      }
      if(p.geometryStatus!="qualified" && p.matingFrame!=null) throw "Unqualified wrist port must not invent a mating frame";
      if(p.geometryStatus=="qualified" && (p.matingFrame==null || p.parentLink==null || p.placementSource==null)) throw "Qualified wrist placement needs a parent link, frame and source";
    }
    var cableIds=new Map<String,Bool>();
    for(c in profile.cables) {
      if(c.id.length==0 || c.selector.length==0 || c.source.length==0 || cableIds.exists(c.id)) throw "Invalid cable evidence inventory";
      cableIds.set(c.id,true);port(profile,c.portId);
    }
    if(profile.layoutSelection!=null) {
      var selected:WristLayoutSelection=profile.layoutSelection;
      if(!cableIds.exists(selected.cableId) || !Math.isFinite(selected.connectorExitDeg) || !Math.isFinite(selected.preferredStudyClockDeg))
        throw "Invalid wrist layout selection";
    }
    for(r in profile.ratings) {
      for(v in [r.singlePinContinuousA]) if(!Math.isFinite(v) || v<=0) throw "Invalid manufacturer current rating";
      if(r.dualPinContinuousA!=null && (!Math.isFinite(r.dualPinContinuousA) || r.dualPinContinuousA<=0)) throw "Invalid dual-pin current rating";
    }
  }
  /** The caller must select a documented rating edition: no implicit 5 A default. */
  public static function checkUse(profile:WristProfile,ratingId:String,use:WristUse):Void {
    validate(profile);
    var found:WristPowerEvidence=null;for(r in profile.ratings) if(r.id==ratingId) found=r;
    if(found==null) throw "Select an explicit manufacturer power-rating evidence edition";
    var rating:WristPowerEvidence=found;
    for(v in [use.voltage,use.continuousA,use.peakA,use.peakDurationS,use.peakDutyFraction,use.averageWindowA,use.capacitanceUf])
      if(!Math.isFinite(v) || v<0) throw "Nonfinite or negative wrist load";
    if(profile.voltageModes.indexOf(use.voltage)<0 || use.voltage==0) throw "Unsupported powered wrist voltage mode";
    if(use.peakDutyFraction>1 || use.peakA<use.continuousA || use.averageWindowA<use.continuousA) throw "Inconsistent wrist load envelope";
    var power=port(profile,use.powerPort),signal=port(profile,use.signalPort);
    pin(profile,power.id,"POWER");pin(profile,power.id,"GND");
    var continuous=rating.singlePinContinuousA;
    if(use.powerMode=="dual-pin") {
      if(rating.dualPinContinuousA==null) throw "Dual-pin power is not documented on this wrist";
      pin(profile,power.id,"PWR_EXTRA");pin(profile,power.id,"GND_EXTRA");continuous=number(rating.dualPinContinuousA);
    } else if(use.powerMode!="single-pin") throw "Unknown wrist power mode";
    var peak=rating.peakA==null ? continuous : number(rating.peakA);
    if(use.continuousA>continuous+1e-9 || use.averageWindowA>continuous+1e-9 || use.peakA>peak+1e-9) throw "Wrist shared power rating exceeded";
    if(use.peakA>continuous) {
      if(rating.peakDurationS==null || rating.peakDutyFraction==null || rating.averageWindowS==null) throw "No qualified transient overload envelope";
      if(use.peakDurationS>number(rating.peakDurationS) || use.peakDutyFraction>number(rating.peakDutyFraction)) throw "Wrist peak duration/duty exceeded";
      var fraction=Math.max(use.peakDutyFraction,Math.min(1,use.peakDurationS/number(rating.averageWindowS)));
      if(use.averageWindowA+1e-9<use.peakA*fraction+use.continuousA*(1-fraction)) throw "Stated window average understates the load envelope";
    }
    if(rating.maxCapacitanceUf!=null && use.capacitanceUf>number(rating.maxCapacitanceUf)) throw "Wrist capacitive load exceeded";
    var rs485=false;for(p in signal.pins) for(f in p.functions) if(StringTools.startsWith(f,"RS485")) rs485=true;
    if(use.signalMode=="rs485" && !rs485) throw "RS485 is not on the selected connector";
    if(use.signalMode!="rs485" && use.signalMode!="io") throw "Unknown wrist signal mode";
    for(f in use.requestedSignals) {
      pin(profile,signal.id,f);
      if(["POWER","GND","PWR_EXTRA","GND_EXTRA","UNASSIGNED"].indexOf(f)>=0) throw "Reserved/power contact cannot be requested as a signal";
      if(use.powerMode=="dual-pin" && (f=="TO0" || f=="TO1") && power.id==signal.id) throw "Dual-pin power consumes both tool digital outputs";
      if(use.signalMode=="rs485" && (f=="AI2" || f=="AI3")) throw "Tool analog inputs are multiplexed with RS485";
      if(use.signalMode=="io" && StringTools.startsWith(f,"RS485")) throw "RS485 function requires communication mode";
    }
  }
  /** Missing supplier facts must not turn into default connector/cable dimensions. */
  public static function cableGeometryGaps(profile:WristProfile,cableId:String):Array<String> {
    var found:WristCableEvidence=null;
    for(c in profile.cables) if(c.id==cableId) found=c;
    if(found==null) throw "Unknown manufacturer cable candidate";
    var c:WristCableEvidence=found,p=port(profile,c.portId),gaps:Array<String> = [];
    if(c.exactPartNumber==null || c.exactPartNumber.length==0) gaps.push("exact cable/plug part number");
    if(c.drawingSource==null || c.drawingSource.length==0) gaps.push("dimensioned supplier drawing");
    if(c.geometrySha256==null || !~/^[0-9a-f]{64}$/.match(c.geometrySha256)) gaps.push("verified plug CAD/envelope hash");
    if(c.seatingOffsetMm==null || !Math.isFinite(c.seatingOffsetMm)) gaps.push("seating offset from observed insert face");
    if(c.keyClockingRad==null || !Math.isFinite(c.keyClockingRad)) gaps.push("mechanical key clocking");
    if(c.cableDiameterMm==null || !Math.isFinite(c.cableDiameterMm) || c.cableDiameterMm<=0) gaps.push("cable outer diameter");
    if(c.minBendRadiusMm==null || !Math.isFinite(c.minBendRadiusMm) || c.minBendRadiusMm<=0) gaps.push("cable bend radius");
    if(c.fixedLeadMm==null || !Math.isFinite(c.fixedLeadMm) || c.fixedLeadMm<0) gaps.push("straight/fixed lead requirement");
    if(p.geometryStatus!="qualified" || p.matingFrame==null) gaps.push("qualified socket mating frame");
    return gaps;
  }
  public static function requireCableGeometry(profile:WristProfile,cableId:String):Void {
    var gaps=cableGeometryGaps(profile,cableId);
    if(gaps.length>0) throw "Cable placement unavailable: "+gaps.join(", ");
  }
  public static function requireObservedGeometry(profile:WristProfile,portId:String):WristGeometryBinding {
    var binding=port(profile,portId).observedGeometry;
    if(binding==null) throw "No source CAD observation for this wrist port";
    return binding;
  }
  public static function requireMatingFrame(profile:WristProfile,portId:String):AssemblyFrame {
    var p=port(profile,portId);
    if(p.geometryStatus!="qualified" || p.matingFrame==null) throw "Wrist mating position/key orientation has not been qualified against this source revision";
    return p.matingFrame;
  }
}
