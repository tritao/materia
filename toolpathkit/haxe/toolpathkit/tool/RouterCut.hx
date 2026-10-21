package toolpathkit.tool;

/** Explicit planning inputs: chip loads in mm/tooth; energy in J/mm3.
    Values are assumptions until qualified for the actual bit/material lot. */
class RouterCut {
 public static function evaluate(operation:String, bit:RouterBit, rpm:Float, chipMm:Float,
  minChipMm:Float,maxChipMm:Float,energyJPerMm3:Float,depthMm:Float,widthMm:Float,
  maxFeedMPerMinute:Float,maxRpm:Float):RouterCutResult {
  if(operation.length==0)throw "Cut needs an operation name";
  for(v in [rpm,chipMm,minChipMm,maxChipMm,energyJPerMm3,depthMm,widthMm,maxFeedMPerMinute,maxRpm])
   if(!Math.isFinite(v)||v<=0)throw operation+": invalid cutting input";
  var flute:Float=cast bit.tool.fluteLength;
  if(minChipMm>maxChipMm||depthMm>flute*1000||widthMm>bit.tool.diameter*1000)
   throw operation+": invalid chip range or engagement exceeds cutter";
  var speed=Math.min(rpm,maxRpm),requested=rpm*bit.flutes*chipMm/1000;
  var feed=Math.min(speed*bit.flutes*chipMm/1000,maxFeedMPerMinute);
  var actualChip=feed*1000/(speed*bit.flutes);
  var power=energyJPerMm3*depthMm*widthMm*feed*1000/60;
  var omega=speed*2*Math.PI/60,torque=power/omega;
  // Mean tangential cutting force; 2x envelope covers intermittent tooth loading.
  var mean=power/(omega*bit.tool.diameter/2);
  return {operation:operation,rpm:speed,requestedFeedMPerMinute:requested,feedMPerMinute:feed,
   chipMm:actualChip,meltingRisk:actualChip<minChipMm,aboveChipRange:actualChip>maxChipMm,
   powerW:power,torqueNm:torque,meanForceN:mean,forceEnvelopeN:2*mean,assumed:true};
 }
}
typedef RouterCutResult = {
 var operation:String;var rpm:Float;var requestedFeedMPerMinute:Float;var feedMPerMinute:Float;
 var chipMm:Float;var meltingRisk:Bool;var aboveChipRange:Bool;
 var powerW:Float;var torqueNm:Float;var meanForceN:Float;var forceEnvelopeN:Float;var assumed:Bool;
}
