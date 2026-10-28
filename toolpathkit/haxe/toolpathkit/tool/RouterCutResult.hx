package toolpathkit.tool;

typedef RouterCutResult = {
 var operation:String;var rpm:Float;var requestedFeedMPerMinute:Float;var feedMPerMinute:Float;
 var chipMm:Float;var meltingRisk:Bool;var aboveChipRange:Bool;
 var powerW:Float;var torqueNm:Float;var meanForceN:Float;var forceEnvelopeN:Float;var assumed:Bool;
}
