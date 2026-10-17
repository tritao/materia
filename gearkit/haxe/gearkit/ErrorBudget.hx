package gearkit;

typedef BudgetSource={id:String,part:String,basis:String};
typedef BudgetContribution={source:BudgetSource,measurement:GearMeasurement,delta:Map<String,Float>};

/** Runs the SAME supplied job with no errors, each source, and all sources.
    Signed metric deltas are diagnostic, not additive independent random variances.
    Superposition is checked on measured traces, not sums of peak-to-peak metrics. */
class ErrorBudget {
  public final baseline:GearMeasurement;
  public final combined:GearMeasurement;
  public final contributions:Array<BudgetContribution> = [];
  public final dominant:Map<String,String> = [];
  public final interaction:Map<String,Float> = [];
  public final warnings:Array<String> = [];
  public var additive(default,null):Bool=true;
  public final relativeTolerance:Float;
  public final absoluteTolerance:Float;
  public function new(sources:Array<BudgetSource>,run:Array<String>->GearMeasurement,
      relativeTolerance:Float=0.25,absoluteTolerance:Float=0.1) {
    if(!Math.isFinite(relativeTolerance)||relativeTolerance<0||!Math.isFinite(absoluteTolerance)||absoluteTolerance<0)
      throw "invalid error-budget tolerance";
    this.relativeTolerance=relativeTolerance;this.absoluteTolerance=absoluteTolerance;
    var ids:Array<String> = [];
    for(source in sources){if(source.id==null||source.id.length==0||ids.indexOf(source.id)>=0)throw "duplicate/empty budget source";ids.push(source.id);}
    baseline=run([]);
    for(source in sources){var measured=run([source.id]),delta:Map<String,Float> = [];
      for(key in baseline.values.keys()){
        if(!measured.values.exists(key))throw 'source ${source.id} lost measurement $key';
        delta.set(key,measured.value(key)-baseline.value(key));
      }
      contributions.push({source:source,measurement:measured,delta:delta});
    }
    combined=run(ids);
    for(key in baseline.values.keys()){
      if(!combined.values.exists(key))throw 'combined budget lost measurement $key';
      var largest=1e-9,id="none";
      for(row in contributions){var value=row.delta.get(key);if(value!=null&&Math.abs(value)>largest){largest=Math.abs(value);id=row.source.id;}}
      dominant.set(key,id);
    }
    for(channel in ["profile","lead","pitch","runout"]){
      var b=trace(baseline,channel),all=trace(combined,channel),individual=[for(row in contributions)trace(row.measurement,channel)];
      if(all.length!=b.length||b.length==0)throw 'incomplete budget channel $channel';
      for(values in individual)if(values.length!=b.length)throw 'inconsistent budget sampling $channel';
      var residual=0.0,scale=0.0;
      for(i in 0...b.length){var sum=0.0,envelope=0.0;
        for(values in individual){sum+=values[i]-b[i];envelope+=Math.abs(values[i]-b[i]);}
        residual=Math.max(residual,Math.abs(all[i]-b[i]-sum));scale=Math.max(scale,envelope);
      }
      interaction.set(channel,residual);
      if(residual>absoluteTolerance+relativeTolerance*scale){additive=false;warnings.push('$channel interaction exceeds stated trace-superposition tolerance');}
    }
    for(measured in [baseline,combined])for(warning in measured.warnings.concat(measured.extrapolation))
      if(warnings.indexOf(warning)<0)warnings.push(warning);
  }
  static function trace(m:GearMeasurement,channel:String):Array<Float> {
    var out:Array<Float> = [];
    switch channel {
      case "profile":for(t in m.profiles)for(v in t.filtered)out.push(v);
      case "lead":for(t in m.helices)for(v in t.filtered)out.push(v);
      case "pitch":for(flank in m.cumulativePitch)for(v in flank)out.push(v);
      case "runout":for(v in m.radialMeasurements)out.push(v*1000);
      case _:throw "unknown budget channel";
    }
    for(v in out)if(!Math.isFinite(v))throw "non-finite budget trace";
    return out;
  }
  public function report():String {
    var lines=['Error budget: trace superposition ${additive?"within tolerance":"outside tolerance"}; '
      +'${absoluteTolerance} um + ${relativeTolerance*100}% of summed trace envelope'];
    var keys=[for(key in combined.values.keys())key];keys.sort(Reflect.compare);
    for(key in keys){var id=dominant.get(key),part="none";for(row in contributions)if(row.source.id==id)part=row.source.part;
      lines.push('$key: baseline ${baseline.value(key)} um; all ${combined.value(key)} um; dominant $part ($id)');
      for(row in contributions)lines.push('  ${row.source.id}: delta ${row.delta.get(key)} um; ${row.source.part}; ${row.source.basis}');
    }
    return lines.concat(warnings).join("\n");
  }
}
