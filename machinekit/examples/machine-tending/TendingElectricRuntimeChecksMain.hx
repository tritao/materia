/** Headless driver-contract check; reads the electric review's bound profiles. */
class TendingElectricRuntimeChecksMain {
  public static function main():Void {
    var file=Sys.args()[0],text=sys.io.File.getContent(file),report:Dynamic=haxe.Json.parse(text);
    var sources:Array<Dynamic> = cast report.sources;
    for(source in sources)if(haxe.crypto.Sha256.encode(sys.io.File.getContent(source.path))!=source.sha256)throw "Runtime review source changed";
    var profiles:Array<Dynamic> = cast report.runtimeProfiles;
    if(profiles==null || profiles.length!=28)throw "All 28 electric profiles are required";
    var supportProfiles=0;
    for(profile in profiles) {
      var outgoing:Array<Dynamic> = cast profile.outgoing;
      var incoming:Array<Dynamic> = cast profile.incoming;
      if(incoming[incoming.length-1].stockOwner=="tool") {
        var seating:Dynamic=profile.seating;
        if(seating==null)throw "Placement needs explicit support confirmation after release";
        var supports:Array<String> = cast seating.supportContacts;
        if(supports==null || supports.length==0)throw "Placement needs named support contacts after release";
      }
      if(StringTools.startsWith(Std.string(profile.caseReference),"vise-load/")) {
        if(profile.seating.datum!="supported-load" || profile.seating.requireSupportLoad!=true ||
          profile.clamping==null || profile.clamping.datum!="seat")
          throw "Vise loading and clamped datums must remain distinct";
        if(Reflect.hasField(report,"placementStrategy") && report.placementStrategy=="clearance-first and guarded locating") {
          if(profile.seating.orientationConstraint!="support-normal" || profile.clamping.orientationConstraint!="full-pose")
            throw "Vise support level and final jaw orientation must remain separate checks";
          if(profile.locating==null || profile.locating.limits.commandCorridorClear!=true || profile.locating.limits.returnCorridorClear!=true)
            throw "Vise locating needs checked insertion and return corridors";
          var samples:Array<Array<Float>> = cast profile.locating.samples;
          if(samples.length<2 || !TendingElectricRuntimeProfile.same(samples[0],incoming[incoming.length-1].to))
            throw "Vise locating must start at the insertion endpoint";
        }
      }
      if(report.placementStrategy=="clearance-first and guarded locating" && StringTools.startsWith(Std.string(profile.caseReference),"outfeedTray-")) {
        if(profile.placing==null || profile.seating.orientationConstraint!="full-pose")throw "Tray placement needs guarded support search and full final orientation";
        var samples:Array<Array<Float>> = cast profile.placing.samples,entry:Array<Dynamic> = cast profile.placing.incoming,exit:Array<Dynamic> = cast profile.placing.outgoing;
        var original:Dynamic=incoming[incoming.length-1],a:Array<Float> = cast original.from,b:Array<Float> = cast original.to,t:Float=profile.placing.limits.entryFraction;
        var subset=[for(j in 0...6)a[j]+t*(b[j]-a[j])];
        if(samples.length!=2 || !TendingElectricRuntimeProfile.same(samples[0],subset) || !TendingElectricRuntimeProfile.same(samples[1],b) ||
          !TendingElectricRuntimeProfile.same(entry[entry.length-1].to,samples[0]) || !TendingElectricRuntimeProfile.same(exit[0].from,samples[0]) ||
          !TendingElectricRuntimeProfile.same(exit[0].to,outgoing[0].to))throw "Support search/return left its reviewed segment subsets";
        supportProfiles++;
      }
      if(report.placementStrategy=="clearance-first and guarded locating" && StringTools.startsWith(Std.string(profile.caseReference),"vise-seat/")) {
        if(profile.registration==null || profile.registration.maximumPointMotionMm!=0.12 || profile.registration.hardwareQualified!=false ||
          profile.registration.extraction==null || profile.registration.extraction.numericalPenetrationLimitMm!=0.001)
          throw "Vise regrasp needs bounded observed registration, with hardware qualification withheld";
        var rejected=false;try TendingElectricRuntimeProfile.ordinaryWaypoints(profile) catch(_:Dynamic)rejected=true;
        if(!rejected)throw "Registered grasp was flattened into ordinary waypoint playback";
      }
      if(outgoing[0].stockOwner=="tool" &&
        (outgoing[0].kind!="guardedJointEdge" || outgoing[0].separating!=true))
        throw "Pickup must separate from its support under contact supervision";
    }
    var checks=TendingElectricRuntimeChecks.run(profiles);
    Reflect.setField(checks,"supportSearchProfiles",supportProfiles);
    Reflect.setField(checks,"locatingChecks",TendingContactSearchAction.checks());
    Reflect.setField(checks,"stockDatumChecks",TendingStockDatum.checks());
    Reflect.setField(checks,"seatingConstraintChecks",TendingSeatPolicy.checks());
    sys.io.File.saveContent(haxe.io.Path.directory(file)+"/electric-runtime-checks.json",haxe.Json.stringify({checks:checks,
      reviewSha256:haxe.crypto.Sha256.encode(text),hardwareReady:false,physicsPlayback:false},null,"  "));
    Sys.println(haxe.Json.stringify(checks));
  }
}
