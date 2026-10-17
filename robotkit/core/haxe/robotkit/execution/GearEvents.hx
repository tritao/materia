package robotkit.execution;

/** Bounded process-event wire contract shared by the native owner and plan checker.
    All values in engage() must share a single plan time; only gear.engage changes the live relation.
    A declared channel's safe value MUST be release(). Device-owned queues reject this host feature.
*/
class GearEvents {
  public static function channel(follower:Int):String {
    if (follower < 0 || follower >= 64) throw "Invalid electronic gear follower";
    return 'motion.gear/$follower';
  }
  public static function release():ProcessEventValue return Process("gear.release", 0);
  public static function declaration(follower:Int):ProcessChannelDeclaration
    return new ProcessChannelDeclaration(channel(follower), release());
  public static function engage(follower:Int, terms:Array<GearTerm>, loopHz:Float):Array<ProcessEventValue> {
    if (terms == null || terms.length == 0 || terms.length > 8) throw "Electronic gear needs one to eight masters";
    if (!Math.isFinite(loopHz) || loopHz <= 0 || loopHz > 1000000) throw "Electronic gear needs an explicit supported loop rate";
    var masters:Array<Int> = [];
    var result:Array<ProcessEventValue> = [Process("gear.begin", 0), Process("gear.loop", loopHz)];
    for (term in terms) {
      if (term == null || term.master == follower || masters.indexOf(term.master) >= 0) throw "Invalid electronic gear masters";
      masters.push(term.master);
      result.push(Process('gear.term/${term.master}/${term.latencyNs}/${term.extrapolate ? 1 : 0}', term.ratio));
      result.push(Process("gear.quantum", term.quantum));
    }
    result.push(Process("gear.engage", 0));
    return result;
  }
}
