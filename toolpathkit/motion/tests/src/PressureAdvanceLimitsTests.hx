import motionkit.robot.PressureAdvanceLimits;

/** Exercises the derivative bounds against an independently evaluated smooth rest-to-rest feed. */
class PressureAdvanceLimitsTests {
  public static function main():Void {
    var checks = 0;
    for (window in [0.002, 0.04, 0.2]) for (lead in [0.0, 0.001, 0.04, 1.0])
      for (ratio in [0.1, 0.5, 1.0, 1.5, 2.0, 10.0]) {
        var duration = window * ratio;
        var driveV = 2.0, driveA = 70.0;
        var bounds = PressureAdvanceLimits.planning(driveV, driveA, lead, window);
        var peak = Math.min(bounds.velocity, bounds.acceleration * duration / Math.PI);
        function state(t:Float):{p:Float, v:Float, a:Float} {
          if (t <= 0) return {p:0, v:0, a:0};
          if (t >= duration) return {p:peak * duration / 2, v:0, a:0};
          var phase = 2 * Math.PI * t / duration;
          return {p:peak * (t / 2 - duration * Math.sin(phase) / (4 * Math.PI)),
            v:peak * (1 - Math.cos(phase)) / 2, a:peak * Math.PI * Math.sin(phase) / duration};
        }
        function ramp(x:Float):{v:Float, d:Float, dd:Float} {
          if (x >= 1) return {v:1, d:0, dd:0};
          return {v:x*x*x*(10+x*(-15+6*x)), d:30*x*x*(1-x)*(1-x)/window,
            dd:60*x*(1-x)*(1-2*x)/(window*window)};
        }
        for (i in 0...1001) {
          var t = duration * i / 1000;
          var q = state(t), left = state(t-window/2), right = state(t+window/2);
          var u = ramp(t/window), v = ramp((duration-t)/window);
          var taper = u.v*v.v, first = u.d*v.v-u.v*v.d;
          var second = u.dd*v.v-2*u.d*v.d+u.v*v.dd;
          var mean = (right.p-left.p)/window, rate = (right.v-left.v)/window;
          var slope = (right.a-left.a)/window;
          var speed = q.v+lead*(rate*taper+mean*first);
          var acceleration = q.a+lead*(slope*taper+2*rate*first+mean*second);
          if (Math.abs(speed)>driveV+1e-10 || Math.abs(acceleration)>driveA+1e-10)
            throw 'Advance exceeds drive at lead=$lead, window=$window, duration=$duration, t=$t';
          checks++;
        }
      }
    var off = PressureAdvanceLimits.planning(2, 70, 0, 0.04);
    if (off.velocity != 2 || off.acceleration != 70) throw "Disabled advance reserves capacity";
    var rounded = PressureAdvanceLimits.planning(2, 70, 0.04, 1.5e-9);
    var tick = PressureAdvanceLimits.planning(2, 70, 0.04, 2e-9);
    if (rounded.velocity != tick.velocity || rounded.acceleration != tick.acceleration || rounded.jerk != tick.jerk)
      throw "Planning and native half-window rounding disagree";
    var base = PressureAdvanceLimits.planning(2, 70, 0.04, 0.04);
    var scaled = PressureAdvanceLimits.planning(20, 700, 0.04, 0.04);
    if (Math.abs(scaled.velocity/base.velocity-10)>1e-12 || Math.abs(scaled.acceleration/base.acceleration-10)>1e-12)
      throw "Drive unit scaling changes the reserve";
    for (bad in [0.0, -1.0, Math.NaN, Math.POSITIVE_INFINITY]) {
      var refused = false;
      try PressureAdvanceLimits.planning(2, 70, 0.04, bad) catch (_:Dynamic) refused = true;
      if (!refused) throw 'Invalid window accepted: $bad';
      refused = false;
      try PressureAdvanceLimits.planning(bad, 70, 0.04, 0.04) catch (_:Dynamic) refused = true;
      if (!refused) throw 'Invalid drive velocity accepted: $bad';
      refused = false;
      try PressureAdvanceLimits.planning(2, bad, 0.04, 0.04) catch (_:Dynamic) refused = true;
      if (!refused) throw 'Invalid drive acceleration accepted: $bad';
    }
    for (bad in [-1.0, Math.NaN, Math.POSITIVE_INFINITY]) {
      var refused = false;
      try PressureAdvanceLimits.planning(2, 70, bad, 0.04) catch (_:Dynamic) refused = true;
      if (!refused) throw 'Invalid lead accepted: $bad';
    }
    Sys.println('Pressure advance bounds passed ($checks derivative checks)');
  }
}
