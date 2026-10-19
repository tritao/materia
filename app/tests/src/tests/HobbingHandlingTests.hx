package tests;
import app.HobbingHandling;
class HobbingHandlingTests {
  public static function run():Void {
    var h=new HobbingHandling(),previous=120.0;
    for(i in 0...81){var t=i*0.05;h.load(t);
      if(h.liftMm>previous+1e-9||h.liftMm<0||h.clamp<0||h.clamp>1)throw "invalid loading trajectory";
      if(h.clamp>0&&h.liftMm!=0)throw "clamping before seating";
      previous=h.liftMm;
    }
    if(h.liftMm!=0||h.clamp!=1)throw "loading ends seated and clamped";
    h.cutting();h.retracting();if(h.liftMm!=0||h.clamp!=1)throw "withdrawal must retain workholding";
    previous=0;
    for(i in 0...81){h.unload(i*0.05);
      if(h.liftMm<previous-1e-9||h.liftMm>120)throw "invalid unloading trajectory";
      if(h.liftMm>0&&h.clamp!=0)throw "lifting before release";previous=h.liftMm;
    }
    if(h.liftMm!=120||h.clamp!=0)throw "unloading ends lifted and released";
    h.inspect("clamped");if(h.displayedLift()!=0||h.displayedClamp()!=1)throw "clamped inspection";
    h.inspect("blank");if(h.displayedLift()!=120||h.displayedClamp()!=0)throw "blank inspection";
    h.inspect("finished");if(h.displayedLift()!=120||h.displayedClamp()!=0)throw "finished inspection";
    h.inspect("sequence");if(h.liftMm!=120||h.clamp!=0)throw "inspection mutated handling timeline";
    Sys.println("Hobbing handling timeline and inspection checks passed");
  }
}
