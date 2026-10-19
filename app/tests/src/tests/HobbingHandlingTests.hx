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
    var front=new HobbingHandling(250,35);
    for(i in 0...161){front.load(i*0.025);
      if(front.frontMm>0&&(front.liftMm!=35||front.clamp!=0))throw "front transfer must clear mandrel before moving horizontally";
      if(front.clamp>0&&(front.frontMm!=0||front.liftMm!=0))throw "front clamp requires seating";
    }
    if(front.frontMm!=0||front.liftMm!=0||front.clamp!=1)throw "front loading incomplete";
    front.cutting();front.retracting();
    for(i in 0...161){front.unload(i*0.025);
      if(front.frontMm>0&&(front.liftMm!=35||front.clamp!=0))throw "front unload must release and lift before transferring";
    }
    if(front.frontMm!=250||front.liftMm!=35||front.clamp!=0)throw "front unloading incomplete";
    front.inspect("clamped");if(front.displayedFront()!=0||front.displayedLift()!=0)throw "front clamped inspection";
    front.inspect("finished");if(front.displayedFront()!=250||front.displayedLift()!=35)throw "front finished inspection";
    front.inspect("sequence");front.load(0);if(front.frontMm!=250||front.liftMm!=35||front.clamp!=0)throw "front reset";
    Sys.println("Hobbing handling timeline and inspection checks passed");
  }
}
