class MobileWelderStationTests {
  static function main():Void {
    var cell = new MobileWelderCell();
    var scene = MobileWelderPreview.design(cell);
    var steps = new MobileWeldStations(cell, scene).mission();
    var stations = 0, registrations = 0;
    var seams:Map<String, Bool> = new Map();
    for (step in steps) {
      if (step.kind == "goTo") stations++;
      if (step.kind == "findWork") {
        registrations++;
        if (step.contactWork == null) throw "Every mobile station needs its CAD contact-registration job";
      }
      if (step.kind == "weld") {
        if (step.weld == null) throw "A mobile weld step has no CAD path";
        for (segment in step.weld.path) {
          if (seams.exists(segment.seam)) throw 'Mobile station plan repeats seam ${segment.seam}';
          seams.set(segment.seam, true);
        }
      }
    }
    var seamCount = 0;
    for (_ in seams.keys()) seamCount++;
    if (stations == 0 || registrations != stations || seamCount != 10)
      throw 'Mobile station plan has $stations stations, $registrations registrations and $seamCount seams';
    Sys.println('Mobile station plan: $stations stations, $seamCount unique CAD seams and checked registration returns');
  }
}
