package app.editor;

import gearkit.GearMeasurement;
import gearkit.GearTrace;

/** Presentation snapshot; numerical inspection and its qualification stay in GearKit. */
class HobbingReportData {
  public var status = "Cutting";
  public var complete = false;
  public var failure:Null<String>;
  public var inspectionFailure:Null<String>;
  public var seconds = 0.0;
  public var syncArcsec = 0.0;
  public var contacts = 0;
  public var windows = 0;
  public var poses = 0;
  public var buffered = 0;
  public var stockKiB = 0;
  public var cutting = 0.0;
  public var radial = 0.0;
  public var power = 0.0;
  public var hobTorque = 0.0;
  public var tableTorque = 0.0;
  public var materialBasis = "";
  public var measurement:Null<GearMeasurement>;

  public function new() {}

  public function quality():String {
    if (measurement == null) return complete ? "Inspection incomplete" : "Inspection pending";
    var grade = measurement.overallGrade;
    return (grade == null ? "Grade estimate incomplete or worse than 11" : 'Candidate grade $grade') +
      (measurement.isoClass == null ? " · ISO class not qualified" : ' · ISO class ${measurement.isoClass}');
  }

  public function warnings():Map<String,Array<String>> {
    var groups:Map<String,Array<String>> = ["Sampling" => [], "Coverage" => [], "Assessment" => []];
    if (measurement == null) return groups;
    for (warning in measurement.warnings.concat(measurement.extrapolation)) {
      var text = warning.toLowerCase();
      var group = text.indexOf("sampl") >= 0 || text.indexOf("spacing") >= 0 || text.indexOf("cutoff") >= 0 ? "Sampling" :
        text.indexOf("unchecked") >= 0 || text.indexOf("coverage") >= 0 || text.indexOf("covered") >= 0 || text.indexOf("profile control") >= 0 ? "Coverage" : "Assessment";
      // Missing face endpoints are a coverage issue as well as missing samples.
      if (text.indexOf("endpoints") >= 0) group = "Coverage";
      groups.get(group).push(warning);
    }
    return groups;
  }

  public static function number(value:Float):String return Std.string(Math.round(value * 1000) / 1000);

  /** Lossless sample coverage: every tooth, both trace families, raw and filtered deviations. */
  public function csv():String {
    var out = new StringBuf();
    out.add("trace,tooth,flank,position_mm,raw_um,filtered_um\n");
    var measured = measurement;
    if (measured != null) {
      appendCsv(out,"profile",measured.profiles);
      appendCsv(out,"lead",measured.helices);
    }
    return out.toString();
  }

  static function appendCsv(out:StringBuf, kind:String, traces:Array<GearTrace>):Void {
    for (trace in traces) for (i in 0...trace.x.length)
      out.add('$kind,${trace.tooth},${trace.flank},${trace.x[i]},${trace.raw[i]},${trace.filtered[i]}\n');
  }
}
