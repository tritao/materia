package machinekit.component;

/** Engineering intent; supplier offers and prices are separate sourcing data. */
@:wire enum Procurement {
  @:id(1) Make(process:String);
  @:id(2) Buy(manufacturer:Null<String>, mpn:Null<String>, standard:Null<String>);
  @:id(3) NotSupplied;
}

/** A numerical selection requirement, with values formatted in the stated unit. */
@:wire typedef ProcurementRequirement = {
  @:id(1) var name:String;
  @:id(2) var min:Null<String>;
  @:id(3) var max:Null<String>;
  @:id(4) var nominal:Null<String>;
  @:id(5) var unit:String;
}

typedef ProcurementData = {
  var kind:String;
  var process:Null<String>;
  var manufacturer:Null<String>;
  var mpn:Null<String>;
  var standard:Null<String>;
}

class ProcurementCodec {
  public static function document(value:Procurement):ProcurementData return switch value {
    case Make(process):
      {kind:"Make", process:required(process, "Manufacturing process"), manufacturer:null, mpn:null, standard:null};
    case Buy(manufacturer, mpn, standard):
      {kind:"Buy", process:null, manufacturer:optional(manufacturer, "Manufacturer"),
        mpn:optional(mpn, "Manufacturer part number"), standard:optional(standard, "Standard")};
    case NotSupplied:
      {kind:"NotSupplied", process:null, manufacturer:null, mpn:null, standard:null};
    case null:
      throw "Procurement kind is required";
  }

  public static function requirement(name:String, unit:String, ?min:Float, ?max:Float, ?nominal:Float):ProcurementRequirement {
    required(name, "Requirement name");
    required(unit, "Requirement unit");
    if (min == null && max == null && nominal == null) throw "Requirement needs a bound or nominal value";
    for (value in [min, max, nominal]) if (value != null) {
      if (!Math.isFinite(value)) throw "Requirement must be finite";
      if (value != 0 && Dimension.format(value) == "0")
        throw "Requirement loses precision; use a smaller unit";
    }
    if (min != null && max != null && min > max) throw "Requirement minimum exceeds maximum";
    if (nominal != null && (min != null && nominal < min || max != null && nominal > max))
      throw "Requirement nominal lies outside its bounds";
    return {name:name, unit:unit, min:min == null ? null : Dimension.format(min),
      max:max == null ? null : Dimension.format(max), nominal:nominal == null ? null : Dimension.format(nominal)};
  }

  public static function choice(name:String, value:String):ProcurementRequirement {
    required(name, "Requirement name");
    required(value, "Requirement value");
    return {name:name, unit:"1", min:null, max:null, nominal:value};
  }

  /** Preserve a declared port interface as categorical and numeric selection terms. */
  public static function port(name:String, iface:PortInterface):Array<ProcurementRequirement> return switch iface {
    case PushIn(diameter):
      [choice(name + "Kind", "PushIn"), requirement(name + "TubeDiameter", "mm", null, null, diameter)];
    case Thread(designation):
      [choice(name + "Kind", "Thread"), choice(name + "Thread", designation)];
    case Plug(designation, pins):
      [choice(name + "Kind", "Plug"), choice(name + "Connector", designation),
        requirement(name + "Pins", "1", null, null, pins)];
    case Coupling(key, channel):
      [choice(name + "Kind", "Coupling"), choice(name + "Coupling", key),
        requirement(name + "Channel", "1", null, null, channel)];
    case Unspecified:
      [choice(name + "Kind", "Unspecified")];
  }

  public static function copyRequirements(terms:Null<Array<ProcurementRequirement>>):Array<ProcurementRequirement> {
    if (terms == null) return [];
    for (term in terms) {
      if (term == null) throw "Requirement cannot be null";
      required(term.name, "Requirement name");
      required(term.unit, "Requirement unit");
      if (term.min == null && term.max == null && term.nominal == null)
        throw "Requirement needs a bound or nominal value";
      optional(term.min, "Requirement minimum");
      optional(term.max, "Requirement maximum");
      optional(term.nominal, "Requirement nominal");
    }
    return [for (term in terms) {name:term.name, min:term.min, max:term.max, nominal:term.nominal, unit:term.unit}];
  }

  static function required(value:String, label:String):String {
    if (value == null || StringTools.trim(value).length == 0) throw '$label is required';
    return value;
  }
  static function optional(value:Null<String>, label:String):Null<String> {
    if (value != null && StringTools.trim(value).length == 0) throw '$label cannot be blank';
    return value;
  }
}
