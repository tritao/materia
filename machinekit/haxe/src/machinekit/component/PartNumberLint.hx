package machinekit.component;

/** Report violations without changing a component's identity. */
class PartNumberLint {
  public static inline final MAX_LENGTH = 48;

  public static function problems(number:String):Array<String> {
    var result:Array<String> = [];
    if (number == null || number.length == 0) return ["empty part number"];
    if (number.length > MAX_LENGTH) result.push("longer than " + MAX_LENGTH + " characters");
    if (!~/^[A-Z0-9][A-Z0-9.\-x\/]*$/.match(number)) result.push("invalid characters");
    if (number.toLowerCase().indexOf("null") >= 0) result.push("contains null");
    // Dimension.format rounds to at most three fractional places. This check
    // detects long decimal renderings bounded as a dimension. Dotted catalogue
    // identifiers such as 10.08.02.00203 are not decimal dimensions.
    if (~/(^|[-x\/A-Z])[0-9]+\.[0-9]{4,}($|[-x\/A-Z])/.match(number)) result.push("unformatted decimal");
    return result;
  }
}
