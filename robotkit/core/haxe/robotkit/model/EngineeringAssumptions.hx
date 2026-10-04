package robotkit.model;

/** An assumption names the quantity it affects, so findings report only relevant inputs. */
typedef QuantityAssumption = {
  var quantity:String;
  var label:String;
}

class EngineeringAssumptions {
  public static function add(values:Array<QuantityAssumption>, quantity:String, label:String):Void {
    for (value in values) if (value.quantity == quantity && value.label == label) return;
    values.push({quantity: quantity, label: label});
  }

  public static function merge(target:Array<QuantityAssumption>, source:Array<QuantityAssumption>):Void
    for (value in source) add(target, value.quantity, value.label);

  public static function labels(values:Array<QuantityAssumption>, quantities:Array<String>):Array<String> {
    var result:Array<String> = [];
    for (value in values) if (quantities.indexOf(value.quantity) >= 0 && result.indexOf(value.label) < 0)
      result.push(value.label);
    result.sort(Reflect.compare);
    return result;
  }
}
