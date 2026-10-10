package robotkit.manipulation;

/** Two bodies closer than they may be. `distance` is zero when they touch or overlap. */
typedef ClearanceViolation = {
  var a:String;
  var b:String;
  var distance:Float;
  var required:Float;
}
