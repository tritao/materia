package machinekit.standard;

/** ISO 8734 type A dowel pin (hardened, m6), nominal diameter in millimetres. */
typedef DowelPinSpec = {
  /** The nominal diameter as the size string ("5" for a 5 mm pin). */
  var size:String;
  var diameter:Float;
  /** The end chamfer, mm (assumed: about an eighth of the diameter; the standard's `c` is a maximum). */
  var chamfer:Float;
}
