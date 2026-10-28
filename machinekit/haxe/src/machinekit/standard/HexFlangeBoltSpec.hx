package machinekit.standard;

/** ISO 4162 small-series, non-serrated flange bolt. Dimensions are mm.
 * Bearing diameters delimit the guaranteed flat annulus (Type F); they are
 * distinct from the maximum flange envelope used for collision/occupancy. */
typedef HexFlangeBoltSpec = {
  var size:String;
  var diameter:Float;
  var pitch:Float;
  var acrossFlats:Float;
  var headHeight:Float;
  var flangeDiameter:Float;
  var flangeThickness:Float;
  var bearingInnerDiameter:Float;
  var bearingOuterDiameter:Float;
  var threadLength:Float;
}
