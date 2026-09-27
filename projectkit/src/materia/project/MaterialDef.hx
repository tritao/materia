package materia.project;

/** Visual and bulk physical properties in SI units. */
typedef MaterialDef = {
  var id:String;
  var name:String;
  var visual:MaterialVisual;
  var physical:MaterialPhysical;
}

typedef MaterialVisual = {
  var baseColor:Array<Float>;
  var metallic:Float;
  var roughness:Float;
  @:optional var emissive:Array<Float>;
  @:optional var alpha:Float;
}

typedef MaterialPhysical = {
  var density:Float;
  var spec:String;
}
