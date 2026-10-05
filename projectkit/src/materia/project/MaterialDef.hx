package materia.project;

/** Visual and bulk properties; density is kg/m3, mechanical properties are MPa. */
@:wire typedef MaterialDef = {
  @:id(1) var id:String;
  @:id(2) var name:String;
  @:id(3) var visual:MaterialVisual;
  @:id(4) var physical:MaterialPhysical;
  @:id(5) @:optional var mechanical:Null<MaterialMechanical>;
  @:id(6) @:optional var sheet:Null<MaterialSheet>;
}

@:wire typedef MaterialVisual = {
  @:id(1) var baseColor:Array<Float>;
  @:id(2) var metallic:Float;
  @:id(3) var roughness:Float;
  @:id(4) @:optional var emissive:Null<Array<Float>>;
  @:id(5) @:optional var alpha:Null<Float>;
}

@:wire typedef MaterialPhysical = {
  @:id(1) var density:Float;
  @:id(2) var spec:String;
}

/** Strengths and elastic modulus in MPa. Optional for nonstructural materials. */
@:wire typedef MaterialMechanical = {
  @:id(1) final uts:Float;
  @:id(2) final yield:Float;
  @:id(3) final youngs:Float;
}

/** Sheet development defaults; grain indicates that the authored rolling direction matters. */
@:wire typedef MaterialSheet = {
  @:id(1) var kFactorTable:Array<MaterialKFactorRow>;
  @:id(2) var defaultRadiusRatio:Float;
  @:id(3) var minRadiusRatio:Float;
  @:id(4) var grain:Bool;
}

/** ANSI K factor at one nonnegative inner-radius/thickness ratio. */
@:wire typedef MaterialKFactorRow = {
  @:id(1) var ratio:Float;
  @:id(2) var k:Float;
}
