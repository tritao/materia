import toolpathkit.path.BeadSection;

/** Independent geometric quadrature and conservation checks for the common section model. */
class BeadSectionTests {
  static var assertions = 0;
  static function near(a:Float,b:Float,relative:Float,message:String):Void {
    assertions++;
    if (!Math.isFinite(a) || !Math.isFinite(b) || Math.abs(a-b)>relative*Math.abs(b)) throw message;
  }
  // Integrate the actual silhouette, not its closed-form area expression.
  static function integrate(width:Float,height:Float):Float {
    var steps = 20000, dx = width/steps, radius = height/2, halfFlat = (width-height)/2;
    var sum = 0.0;
    for (i in 0...steps) {
      var x = Math.abs(-width/2+(i+0.5)*dx);
      sum += x<=halfFlat ? height : 2*Math.sqrt(Math.max(0,radius*radius-(x-halfFlat)*(x-halfFlat)));
    }
    return sum*dx;
  }
  public static function main():Void { run(); }
  public static function run():Int {
    assertions = 0;
    for (height in [0.0001,0.0002,0.0005]) for (ratio in [1.0,1.5,3.0]) {
      var width = ratio*height, area = BeadSection.area(width,height);
      near(area,integrate(width,height),3e-6,"Rounded bead area differs from its silhouette");
      near(BeadSection.width(area,height),width,2e-15,"Area-to-width inverse changed geometry");
      near(height*BeadSection.spacing(width,height),area,2e-15,"Neighbouring beads do not conserve layer volume");
      for (diameter in [0.00175,0.00285]) {
        var filament = BeadSection.filamentArea(diameter);
        near(filament,integrate(diameter,diameter),3e-6,"Filament section differs from a circular silhouette");
        var pathLength = 0.123, filamentLength = area*pathLength/filament;
        near(filament*filamentLength,area*pathLength,2e-15,"Filament-to-bead conversion loses volume");
      }
    }
    for (area in [1e-10,1e-9,1e-8]) {
      var height = BeadSection.heightForArea(area,0.0005);
      near(BeadSection.width(area,height),height,2e-15,"Small flows do not reduce to circular beads");
      near(BeadSection.area(height,height),area,2e-15,"Reduced bead height loses supplied area");
    }
    Sys.println('Bead section checks passed ($assertions assertions)');
    return assertions;
  }
}
