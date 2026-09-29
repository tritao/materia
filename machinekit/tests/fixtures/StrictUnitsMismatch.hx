import machinekit.units.Metres;
import machinekit.units.Millimetres;

class StrictUnitsMismatch {
  static function acceptsMetres(value:Metres):Void {}

  static function main():Void {
    acceptsMetres(new Millimetres(10));
  }
}
