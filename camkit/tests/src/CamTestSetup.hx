import toolpathkit.setup.Setup;
import toolpathkit.setup.SetupStock;
import toolpathkit.path.Point3;

/** Broad stock for geometry round-trip fixtures; focused tests use real bounds. */
class CamTestSetup {
  public static function standard():Setup
    return new Setup("1", new Point3(0, 0, 0),
      new SetupStock(-1.0, 1.0, -1.0, 1.0, 0.0, -1.0, 0.005));
}
