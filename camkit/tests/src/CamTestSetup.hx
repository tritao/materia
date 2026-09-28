import toolpathkit.setup.Setup;

/** Broad stock for geometry round-trip fixtures; focused tests use real bounds. */
class CamTestSetup {
  public static function standard():Setup
    return new Setup(-1.0, 1.0, -1.0, 1.0, 0.0, -1.0, 0.005);
}
