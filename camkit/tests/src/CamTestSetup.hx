import camkit.CamSetup;

/** Broad stock for geometry round-trip fixtures; focused tests use real bounds. */
class CamTestSetup {
  public static function standard():CamSetup
    return new CamSetup(-1.0, 1.0, -1.0, 1.0, 0.0, -1.0, 0.005);
}
