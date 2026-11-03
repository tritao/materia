package machinekit.motion;

import machinekit.catalog.PinnedCad;
import machinekit.catalog.SupplierLibraries;
import machinekit.component.SupplierCadComponent;

/**
 * The maker's STEP model of the eRob 110 actuator, pinned by SHA-256 and kept out of the repository (it is 41 MB).
 * Put it in the library with `machinekit/scripts/prepare-erob-cad`; `MATERIA_ZEROERR_ROOT` moves the library.
 * `ErobActuatorPart.geometry` is nominal and stands in whenever the file is absent. The STEP is one whole-actuator
 * solid set whose frame relative to the nominal CAD frame has not been reviewed, so it is for visual checks only.
 */
class ErobActuatorCad {
  public static function path():String return SupplierLibraries.asset(SupplierLibraries.zeroerr(), ErobActuatorData.STEP_SHA256);

  public static function available():Bool return sys.FileSystem.exists(path());

  /** The pinned CAD, or null when the file has not been prepared. A file that differs from the pin throws. The caller closes it. */
  public static function load():Null<SupplierCadComponent> {
    if (!available()) return null;
    PinnedCad.require(path(), ErobActuatorData.STEP_SHA256, "machinekit/scripts/prepare-erob-cad");
    return new SupplierCadComponent(
      path(),
      ErobActuatorData.STEP_SHA256,
      "EROB110I-V6D-SUPPLIER-CAD",
      "ZeroErr eRob 110 V6 rev D supplier CAD (visual reference)"
    );
  }
}
