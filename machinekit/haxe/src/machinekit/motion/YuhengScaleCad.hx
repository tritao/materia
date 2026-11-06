package machinekit.motion;

import machinekit.catalog.PinnedCad;
import machinekit.catalog.SupplierLibraries;
import machinekit.component.SupplierCadComponent;

/**
 * The maker's STEP models of the Yuheng JFT-40 scale (housing and reading head in one file, ML 70, 120 and 220), pinned by SHA-256 and kept out of the
 * repository (11 MB each). Put them in the library with `machinekit/scripts/prepare-yuheng-cad` (it checks the digests); `MATERIA_YUHENG_ROOT` moves the
 * library. Unlike the eRob's hash-addressed asset, these keep the names the maker delivered, so a different file under the same name throws.
 * `SealedScalePart.geometry` is the recipe's own and is checked against these files by `SealedScaleStepTests`; the STEP is for comparison and visual checks.
 * The STEP frame: travel along +Z with the housing's start end at z 158.86, the housing's flush face at x 183.19 (its other face at 164.94 on the end caps
 * and 165.19 on the extrusion), the fixing-hole axis at y 236.78 and the reading face at y 255.48.
 */
class YuhengScaleCad {
  /** The files as delivered 2026-11-06 and their SHA-256 (lower-case hex). */
  public static inline final STEP_70 = "JFT-40B-70.STEP";
  public static inline final STEP_70_SHA256 = "d94459a392ed6cc5a9756dd231a0232d9452f43c7d8c4bc1102c785ed6890978";
  public static inline final STEP_120 = "JFT-40B-120.STEP";
  public static inline final STEP_120_SHA256 = "1875bf25774f66912e93fd54e9be4c9f3df26611af477bdd128d30501c9b0e7f";
  public static inline final STEP_220 = "JFT-40B-220.STEP";
  public static inline final STEP_220_SHA256 = "0966f2f4dd38fc23859941398fa915fcaf6a017e4876bfd8f567deb6ef656d30";

  /** The measuring lengths the maker sent, ascending. */
  public static function lengths():Array<Float> return [70, 120, 220];

  static function entry(ml:Float):{file:String, sha256:String} {
    if (ml == 70) return {file: STEP_70, sha256: STEP_70_SHA256};
    if (ml == 120) return {file: STEP_120, sha256: STEP_120_SHA256};
    if (ml == 220) return {file: STEP_220, sha256: STEP_220_SHA256};
    throw 'No Yuheng JFT-40 STEP for ML $ml; sent: 70, 120, 220';
  }

  public static function sha256(ml:Float):String return entry(ml).sha256;
  public static function path(ml:Float):String return SupplierLibraries.yuheng() + "/" + entry(ml).file;
  public static function available(ml:Float):Bool return sys.FileSystem.exists(path(ml));

  /** The pinned CAD of the JFT-40 with measuring length `ml`, or null when the file has not been prepared. A file that differs from the pin throws. The caller closes it. */
  public static function load(ml:Float):Null<SupplierCadComponent> {
    if (!available(ml)) return null;
    PinnedCad.require(path(ml), sha256(ml), "machinekit/scripts/prepare-yuheng-cad");
    return new SupplierCadComponent(path(ml), sha256(ml), 'JFT-40B-$ml-SUPPLIER-CAD', 'Yuheng JFT-40B ML $ml supplier CAD (housing and head, comparison reference)');
  }
}
