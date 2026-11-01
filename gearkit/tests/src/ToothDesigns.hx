import gearkit.StrainWaveInput;

/** The four test designs of TOOTH_DESIGN_PLAN.md as inputs. Values come from the plan; D4's kappa
    is an assumed placeholder until TD5 measures where the derived circular spline interferes. */
class ToothDesigns {
  public static final DEG = Math.PI / 180;

  /** D1, the hobber reference: involute 20 deg, m 0.3, 200/202, cosine cam, kappa 1. */
  public static function d1():StrainWaveInput return new StrainWaveInput("D1", "involute", 0.3, 200, 202, 1.0, "cosine", 20 * DEG);

  /** D2, Liu 2025 defaults: double-circular-arc, m 0.5, 100/102, w0 0.5 (so kappa 1). */
  public static function d2():StrainWaveInput return new StrainWaveInput("D2", "double-circular-arc", 0.5, 100, 102, 0.5 / 0.5, "cosine");

  /** D3, involute 30 deg, m 0.3, 200/202; kappa swept 0.8 to 1.2. */
  public static function d3(kappa:Float = 1.0):StrainWaveInput return new StrainWaveInput(
    "D3", "involute", 0.3, 200, 202, kappa, "cosine", 30 * DEG);

  public static function d3Sweep():Array<StrainWaveInput> return [for (k in [0.8, 0.9, 1.0, 1.1, 1.2]) d3(k)];

  /** D4, deliberately bad: the D1 gear with kappa far above the 0.8..1.2 literature range (assumed 2.0). */
  public static function d4():StrainWaveInput return new StrainWaveInput("D4", "involute", 0.3, 200, 202, 2.0, "cosine", 20 * DEG);
}
