package machinekit.hobbing;

import machinekit.structural.GroundStock;

/** X carriage mounting acceptance. Stock data is assumed, not a supplier quote.
 * HIWIN HG25 ZA S1 is 85 um per 500 mm rail spacing; apply the stricter
 * project 10 um coplanarity target to all four block pads as well. */
class HobberCarriageStockCheck {
  public static inline final PRELOAD = "ZA (purchase requirement; assumed until ordered)";
  public static inline final SOURCE = "HIWIN Linear Guideway catalogue, HG mounting surface height S1, HG25 ZA 85 um at 500 mm; https://www.hiwin.info/upload/iblock/030/02a1gowfidptr2a0s23w0i3p2w2gukg5/1bd14b4f_ca32_11e8_80c8_6c3be5b9a4d0_f44bc6b3_037b_11ed_812b_6c3be5b9a4d0.pdf";
  public static inline final ACCURACY_CLASS = "C";
  /** HIWIN G99TE22-2008 table 2-1-3, HG25 normal-class matched-set height variation. */
  public static inline final BLOCK_HEIGHT_VARIATION_MM = 0.020;
  public static inline final PAD_COPLANARITY_MM = 0.01;
  public static function result():{stockFlatnessMm:Float, hiwinMountHeightMm:Float,
    blockHeightVariationMm:Float, requiredMm:Float, finishingRequired:Bool, assumedStock:Bool} {
    var stock = GroundStock.plate(GroundStock.STEEL, HobberSaddle.X_CARRIAGE_THICKNESS);
    var spacing = 2 * HobberLayout.get().bedRailCentre;
    var allowance = 0.085 * spacing / 500;
    var flatness = stock.flatness * spacing / 100;
    var required = Math.min(allowance, Math.min(BLOCK_HEIGHT_VARIATION_MM, PAD_COPLANARITY_MM));
    return {stockFlatnessMm: flatness, hiwinMountHeightMm: allowance, blockHeightVariationMm: BLOCK_HEIGHT_VARIATION_MM, requiredMm: required,
      finishingRequired: flatness > required, assumedStock: stock.assumed};
  }
  public static function report():String {
    var r = result();
    return 'X plate stock flatness ${r.stockFlatnessMm} mm (ASSUMED stock) versus HIWIN mounting S1 ${r.hiwinMountHeightMm} mm '
      + 'and class $ACCURACY_CLASS block height variation ${r.blockHeightVariationMm} mm; all-pad target ${r.requiredMm} mm; preload $PRELOAD; '
      + (r.finishingRequired ? "stock acceptance OPEN: inspect/select stock or supplier finish-grind four lands in one setup; no small replicated pads" : "stock meets requirement")
      + "; " + SOURCE;
  }
}
