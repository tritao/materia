package machinekit.enclosure;

/** Initial planning constructions. Thicknesses/densities are assumed; qualification is unfilled. */
class PanelCatalogue {
  public static function all():Array<LaminatedPanel> return [
    new LaminatedPanel("compact-laminate-13", [{materialId:"compact-laminate", thicknessMm:13}], RouterCut, "supported and gasketed removable joints"),
    new LaminatedPanel("compact-laminate-6", [{materialId:"compact-laminate", thicknessMm:6}], RouterCut, "corner profile"),
    new LaminatedPanel("fr-acm-4", [{materialId:"fr-acm", thicknessMm:4}], VGrooveFold, "fold and sealed trim"),
    new LaminatedPanel("pp-6", [{materialId:"pp", thicknessMm:6}], HotGasWeld, "hot-gas welded seam"),
    new LaminatedPanel("hdpe-6", [{materialId:"hdpe", thicknessMm:6}], HotGasWeld, "welded seam"),
    new LaminatedPanel("uhmw-pe-6", [{materialId:"uhmw-pe", thicknessMm:6}], RouterCut, "mechanically retained liner"),
    new LaminatedPanel("steel-2", [{materialId:"machined-steel", thicknessMm:2}], PressBrakeFold, "folded and sealed", Epoxy),
    new LaminatedPanel("polycarbonate-6", [{materialId:"polycarbonate", thicknessMm:6}], RouterCut, "retained glazing frame"),
    new LaminatedPanel("al-honeycomb-20", [{materialId:"al-honeycomb", thicknessMm:20}], BoughtFinished, "bonded edge closure"),
    new LaminatedPanel("insulated-sandwich-40", [{materialId:"insulated-sandwich", thicknessMm:40}], BoughtFinished, "sealed panel profile"),
    new LaminatedPanel("stainless-faced-acm", [
      {materialId:"fr-acm", thicknessMm:4}, {materialId:"panel-adhesive", thicknessMm:0.2},
      {materialId:"stainless-sheet", thicknessMm:0.5}], BoughtFinished, "sealed edge closure"),
    new LaminatedPanel("grp-faced-acm", [
      {materialId:"fr-acm", thicknessMm:4}, {materialId:"panel-adhesive", thicknessMm:0.2},
      {materialId:"grp", thicknessMm:1.5}], WetLayup, "sealed edge closure")
  ];

  public static function require(id:String):LaminatedPanel {
    for (panel in all()) if (panel.id == id) return panel;
    throw 'Unknown panel construction "$id"';
  }
}
