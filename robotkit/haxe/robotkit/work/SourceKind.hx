package robotkit.work;

/** Where a WorkSurface's geometry came from. */
enum abstract SourceKind(String) from String to String {
  var Design = "design";
  var Observed = "observed";
  var Work = "work";
}
