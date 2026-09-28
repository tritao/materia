package toolpathkit.path;

/** Intended role of a move along the path. */
enum MoveKind {
  Rapid;
  Cut;
  Plunge;
  Ramp;
  Link;
  Retract;
}
