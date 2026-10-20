package machinekit.enclosure;

/** Preliminary removable laminate tray stack, millimetres. Gasket/material qualification is separate. */
class TraySeat {
  public static inline final WIDTH=25.0;
  public static inline final DEPTH=4.0;
  public static inline final GASKET=2.0;
  public static inline final THICKNESS=13.0;
  public static inline final BOTTOM=GASKET-DEPTH;
  public static inline final TOP=BOTTOM+THICKNESS;
  public static inline final SEAM=2.0;
  public static inline final CONSTRUCTION="compact-laminate-13";
}
