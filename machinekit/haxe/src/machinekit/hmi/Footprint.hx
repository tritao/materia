package machinekit.hmi;

/** A flat shape on the face, centred on a control's position. Millimetres. */
enum Footprint {
  Circle(diameter:Float);
  Rect(width:Float, height:Float);
}
