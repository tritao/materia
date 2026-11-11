package cadkit.drawing;

/** Manufacturing layers shared by drawing formats. */
enum DrawingLayer implements Equatable {
  Outline;
  BendUp;
  BendDown;
  Etch;
  Text;
  /** Named manufacturing process layer, independent of a particular consumer. */
  Process(name:String);
}
