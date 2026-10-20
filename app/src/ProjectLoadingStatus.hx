package app;

/** A snapshot shared by editor chrome, independent of the selected workspace tab. */
typedef ProjectLoadingStatus = {
  var title:String;
  var phase:String;
  var elapsedSeconds:Int;
  var cancelling:Bool;
}
