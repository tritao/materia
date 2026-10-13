/** Plain convex fixture data; production collision sources use descriptions and shapes. */
typedef ClearanceBodyData = {
  var name:String;
  var link:String;
  var vertices:Array<Float>;
  var tool:Bool;
}
