package robotkit.navigation;

/** Discrete occupancy values stored by an OccupancyGrid2. */
enum abstract OccupancyCell(Int) from Int to Int {
  var Unknown = -1;
  var Free = 0;
  var Occupied = 1;
}
