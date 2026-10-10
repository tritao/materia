package processkit.collision;

import collisionkit.CollisionDescription;
import collisionkit.CollisionGeometry;
import collisionkit.CollisionPose;
import processkit.work.HeightMap;

/**
 * A `HeightMap` as a collision height field (COLLISION.md CL3a, CL-D6). The
 * map's vertices run along +x by column and +y by row from its origin; a
 * collision height field is centred on its offset with rows along -y, so
 * rows are flipped and the offset is the map's centre in its frame.
 *
 * The field is a solid down to `floor`, which must lie below the deepest
 * dig: the world refuses heights below it rather than clamp them.
 */
class HeightMapCollision {
  public static function geometry(map:HeightMap, floor:Float):CollisionGeometry {
    if (map.columns < 2 || map.rows < 2) throw "A terrain height field needs at least 2 x 2 vertices";
    return CollisionGeometry.HeightField((map.columns - 1) * map.cellSize, (map.rows - 1) * map.cellSize,
      heights(map), map.rows, floor);
  }

  /** The field's offset in the map's frame: its centre. */
  public static function offset(map:HeightMap):CollisionPose
    return CollisionPose.translation(map.originX + (map.columns - 1) * map.cellSize / 2,
      map.originY + (map.rows - 1) * map.cellSize / 2, 0.0);

  /** The heights in the field's layout (rows along -y), for `CollisionWorld.setHeights` after digging. */
  public static function heights(map:HeightMap):Array<Float>
    return [for (row in 0...map.rows) for (col in 0...map.columns) map.elevationAt(col, map.rows - 1 - row)];

  /**
   * Adds the terrain as a fixed body at `framePose` (the map frame's pose in
   * the cell) in `group`; returns the described object.
   */
  public static function describe(description:CollisionDescription, name:String, map:HeightMap, floor:Float,
      framePose:CollisionPose, group:Int):Int {
    var body = description.addBody(name, group, true, framePose);
    description.note('Terrain "$name" is a height field down to $floor; digs below it are refused');
    return description.addObject(name, body, offset(map), geometry(map, floor), 0.0, "height field");
  }
}
