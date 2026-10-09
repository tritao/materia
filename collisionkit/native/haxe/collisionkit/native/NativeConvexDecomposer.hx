package collisionkit.native;

import CollisionKitNative;
import collisionkit.ConvexDecomposer;
import collisionkit.ConvexDecomposition;

/** V-HACD 4 through collisionkit's C ABI (`ck_decompose`), with the enclosure measured natively (CL-D11). */
class NativeConvexDecomposer implements ConvexDecomposer {
  /** V-HACD's voxel resolution. */
  public final resolution:Int;

  public function new(resolution:Int = 100000) {
    this.resolution = resolution;
  }

  public function decompose(vertices:Array<Float>, indices:Array<Int>, maxPieces:Int, sampleSpacing:Float):ConvexDecomposition {
    var created = CollisionKitNative.ck_decompose(vertices, indices, maxPieces, resolution, 64, sampleSpacing);
    if (created.status != CollisionKitNativeConstants.CK_OK)
      throw 'Convex decomposition failed with error ${created.status}';
    var handle = created.out_decomposition;
    try {
      var info = CollisionKitNative.ck_decomposition_info(handle.borrow(), 3);
      if (info.status != CollisionKitNativeConstants.CK_OK) throw 'Convex decomposition info failed with error ${info.status}';
      var pieces:Array<Array<Float>> = [];
      for (piece in 0...info.out_pieces) {
        var result = CollisionKitNative.ck_decomposition_piece(handle.borrow(), piece, 192);
        if (result.status != CollisionKitNativeConstants.CK_OK) throw 'Convex decomposition piece failed with error ${result.status}';
        pieces.push(result.out_points.slice(0, result.out_count));
      }
      var values = info.out_values;
      handle.close();
      return new ConvexDecomposition(pieces, values[0], values[1], values[2]);
    } catch (error:Dynamic) {
      handle.close();
      throw error;
    }
  }
}
