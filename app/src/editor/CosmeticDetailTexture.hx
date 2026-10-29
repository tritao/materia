package app.editor;

import app.CadGeometryData.CadDetailGeometry;
import cadkit.threads.DetailTexture;
import materia.project.SurfaceDetailRenderData.SurfaceDetailKinds;
import nativekit.scene.ImageData;

/** CadKit's DetailTexture as a scene image with its mip levels. */
class CosmeticDetailTexture {
  /** Identifies the texture, so details with the same numbers share it. */
  public static function key(detail:CadDetailGeometry):String {
    var source = detail.source;
    return source.kind
      == SurfaceDetailKinds.KNURL ? 'knurl|${source.count}|${source.depth}|${Math.round(detail.radiusMm * 100)}'
      : 'scale|${source.count}|${source.numberedEvery}|${Math.round(detail.radiusMm * 100)}|${Math.round(detail.radiusEndMm * 100)}|${Math.round(detail.surfaceMm * 100)}';
  }

  public static function image(detail:CadDetailGeometry, normal:Bool):ImageData {
    var source = detail.source;
    var made = source.kind == SurfaceDetailKinds.KNURL ? DetailTexture.knurl(
      source.count,
      source.depth,
      detail.radiusMm,
      normal
    ) : DetailTexture.graduation(
      source.count,
      source.numberedEvery,
      detail.radiusMm,
      detail.radiusEndMm,
      detail.surfaceMm,
      normal
    );
    return new ImageData(made.width, made.height).setMipCount(made.mips).setPixels(made.pixels);
  }
}
