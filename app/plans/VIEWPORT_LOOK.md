# Viewport look: one lighting definition for the editor and the docs

The editor's 3D view and the docs' component viewers should light a part the same way, from one definition, and
metal should read as metal in both.

## Where it started (2026-10-21)

- SceneKit's scene shader is metallic-roughness GGX with up to 32 lights, a sky/ground hemisphere ambient, a
  two-colour "environment" reflection (the same sky and ground colours, mixed by reflection height, at a fixed
  strength of 0.6), screen-space obscurance and a studio roughness floor. No tone mapping: colour is clamped.
- The CAD studio rig is three camera-relative directional lights (key, fill, rim). Its values lived in two places:
  `haxeon.editor.ViewportLook` (presets) and `app/EditorViewportLighting` (overriding the Studio preset, the light
  directions of every preset, and the surface detail). Haxeon's own Studio values were not what the editor showed.
- Metal parts (most of MachineKit: machined steel, anodised aluminium) look flat: a metal has no diffuse term, so
  it shows its reflections, and the only thing to reflect was a two-colour gradient plus three point highlights.
- The docs catalogue used `<model-viewer>` with its default neutral environment, a different camera from its own
  thumbnails, and wrote the material colours (sRGB, as the editor's shader reads them) as glTF's linear factors.

## Steps

| Step | What |
|---|---|
| L0 | Docs export writes base colours as linear (sRGB decode), as glTF requires. |
| L1 | `haxeon.editor.ViewportLook` owns each preset whole: lights (camera-space direction, intensity, colour, panel size), sky and ground, reflection strength, obscurance, roughness floor, exposure and background. `EditorViewportLighting` is deleted; the app picks a preset and passes it through. The app's current Studio values become Haxeon's Studio, so the editor looks the same. |
| L2 | Display mapping in SceneKit: exposure and the Khronos PBR Neutral tone mapper on scene shading (strokes, grid and overlays keep exact colours). Exposure zero keeps the old clamp, so views without a look are unchanged. |
| L3 | Studio lights become soft panels with an angular size, and the room becomes sky above the horizon over ground below (the hemisphere ambient is exactly its irradiance), its edge blurred by roughness in reflections. Their reflections come from an analytic environment: each panel is a Gaussian lobe whose width grows with surface roughness and whose radiance keeps the panel's energy equal to the directional light's (`L = I / r^2` for a panel of angular radius `r`), added to the sky/ground room. Panels replace the point GGX highlight of their light; diffuse stays directional. No textures, so it runs unchanged on every backend, including the browser build, and it turns with the camera like the lights. |
| L4 | Docs from the same look. The docs exporter reads `ViewportLook` and writes, per catalogue view, an equirectangular HDR of the same room and panels placed for that view's camera, plus the viewer settings (exposure, Neutral tone mapping) and the roughness floor applied to the exported materials. `docs/screens/studio.py` goes away. model-viewer's environment is fixed in the world, so the docs match the editor exactly at a part's first frame (its thumbnail) and drift as you orbit. |
| L5 | Optional soft ground shadow per preset: on for presentation, off for CAD inspection. Not started. |

## Status (2026-10-21)

L0–L4 are done. The new shading is opt-in per view (`SceneView.setStudioEnvironment`); without it
SceneKit renders exactly as before, which the lighting regression references check. The Studio room is
brighter than the old fill-light values (sky 0.50, ground 0.22) because metal now shows the room it
reflects.

Follow-ups:

- MachineKit's metal colours are below real reflectance for a PBR pipeline: machined steel is sRGB 0.62
  at metallic 0.8 (about 0.28 reflectance), real steel is about 0.56 (sRGB 0.78), so steel reads dark in
  both the editor and the docs. Fix in `materia.project.MaterialLibrary`, checked against the catalogue.
- L5. The docs viewer already draws model-viewer's ground shadow; an editor shadow would match it.

## Checks

- SceneKit: the lighting gallery and studio tests, plus a test that a studio panel's reflection appears on a
  metallic surface and that exposure zero reproduces the clamped output.
- Haxeon editor package: DocCheck covers every preset's fields.
- App: before/after screenshots of the Studio, Soft and Contrast presets; L1 alone must not change pixels.
- Docs: re-exported models and thumbnails; a part's thumbnail equals its viewer's first frame.
