#ifndef NATIVEKIT_SCENE_SHADER_SOURCES_HPP
#define NATIVEKIT_SCENE_SHADER_SOURCES_HPP

#include "nativekit_gpu.h"
#include "scene_shader_sources.h"

namespace nkscene::render_internal {

struct SceneShaderSources {
    const char *vertex = nullptr;
    const char *fragment = nullptr;
    nkgpu_shader_language language = 0;
};

using PostProcessShaderSources = SceneShaderSources;

inline SceneShaderSources scene_shader_sources(nkgpu_backend backend, bool picking) {
    switch (backend) {
    case NKGPU_BACKEND_GLCORE:
        return picking ? SceneShaderSources{shader_source::pick_vertex_gl,
                                            shader_source::pick_fragment_gl,
                                            NKGPU_SHADERLANGUAGE_GLSL}
                       : SceneShaderSources{shader_source::scene_vertex_gl,
                                            shader_source::scene_fragment_gl,
                                            NKGPU_SHADERLANGUAGE_GLSL};
    case NKGPU_BACKEND_GLES3:
        return picking ? SceneShaderSources{shader_source::pick_vertex_gles,
                                            shader_source::pick_fragment_gles,
                                            NKGPU_SHADERLANGUAGE_GLSL}
                       : SceneShaderSources{shader_source::scene_vertex_gles,
                                            shader_source::scene_fragment_gles,
                                            NKGPU_SHADERLANGUAGE_GLSL};
    case NKGPU_BACKEND_D3D11:
        return picking ? SceneShaderSources{shader_source::scene_pick_vertex_hlsl, shader_source::scene_pick_fragment_hlsl,
                                            NKGPU_SHADERLANGUAGE_HLSL5}
                       : SceneShaderSources{shader_source::scene_vertex_hlsl, shader_source::scene_fragment_hlsl,
                                            NKGPU_SHADERLANGUAGE_HLSL5};
    case NKGPU_BACKEND_METAL:
        return picking ? SceneShaderSources{shader_source::scene_pick_vertex_metal, shader_source::scene_pick_fragment_metal,
                                            NKGPU_SHADERLANGUAGE_MSL}
                       : SceneShaderSources{shader_source::scene_vertex_metal, shader_source::scene_fragment_metal,
                                            NKGPU_SHADERLANGUAGE_MSL};
    default:
        return {};
    }
}

inline SceneShaderSources stroke_shader_sources(nkgpu_backend backend) {
    switch (backend) {
    case NKGPU_BACKEND_GLCORE:
        return {shader_source::stroke_vertex_gl, shader_source::stroke_fragment_gl,
                NKGPU_SHADERLANGUAGE_GLSL};
    case NKGPU_BACKEND_GLES3:
        return {shader_source::stroke_vertex_gles, shader_source::stroke_fragment_gles,
                NKGPU_SHADERLANGUAGE_GLSL};
    case NKGPU_BACKEND_D3D11:
        return {shader_source::stroke_vertex_hlsl, shader_source::stroke_fragment_hlsl,
                NKGPU_SHADERLANGUAGE_HLSL5};
    case NKGPU_BACKEND_METAL:
        return {shader_source::stroke_vertex_metal, shader_source::stroke_fragment_metal,
                NKGPU_SHADERLANGUAGE_MSL};
    default:
        return {};
    }
}

inline PostProcessShaderSources post_process_shader_sources(nkgpu_backend backend) {
    switch (backend) {
    case NKGPU_BACKEND_GLCORE:
        return {shader_source::postprocess_vertex_gl, shader_source::postprocess_fragment_gl,
                NKGPU_SHADERLANGUAGE_GLSL};
    case NKGPU_BACKEND_GLES3:
        return {shader_source::postprocess_vertex_gles, shader_source::postprocess_fragment_gles,
                NKGPU_SHADERLANGUAGE_GLSL};
    default:
        return {};
    }
}

} // namespace nkscene::render_internal

#endif
