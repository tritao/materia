// Included after section_caps.hpp: section contours share cap eligibility and
// stroke geometry, but have their own depth precedence and clipping predicate.
template<class StateT>
bool prepare_contours(StateT &state,const RenderPlan &plan,const SceneSnapshot &snapshot,
                      unsigned samples,GpuExecutionStats &stats) {
    const auto *diagnostic=std::getenv("NK_SCENE_SECTION_CONTOURS");
    if(diagnostic && std::string(diagnostic)=="0")return true;
    for(auto it=state.contours.begin();it!=state.contours.end();) {
        if(!snapshot.find_node(NodeId{it->first.first})) {
            if(it->second.buffer.id)(void)nkgpu_buffer_destroy(state.renderer,it->second.buffer);
            it=state.contours.erase(it);
        } else ++it;
    }
    if(!state.caps.empty()&&!ensure_stroke_pipeline(state,stats,samples,true))return false;
    for(const auto &cap:state.caps) {
        const auto &batch=state.batches[cap.batch];
        const auto *geometry=snapshot.find_geometry(batch.key.geometry);
        auto local=render_internal::section_plane_local(plan.clip_planes()[cap.plane],batch.records[cap.instance].transform);
        auto &entry=state.contours[{batch.instances[cap.instance].value,cap.plane}];
        const bool changed=entry.section.update(geometry->payload,local,geometry->revision);
        if(changed){++stats.section_contours_rebuilt;entry.uploaded=false;}
        else ++stats.section_contour_cache_hits;
        if(entry.uploaded)continue;
        const auto &segments=entry.section.segments;
        if(segments.size()>std::numeric_limits<std::uint32_t>::max()/(18*sizeof(StrokeVertex)))
            return set_failure(state,stats,NKGPU_ERROR_INVALID_ARGUMENT);
        std::vector<StrokeVertex> vertices;vertices.reserve(segments.size()*18);
        for(const auto &segment:segments)for(const auto &corner:stroke_corners)
            vertices.push_back({segment.start,segment.end,corner});
        const auto bytes=static_cast<std::uint32_t>(vertices.size()*sizeof(StrokeVertex));
        if(bytes>entry.capacity) {
            if(entry.buffer.id)(void)nkgpu_buffer_destroy(state.renderer,entry.buffer);
            entry.buffer={};entry.capacity=0;
            std::uint32_t capacity=1024;
            while(capacity<bytes && capacity<=std::numeric_limits<std::uint32_t>::max()/2)capacity*=2;
            capacity=std::max(capacity,bytes);
            nkgpu_buffer_desc descriptor{};descriptor.struct_size=sizeof(descriptor);
            descriptor.size=capacity;descriptor.usage=NKGPU_BUFFER_VERTEX;descriptor.dynamic_update=1;
            nkgpu_result result=nkgpu_buffer_create_desc(state.renderer,&descriptor,&entry.buffer);
            if(result!=NKGPU_OK)return set_failure(state,stats,result);
            entry.capacity=capacity;
        }
        if(bytes)if(auto result=nkgpu_buffer_update(state.renderer,entry.buffer,0,
            reinterpret_cast<const std::uint8_t *>(vertices.data()),bytes);result!=NKGPU_OK)
            return set_failure(state,stats,result);
        entry.count=static_cast<std::uint32_t>(vertices.size());entry.uploaded=true;
    }
    return true;
}

template<class StateT>
bool draw_contours(StateT &state,const RenderPlan &plan,const SceneSnapshot &snapshot,
                   unsigned samples,GpuExecutionStats &stats) {
    const auto *diagnostic=std::getenv("NK_SCENE_SECTION_CONTOURS");
    if(diagnostic && std::string(diagnostic)=="0")return true;
    if(state.caps.empty())return true;
    std::array<float,16> inverse{};
    if(!invert_matrix(plan.view_projection(),inverse))return true;
    StrokeUniformData viewport{{float(std::max(state.viewport_width,1u)),float(std::max(state.viewport_height,1u)),0.85f,1.0f}};
    for(unsigned row=0;row<4;++row) {
        viewport.x_unprojection[row]=inverse[row];
        viewport.y_unprojection[row]=inverse[4+row];
        viewport.depth_unprojection[row]=inverse[8+row];
    }
    for(const auto &cap:state.caps) {
        const auto &batch=state.batches[cap.batch];
        const auto &entry=state.contours.at({batch.instances[cap.instance].value,cap.plane});
        if(!entry.count)continue;
        viewport.contour_plane=plan.clip_planes()[cap.plane];
        ClipUniformData clips{};clips.count[1]=1; // Geometric contours, not surface-edge remnants.
        for(std::size_t p=0;p<plan.clip_planes().size();++p)if(p!=cap.plane)
            clips.planes[static_cast<std::size_t>(clips.count[0]++)]=plan.clip_planes()[p];
        const auto color=stroke_color_for(plan,snapshot.find_material(batch.key.material));
        auto result=nkgpu_apply_pipeline(state.renderer,state.pipelines[pipeline_slot(samples)].contour);
        if(result==NKGPU_OK)result=nkgpu_apply_uniform_data(state.renderer,1,
            reinterpret_cast<const std::uint8_t *>(plan.view_projection().data()),sizeof(float)*16);
        if(result==NKGPU_OK)result=nkgpu_apply_uniform_data(state.renderer,3,reinterpret_cast<const std::uint8_t *>(&viewport),sizeof(viewport));
        if(result==NKGPU_OK)result=nkgpu_apply_uniform_data(state.renderer,0,reinterpret_cast<const std::uint8_t *>(color.data()),sizeof(color));
        if(result==NKGPU_OK)result=nkgpu_apply_uniform_data(state.renderer,2,reinterpret_cast<const std::uint8_t *>(&clips),sizeof(clips));
        if(result==NKGPU_OK)result=nkgpu_apply_vertex_buffer(state.renderer,0,entry.buffer,0);
        if(result==NKGPU_OK)result=nkgpu_apply_vertex_buffer(state.renderer,1,batch.buffer,static_cast<unsigned>(cap.instance*instance_stride));
        if(result==NKGPU_OK)result=nkgpu_draw(state.renderer,0,entry.count,1);
        if(result!=NKGPU_OK)return set_failure(state,stats,result);
        ++stats.draw_calls;
    }
    return true;
}
