// Included by gpu_executor.cpp after its surface and pick pipeline builders.
// Stencil parity is private render state: no mesh or presentation is modified.
template<class StateT>
bool prepare_caps(StateT &state,const RenderPlan &plan,const SceneSnapshot &snapshot,
                  unsigned width,unsigned height,unsigned samples,bool picking,bool for_ao,GpuExecutionStats &stats) {
    state.caps.clear();
    if(state.cap_instances.id){(void)nkgpu_buffer_destroy(state.renderer,state.cap_instances);state.cap_instances={};}
    if(plan.clip_planes().empty())return true;
    const std::array<float,16> identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    std::vector<InstanceData> records{{identity,{},{}}};
    const auto &vp=plan.view_projection();
    for(std::size_t b=0;b<state.batches.size();++b) {
        const auto &batch=state.batches[b];
        const auto *geometry=snapshot.find_geometry(batch.key.geometry);
        const auto *material=snapshot.find_material(batch.key.material);
        if(!geometry||!geometry->payload->closed||geometry->payload->primitive_type!=PrimitiveType::Triangles||
           !material||material->state->base_color[3]*material->state->opacity<1.0f||material_is_blended(*material->state)||material->state->alpha_mode==AlphaMode::Mask||
           has_material_flag(material->state->flags,MaterialFlags::DoubleSided))continue;
        for(std::size_t instance=0;instance<batch.records.size();++instance) {
            LocalTransform transform;transform.matrix=batch.records[instance].transform;
            auto bounds=render_internal::transformed_bounds(geometry->bounds,transform);if(!bounds.valid)continue;
            std::array<float,3> center{},half{};
            for(int a=0;a<3;++a){center[a]=(bounds.minimum[a]+bounds.maximum[a])*0.5f;half[a]=(bounds.maximum[a]-bounds.minimum[a])*0.5f;}
            std::array<int,4> scissor{0,0,static_cast<int>(width),static_cast<int>(height)};
            float minx=static_cast<float>(width),miny=static_cast<float>(height),maxx=0,maxy=0;bool near_eye=false;
            for(float x:{bounds.minimum[0],bounds.maximum[0]})for(float y:{bounds.minimum[1],bounds.maximum[1]})for(float z:{bounds.minimum[2],bounds.maximum[2]}) {
                float w=vp[3]*x+vp[7]*y+vp[11]*z+vp[15];
                if(w<=1e-6f){near_eye=true;continue;}
                float px=((vp[0]*x+vp[4]*y+vp[8]*z+vp[12])/w+1)*width*0.5f;
                float py=(1-(vp[1]*x+vp[5]*y+vp[9]*z+vp[13])/w)*height*0.5f;
                minx=std::min(minx,px);maxx=std::max(maxx,px);miny=std::min(miny,py);maxy=std::max(maxy,py);
            }
            if(!near_eye) {
                int x=std::clamp(static_cast<int>(std::floor(std::clamp(minx,-1.f,static_cast<float>(width)+1)))-1,0,static_cast<int>(width));
                int y=std::clamp(static_cast<int>(std::floor(std::clamp(miny,-1.f,static_cast<float>(height)+1)))-1,0,static_cast<int>(height));
                int right=std::clamp(static_cast<int>(std::ceil(std::clamp(maxx,-1.f,static_cast<float>(width)+1)))+1,0,static_cast<int>(width));
                int bottom=std::clamp(static_cast<int>(std::ceil(std::clamp(maxy,-1.f,static_cast<float>(height)+1)))+1,0,static_cast<int>(height));
                scissor={x,y,right-x,bottom-y};if(right<=x||bottom<=y)continue;
            }
            bool outside=false;
            for(const auto &plane:plan.clip_planes()) {
                float distance=plane[3],radius=0;
                for(int a=0;a<3;++a){distance+=plane[a]*center[a];radius+=std::abs(plane[a])*half[a];}
                if(distance+radius<0){outside=true;break;}
            }
            if(outside)continue;
            for(std::size_t p=0;p<plan.clip_planes().size();++p) {
                const auto &plane=plan.clip_planes()[p];float distance=plane[3],radius=0,n2=0;
                for(int a=0;a<3;++a){distance+=plane[a]*center[a];radius+=std::abs(plane[a])*half[a];n2+=plane[a]*plane[a];}
                if(n2<=0||distance-radius>=0||distance+radius<=0)continue;
                std::array<float,3> n{},u{},v{},at{};float norm=std::sqrt(n2);
                for(int a=0;a<3;++a){n[a]=-plane[a]/norm;at[a]=center[a]-plane[a]*distance/n2;}
                // Stable plane-space basis, independent of camera and instance parity.
                u=std::abs(n[2])<0.9f?std::array<float,3>{-n[1],n[0],0}:std::array<float,3>{n[2],0,-n[0]};
                float len=std::sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);for(auto &a:u)a/=len;
                v={n[1]*u[2]-n[2]*u[1],n[2]*u[0]-n[0]*u[2],n[0]*u[1]-n[1]*u[0]};
                float extent=std::sqrt(half[0]*half[0]+half[1]*half[1]+half[2]*half[2])*1.01f;
                InstanceData record{identity,batch.records[instance].pick_color,batch.records[instance].highlight_tint};
                for(int a=0;a<3;++a){record.transform[a]=u[a]*extent;record.transform[4+a]=v[a]*extent;record.transform[8+a]=n[a];record.transform[12+a]=at[a];}
                records.push_back(record);state.caps.push_back({b,instance,p,scissor});
            }
        }
    }
    if(state.caps.empty())return true;
    auto upload=[&](const auto &values,nkgpu_buffer &buffer){return nkgpu_buffer_create(state.renderer,reinterpret_cast<const std::uint8_t *>(values.data()),static_cast<unsigned>(values.size()*sizeof(values[0])),&buffer);};
    if(!state.cap_vertices.id) {
        std::array<SceneVertex,6> vertices{};std::array<PickVertex,6> picks{};
        const float xy[6][2]={{-1,-1},{1,-1},{1,1},{-1,-1},{1,1},{-1,1}};
        for(int i=0;i<6;++i){vertices[i].position={xy[i][0],xy[i][1],0};picks[i].position=vertices[i].position;}
        if(auto r=upload(vertices,state.cap_vertices);r!=NKGPU_OK)return set_failure(state,stats,r);
        if(auto r=upload(picks,state.cap_pick_vertices);r!=NKGPU_OK)return set_failure(state,stats,r);
    }
    if(auto r=upload(records,state.cap_instances);r!=NKGPU_OK)return set_failure(state,stats,r);
    for(int mode=1;mode<=3;++mode) {
        if(picking){if(!ensure_pick_pipeline(state,stats,for_ao,mode))return false;}
        else if(!ensure_pipeline(state,stats,false,false,samples,mode))return false;
    }
    if(!picking)for(const auto &cap:state.caps) {
        auto &gpu=state.geometry_resources.at(state.batches[cap.batch].key.geometry);
        if(gpu.indexed&&!ensure_pipeline(state,stats,true,false,samples,2))return false;
    }
    return true;
}

template<class StateT>
nkgpu_result draw_caps(StateT &state,const RenderPlan &plan,const SceneSnapshot &snapshot,
                       const LightingUniformData &lighting,unsigned samples,bool picking,bool for_ao,GpuExecutionStats &stats) {
    const std::array<float,16> identity{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
    const auto &set=state.pipelines[pipeline_slot(samples)];
    std::size_t cap_index=0;
    for(const auto &cap:state.caps) {
        const auto &batch=state.batches[cap.batch];const auto &gpu=state.geometry_resources.at(batch.key.geometry);
        const auto *material=snapshot.find_material(batch.key.material);
        const auto &textures=state.material_resources.at(batch.key.material);
        auto result=nkgpu_apply_scissor(state.renderer,1,cap.scissor[0],cap.scissor[1],cap.scissor[2],cap.scissor[3]);if(result!=NKGPU_OK)return result;
        for(int mode=1;mode<=3;++mode) {
            bool parity=mode==2,clear=mode==1;ClipUniformData clips{};
            for(std::size_t p=0;p<plan.clip_planes().size();++p)if((parity&&p==cap.plane)||(!clear&&!parity&&p!=cap.plane))clips.planes[static_cast<int>(clips.count[0]++)]=plan.clip_planes()[p];
            auto pipeline=picking?state.pick_caps[for_ao?1:0][mode-1]:parity&&gpu.indexed?set.indexed_parity:set.caps[mode-1];
            if((result=nkgpu_apply_pipeline(state.renderer,pipeline))!=NKGPU_OK)return result;
            const auto &vp=clear?identity:plan.view_projection();
            if((result=nkgpu_apply_uniform_data(state.renderer,1,reinterpret_cast<const std::uint8_t *>(vp.data()),sizeof(vp)))!=NKGPU_OK)return result;
            auto vertex=parity?(picking?gpu.pick_buffer:gpu.buffer):(picking?state.cap_pick_vertices:state.cap_vertices);
            if((result=nkgpu_apply_vertex_buffer(state.renderer,0,vertex,0))!=NKGPU_OK)return result;
            if(parity&&!picking&&gpu.indexed)if((result=nkgpu_apply_index_buffer(state.renderer,gpu.index_buffer,0))!=NKGPU_OK)return result;
            auto instances=parity?batch.buffer:state.cap_instances;
            auto offset=parity?cap.instance:clear?0:cap_index+1;
            if((result=nkgpu_apply_vertex_buffer(state.renderer,1,instances,static_cast<unsigned>(offset*instance_stride)))!=NKGPU_OK)return result;
            if((result=nkgpu_apply_uniform_data(state.renderer,2,reinterpret_cast<const std::uint8_t *>(&clips),sizeof(clips)))!=NKGPU_OK)return result;
            if(picking) {
                std::array<float,4> output{for_ao?1.f:0.f,1.f,mode==3?2.f:1.f,static_cast<float>(cap.plane)};
                if((result=nkgpu_apply_uniform_data(state.renderer,0,reinterpret_cast<const std::uint8_t *>(output.data()),sizeof(output)))!=NKGPU_OK)return result;
            } else {
                MaterialUniformData data{};data.base_color=material->state->base_color;data.base_color[3]=1;
                data.surface_params={0,0.8f,0,0};data.texture_flags[3]=mode==3?2.f:3.f;
                if((result=nkgpu_apply_uniform_data(state.renderer,0,reinterpret_cast<const std::uint8_t *>(&data),sizeof(data)))!=NKGPU_OK)return result;
                if((result=nkgpu_apply_uniform_data(state.renderer,3,reinterpret_cast<const std::uint8_t *>(&lighting),sizeof(lighting)))!=NKGPU_OK)return result;
                for(unsigned i=0;i<5;++i)if((result=nkgpu_apply_image(state.renderer,i,textures.images[i]))!=NKGPU_OK)return result;
                if((result=nkgpu_apply_image(state.renderer,5,lighting.ao_params[1]>0?state.ao_position:textures.images[3]))!=NKGPU_OK)return result;
                if((result=nkgpu_apply_sampler(state.renderer,0,textures.sampler))!=NKGPU_OK)return result;
                if((result=nkgpu_apply_sampler(state.renderer,1,state.ao_sampler))!=NKGPU_OK)return result;
            }
            unsigned count=parity?(picking?gpu.pick_vertex_count:gpu.indexed?gpu.index_count:gpu.vertex_count):6;
            if((result=nkgpu_draw(state.renderer,0,count,1))!=NKGPU_OK)return result;
            ++stats.draw_calls;
        }
        ++cap_index;
    }
    return nkgpu_apply_scissor(state.renderer,0,0,0,0,0);
}
