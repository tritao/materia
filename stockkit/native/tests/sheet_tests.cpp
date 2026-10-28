// Sheet-scale storage checks through the public ABI. Geometry units: mm.
#include "stockkit.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#if defined(__linux__)
#include <sys/resource.h>
#endif
namespace {
void require(bool ok, const char *what) { if (!ok) throw std::runtime_error(what); }
uint64_t hash_bytes(uint64_t h, const void *ptr, size_t bytes) {
    const auto *p = static_cast<const unsigned char *>(ptr);
    while (bytes--) { h ^= *p++; h *= 1099511628211ull; } return h;
}
sk_stock_info info(sk_stock_handle stock) { sk_stock_info v{};v.struct_size=sizeof v;require(sk_stock_get_info(stock,&v)==SK_OK,"stock info");return v; }
sk_move move(double x0,double y0,double z0,double x1,double y1,double z1,uint32_t source) {
    sk_move m{};m.struct_size=sizeof m;m.kind=SK_MOVE_LINE;m.source=source;
    m.start[0]=x0;m.start[1]=y0;m.start[2]=z0;m.end[0]=x1;m.end[1]=y1;m.end[2]=z1;return m;
}
sk_tool_handle cutter() {
    sk_profile_segment p[2]{};
    for(auto &s:p){s.struct_size=sizeof s;s.kind=SK_SEGMENT_LINE;}
    p[0].r1=3;p[1].r0=3;p[1].r1=3;p[1].z1=40;
    sk_tool_handle t{};require(sk_tool_create(p,2,&t)==SK_OK,"flat cutter");return t;
}
sk_lattice lattice(double width,double length,double spacing) {
    sk_lattice g{};g.struct_size=sizeof g;g.axes=SK_AXES_ALL;g.spacing=spacing;g.tile_size=16;
    g.origin[0]=g.origin[1]=g.origin[2]=spacing/2;
    g.count[0]=uint32_t(std::ceil(width/spacing));g.count[1]=uint32_t(std::ceil(length/spacing));g.count[2]=uint32_t(std::ceil(18/spacing));return g;
}
sk_stock_handle sheet(double width,double length,bool mesh) {
    auto g=lattice(width,length,.5);sk_stock_handle s{};
    if(!mesh) {
        sk_box b{};b.struct_size=sizeof b;b.max[0]=width;b.max[1]=length;b.max[2]=18;
        require(sk_stock_create_box(&g,&b,&s)==SK_OK,"box sheet");
    } else {
        double p[]={0,0,0,width,0,0,width,length,0,0,length,0,0,0,18,width,0,18,width,length,18,0,length,18};
        uint32_t i[]={0,2,1,0,3,2,4,5,6,4,6,7,0,1,5,0,5,4,1,2,6,1,6,5,2,3,7,2,7,6,3,0,4,3,4,7};
        require(sk_stock_create_mesh(&g,p,24,i,36,&s)==SK_OK,"mesh sheet");
    }
    return s;
}
std::vector<sk_move> cabinet(double width,double length) {
    std::vector<sk_move> moves;uint32_t source=0;
    // Two columns of four door/shelf blanks on the long sheet (three short).
    const int rows=length>2800?4:3;double w=(width-36)/2,h=(length-20-8*(rows-1))/rows;
    for(int row=0;row<rows;++row)for(int col=0;col<2;++col) {
        double x=10+col*(w+8),y=10+row*(h+8);
        double xy[5][2]={{x,y},{x+w,y},{x+w,y+h},{x,y+h},{x,y}};
        for(int edge=0;edge<4;++edge)moves.push_back(move(xy[edge][0],xy[edge][1],.5,xy[edge+1][0],xy[edge+1][1],.5,source++));
        for(int hole=0;hole<4;++hole)moves.push_back(move(x+37,y+37+32*hole,20,x+37,y+37+32*hole,4,source++));
    }
    return moves;
}
uint64_t digest(sk_stock_handle s) {
    auto v=info(s);uint64_t hash=1469598103934665603ull;
    hash=hash_bytes(hash,&v.volume,sizeof v.volume);
    for(uint32_t axis=0;axis<3;++axis) {
        const auto &g=v.grids[axis];const uint32_t width=g.count[0];
        std::vector<uint32_t> counts(width);
        for(uint32_t row=0;row<g.count[1];++row) {
            uint32_t total=0;auto rc=sk_stock_read_rays(s,axis,0,row,width,1,counts.data(),width,nullptr,&total);
            require(rc==SK_ERROR_LIMIT||rc==SK_OK,"ray size");std::vector<sk_interval> values(total);
            require(sk_stock_read_rays(s,axis,0,row,width,1,counts.data(),width,values.data(),&total)==SK_OK,"ray data");
            hash=hash_bytes(hash,counts.data(),counts.size()*sizeof(uint32_t));hash=hash_bytes(hash,values.data(),values.size()*sizeof(sk_interval));
        }
    }
    return hash;
}
void small() {
    auto tool=cutter();auto moves=cabinet(160,315); // small deterministic pattern, same cut algebra
    uint64_t reference=0,box_reference=0;
    for(bool mesh:{false,true})for(uint32_t threads:{1u,4u}) {
        auto stock=sheet(160,315,mesh);require(sk_stock_set_threads(stock,threads)==SK_OK,"set threads");
        sk_snapshot_handle start{};require(sk_stock_snapshot(stock,&start)==SK_OK,"initial snapshot");auto initial=digest(stock);
        std::vector<sk_move_result> results(moves.size());sk_cut_summary summary{};
        require(sk_stock_cut(stock,tool,moves.data(),uint32_t(moves.size()),results.data(),&summary)==SK_OK,"cabinet cut");
        uint64_t hash=digest(stock);hash=hash_bytes(hash,results.data(),results.size()*sizeof(sk_move_result));
        if(threads==1)reference=hash;else require(hash==reference,"thread counts must be bit-identical");
        require(sk_stock_restore(stock,start)==SK_OK&&digest(stock)==initial,"snapshot survives dense materialisation");
        if(!mesh&&threads==1)box_reference=hash;
        std::printf("fixture %s threads=%u digest=%016llx\n",mesh?"mesh":"box",threads,(unsigned long long)hash);
        sk_snapshot_destroy(start);sk_stock_destroy(stock);
    }
    require(box_reference!=0,"nonempty reference digest");sk_tool_destroy(tool);
}
void full() {
    auto tool=cutter();
    for(double length:{2600.0,3150.0}) {
        auto stock=sheet(1600,length,true);auto before=info(stock);auto moves=cabinet(1600,length);
        sk_snapshot_handle start{};require(sk_stock_snapshot(stock,&start)==SK_OK,"full snapshot");
        require(sk_stock_set_threads(stock,4)==SK_OK,"full threads");std::vector<sk_move_result> results(moves.size());sk_cut_summary summary{};
        require(sk_stock_cut(stock,tool,moves.data(),uint32_t(moves.size()),results.data(),&summary)==SK_OK,"full cabinet cut");
        auto after=info(stock);require(after.bytes<300000000,"stock storage exceeds 300 MB");
        require(after.volume<before.volume&&summary.removed>0,"cabinet removes material");
        require(std::abs((before.volume-after.volume)-summary.removed)<1e-5,"removed volume agrees with remaining stock");
        sk_snapshot_handle cut{};require(sk_stock_snapshot(stock,&cut)==SK_OK,"cut snapshot");
        require(sk_stock_restore(stock,start)==SK_OK&&info(stock).volume==before.volume,"restore full sheet");
        require(sk_stock_restore(stock,cut)==SK_OK&&info(stock).volume==after.volume,"restore cabinet");
        sk_mesh_handle chunk{};require(sk_stock_mesh(stock,0,0,2,2,SK_MESH_CONTOUR,&chunk)==SK_OK,"tile contour from compact stock");sk_mesh_destroy(chunk);
        long peak=0;
#if defined(__linux__)
        rusage usage{};getrusage(RUSAGE_SELF,&usage);peak=usage.ru_maxrss*1024;

#endif
        std::printf("sheet y=%.0f spacing=0.5 moves=%zu initial_bytes=%llu cut_bytes=%llu peak_rss=%ld removed_mm3=%.9f\n",length,moves.size(),(unsigned long long)before.bytes,(unsigned long long)after.bytes,peak,summary.removed);
        require(peak<300000000,"peak RSS exceeds 300 MB");
        sk_snapshot_destroy(cut);sk_snapshot_destroy(start);sk_stock_destroy(stock);
    }
    sk_tool_destroy(tool);
}
}
int main(int argc,char **argv) { try {small();if(argc<2||std::string(argv[1])!="--small")full();return 0;}catch(const std::exception &e){std::fprintf(stderr,"FAIL: %s\n",e.what());return 1;} }
