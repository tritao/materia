#include "visionkit.h"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include "stb_image.h"
#include <algorithm>
#include <array>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
namespace fs=std::filesystem;
static std::string escape(const std::string &s) {
    static const char *hex="0123456789abcdef";
    std::string out="\"";
    for(unsigned char c:s) {
        if(c=='\"'||c=='\\') {out+='\\';out+=char(c);}
        else if(c<0x20) {out+="\\u00";out+=hex[c>>4];out+=hex[c&15];}
        else out+=char(c);
    }
    return out+'\"';
}
static void usage(){std::cerr<<"usage: visionkit_calibrate <image-dir> <output.json> <chessboard|charuco> <columns> <rows> <square-metres> [marker-metres] [min-coverage] [max-rms-px] [--dictionary=4x4_50|5x5_100|apriltag_36h11] [--legacy-pattern]\n";}
int main(int argc,char **argv){
    if(argc<7){usage();return 2;}
    try{
        fs::path directory=argv[1],output=argv[2];
        vk_board_spec board{};board.struct_size=sizeof(board);
        std::string kind=argv[3];board.kind=kind=="chessboard"?VK_BOARD_CHESSBOARD:kind=="charuco"?VK_BOARD_CHARUCO:0;
        if(board.kind==VK_BOARD_CHARUCO && argc<8){usage();return 2;}
        board.columns=std::stoul(argv[4]);board.rows=std::stoul(argv[5]);board.square_size_m=std::stod(argv[6]);
        board.marker_size_m=board.kind==VK_BOARD_CHARUCO&&argc>7?std::stod(argv[7]):0;
        board.dictionary=VK_ARUCO_4X4_50;
        int next=board.kind==VK_BOARD_CHARUCO?8:7;
        vk_calibration_limits limits{sizeof(vk_calibration_limits),0.35,1.5};
        int numeric_options=0;
        for(int i=next;i<argc;++i){
            const std::string option=argv[i];
            if(option=="--legacy-pattern")board.legacy_pattern=1;
            else if(option=="--dictionary=4x4_50")board.dictionary=VK_ARUCO_4X4_50;
            else if(option=="--dictionary=5x5_100")board.dictionary=VK_ARUCO_5X5_100;
            else if(option=="--dictionary=apriltag_36h11")board.dictionary=VK_APRILTAG_36H11;
            else if(option.rfind("--",0)==0){usage();return 2;}
            else if(numeric_options++==0)limits.minimum_coverage=std::stod(option);
            else if(numeric_options==2)limits.maximum_rms_pixels=std::stod(option);
            else {usage();return 2;}
        }
        if(!board.kind||!fs::is_directory(directory)){usage();return 2;}
        std::vector<fs::path> files;for(const auto &entry:fs::directory_iterator(directory)){
            auto ext=entry.path().extension().string();std::transform(ext.begin(),ext.end(),ext.begin(),::tolower);
            if(entry.is_regular_file()&&(ext==".png"||ext==".jpg"||ext==".jpeg"))files.push_back(entry.path());
        }std::sort(files.begin(),files.end());
        std::vector<std::vector<vk_board_corner>> corners;std::vector<std::string> names;
        uint32_t width=0,height=0;
        for(const auto &file:files){
            int w=0,h=0,ch=0;auto *data=stbi_load(file.string().c_str(),&w,&h,&ch,1);
            if(!data){std::cerr<<file.filename()<<": decode failed\n";continue;}
            if(width==0){width=w;height=h;}
            if(uint32_t(w)!=width||uint32_t(h)!=height){stbi_image_free(data);std::cerr<<file.filename()<<": dimensions differ\n";continue;}
            vk_image_view image{sizeof(vk_image_view),uint32_t(w),uint32_t(h),uint32_t(w),VK_PIXEL_GRAY8,uint32_t(w*h),data};
            uint32_t count=0;auto status=vk_board_detect(&image,&board,nullptr,0,&count);
            std::vector<vk_board_corner> found(count);
            if(status==VK_ERROR_LIMIT)status=vk_board_detect(&image,&board,found.data(),count,&count);
            stbi_image_free(data);
            std::cerr<<file.filename()<<": "<<count<<" corners\n";
            if(status==VK_OK&&count>=6){corners.push_back(std::move(found));names.push_back(file.filename().string());}
        }
        std::vector<vk_calibration_view> views;for(auto &c:corners)views.push_back({sizeof(vk_calibration_view),uint32_t(c.size()),c.data()});
        vk_camera_model model{};model.struct_size=sizeof(model);double rms=0;
        std::vector<vk_calibration_view_result> results(views.size());for(auto &v:results)v.struct_size=sizeof(v);
        auto status=vk_calibrate(&board,width,height,views.data(),views.size(),&limits,0,&model,&rms,results.data());
        if(status!=VK_OK){
            std::cerr<<"calibration rejected: "<<(status==VK_ERROR_CALIBRATION_COVERAGE?
                "insufficient image coverage":status==VK_ERROR_CALIBRATION_RMS?
                "RMS exceeds limit":"invalid or unsolvable views")<<" ("<<status<<")";
            if(status==VK_ERROR_CALIBRATION_RMS)
                std::cerr<<" measured="<<rms<<" px, limit="<<limits.maximum_rms_pixels<<" px";
            if(status==VK_ERROR_CALIBRATION_COVERAGE)
            {
                double minx=width,miny=height,maxx=0,maxy=0;
                std::array<uint32_t,16> visits{};
                for(const auto &view:corners){
                    std::array<bool,16> seen{};
                    for(const auto &corner:view){
                        minx=std::min(minx,corner.pixel.x);
                        miny=std::min(miny,corner.pixel.y);
                        maxx=std::max(maxx,corner.pixel.x);
                        maxy=std::max(maxy,corner.pixel.y);
                        const auto cell_x=std::min(3u,uint32_t(corner.pixel.x*4/width));
                        const auto cell_y=std::min(3u,uint32_t(corner.pixel.y*4/height));
                        seen[cell_y*4+cell_x]=true;
                    }
                    for(size_t cell=0;cell<16;++cell)if(seen[cell])++visits[cell];
                }
                const auto repeated=std::count_if(visits.begin(),visits.end(),
                    [](uint32_t count){return count>=2;});
                std::cerr<<" horizontal="<<(maxx-minx)/width<<", vertical="<<
                    (maxy-miny)/height<<", repeated cells="<<double(repeated)/16<<
                    ", required="<<limits.minimum_coverage;
            }
            std::cerr<<'\n';return 1;
        }
        for(size_t i=0;i<results.size();++i)std::cout<<names[i]<<": RMS "<<results[i].rms_reprojection_error<<" px, camera_T_board ("<<results[i].camera_T_board.x<<", "<<results[i].camera_T_board.y<<", "<<results[i].camera_T_board.z<<") m\n";
        std::time_t now=std::time(nullptr);char timestamp[32];std::strftime(timestamp,sizeof(timestamp),"%Y-%m-%dT%H:%M:%SZ",std::gmtime(&now));
        std::string description=kind+" "+std::to_string(board.columns)+"x"+
            std::to_string(board.rows)+" square="+std::to_string(board.square_size_m)+"m";
        if(board.kind==VK_BOARD_CHARUCO)description+=" marker="+
            std::to_string(board.marker_size_m)+"m dictionary="+
            std::to_string(board.dictionary)+" legacy="+std::to_string(board.legacy_pattern);
        std::ostringstream json;json<<std::setprecision(17)<<"{\n  \"version\":1,\n  \"cameraModel\":{\"width\":"<<model.width<<",\"height\":"<<model.height<<",\"fx\":"<<model.fx<<",\"fy\":"<<model.fy<<",\"cx\":"<<model.cx<<",\"cy\":"<<model.cy<<",\"distortionModel\":"<<model.distortion_model<<",\"k1\":"<<model.k1<<",\"k2\":"<<model.k2<<",\"p1\":"<<model.p1<<",\"p2\":"<<model.p2<<",\"k3\":"<<model.k3<<"},\n  \"rmsReprojectionError\":"<<rms<<",\n  \"calibrationTime\":"<<escape(timestamp)<<",\n  \"boardDescription\":"<<escape(description)<<",\n  \"source\":"<<escape(fs::absolute(directory).string())<<"\n}\n";
        std::ofstream stream(output);if(!stream){std::cerr<<"cannot write "<<output<<"\n";return 1;}stream<<json.str();if(!stream){std::cerr<<"write failed\n";return 1;}
        std::cout<<"RMS "<<rms<<" px; wrote "<<output<<"\n";return 0;
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
}
