#include "visionkit.h"
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include "stb_image.h"
#include <algorithm>
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
static void usage(){std::cerr<<"usage: visionkit_calibrate <image-dir> <output.json> <chessboard|charuco> <columns> <rows> <square-metres> [marker-metres] [min-coverage] [max-rms-px]\n";}
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
        vk_calibration_limits limits{sizeof(vk_calibration_limits),argc>next?std::stod(argv[next]):0.35,
            argc>next+1?std::stod(argv[next+1]):1.5};
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
        if(status!=VK_OK){std::cerr<<"calibration rejected: "<<(status==VK_ERROR_CALIBRATION_COVERAGE?"insufficient image coverage":status==VK_ERROR_CALIBRATION_RMS?"RMS exceeds limit":"invalid or unsolvable views")<<" ("<<status<<")\n";return 1;}
        for(size_t i=0;i<results.size();++i)std::cout<<names[i]<<": RMS "<<results[i].rms_reprojection_error<<" px, camera_T_board ("<<results[i].camera_T_board.x<<", "<<results[i].camera_T_board.y<<", "<<results[i].camera_T_board.z<<") m\n";
        std::time_t now=std::time(nullptr);char timestamp[32];std::strftime(timestamp,sizeof(timestamp),"%Y-%m-%dT%H:%M:%SZ",std::gmtime(&now));
        std::string description=kind+" "+std::to_string(board.columns)+"x"+
            std::to_string(board.rows)+" square="+std::to_string(board.square_size_m)+"m";
        if(board.kind==VK_BOARD_CHARUCO)description+=" marker="+
            std::to_string(board.marker_size_m)+"m DICT_4X4_50";
        std::ostringstream json;json<<std::setprecision(17)<<"{\n  \"version\":1,\n  \"cameraModel\":{\"width\":"<<model.width<<",\"height\":"<<model.height<<",\"fx\":"<<model.fx<<",\"fy\":"<<model.fy<<",\"cx\":"<<model.cx<<",\"cy\":"<<model.cy<<",\"distortionModel\":"<<model.distortion_model<<",\"k1\":"<<model.k1<<",\"k2\":"<<model.k2<<",\"p1\":"<<model.p1<<",\"p2\":"<<model.p2<<",\"k3\":"<<model.k3<<"},\n  \"rmsReprojectionError\":"<<rms<<",\n  \"calibrationTime\":"<<escape(timestamp)<<",\n  \"boardDescription\":"<<escape(description)<<",\n  \"source\":"<<escape(fs::absolute(directory).string())<<"\n}\n";
        std::ofstream stream(output);if(!stream){std::cerr<<"cannot write "<<output<<"\n";return 1;}stream<<json.str();if(!stream){std::cerr<<"write failed\n";return 1;}
        std::cout<<"RMS "<<rms<<" px; wrote "<<output<<"\n";return 0;
    }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 2;}
}
