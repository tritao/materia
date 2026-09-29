#include "visionkit.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#ifdef VK_HAVE_STB
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#endif
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <vector>
namespace fs=std::filesystem;
static vk_board_spec board(uint32_t kind){return {sizeof(vk_board_spec),kind,8,6,0.04,0.025,VK_ARUCO_4X4_50};}
static std::vector<vk_board_corner> projected(const vk_board_spec &b,int view,bool noisy=false){
    std::vector<cv::Point3f> object;
    if(b.kind==VK_BOARD_CHESSBOARD)for(uint32_t y=0;y<b.rows;++y)for(uint32_t x=0;x<b.columns;++x)object.emplace_back(x*b.square_size_m,y*b.square_size_m,0);
    else {cv::aruco::CharucoBoard native(cv::Size(b.columns,b.rows),b.square_size_m,b.marker_size_m,cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));for(auto &p:native.getChessboardCorners())object.emplace_back(p.x,p.y,0);}
    cv::Mat k=(cv::Mat_<double>(3,3)<<750,0,640,0,740,480,0,0,1);
    cv::Mat d=(cv::Mat_<double>(1,5)<<0.04,-0.015,0.001,-0.002,0.003);
    cv::Matx33d basis=cv::Matx33d::eye();
    cv::Mat rvec;cv::Rodrigues(cv::Mat(basis),rvec);
    rvec.at<double>(0)+=0.28*std::sin(view*1.3);rvec.at<double>(1)+=0.32*std::cos(view*0.9);
    cv::Vec3d t(-0.30+0.15*(view%5),-0.20+0.15*(view/5),1.10+0.06*(view%3));
    std::vector<cv::Point2f> pixels;cv::projectPoints(object,rvec,t,k,d,pixels);
    std::vector<vk_board_corner> out;for(size_t i=0;i<pixels.size();++i)out.push_back({uint32_t(i),{pixels[i].x+(noisy?4*std::sin(i*17+view):0),pixels[i].y+(noisy?4*std::cos(i*13+view):0)}});
    return out;
}
int main(int argc,char **argv){
    for(auto kind:{VK_BOARD_CHESSBOARD,VK_BOARD_CHARUCO}){
        auto b=board(kind);std::vector<std::vector<vk_board_corner>> points;
        for(int i=0;i<15;++i)points.push_back(projected(b,i));
        std::vector<vk_calibration_view> views;std::vector<vk_calibration_view_result> results(points.size());
        for(size_t i=0;i<points.size();++i){views.push_back({sizeof(vk_calibration_view),uint32_t(points[i].size()),points[i].data()});results[i].struct_size=sizeof(vk_calibration_view_result);}
        vk_camera_model model{};model.struct_size=sizeof(model);double rms=0;
        vk_calibration_limits limits{sizeof(limits),0.1,1.0};
        auto status=vk_calibrate(&b,1280,960,views.data(),views.size(),&limits,0,&model,&rms,results.data());
        assert(vk_opencv_threads()==VK_OPENCV_THREADS);
        if(status!=VK_OK)fprintf(stderr,"kind %u status %d\n",kind,status);assert(status==VK_OK);assert(rms<0.01);assert(std::abs(model.fx-750)<5);assert(std::abs(model.fy-740)<5);
        assert(std::abs(model.cx-640)<5);assert(std::abs(model.cy-480)<5);
        assert(std::abs(model.k1-0.04)<0.01);
        assert(std::abs(model.k2+0.015)<0.03);
        assert(std::abs(model.p1-0.001)<0.005);
        assert(std::abs(model.p2+0.002)<0.005);
        assert(std::abs(model.k3-0.003)<0.01);
        for(auto &v:results){assert(v.rms_reprojection_error<0.01);assert(std::isfinite(v.camera_T_board.qw));}
        assert(std::abs(results[0].camera_T_board.x-1.1)<0.02);
        assert(std::abs(results[0].camera_T_board.y-0.30)<0.02);
        limits.minimum_coverage=0.95;
        assert(vk_calibrate(&b,1280,960,views.data(),views.size(),&limits,0,&model,&rms,results.data())==VK_ERROR_CALIBRATION_COVERAGE);
        limits.minimum_coverage=0;limits.maximum_rms_pixels=0.000001;
        assert(vk_calibrate(&b,1280,960,views.data(),views.size(),&limits,0,&model,&rms,results.data())==VK_ERROR_CALIBRATION_RMS);
        assert(rms>limits.maximum_rms_pixels);
        limits.maximum_rms_pixels=1;
        assert(vk_calibrate(&b,1280,960,views.data(),4,&limits,0,&model,&rms,results.data())==VK_ERROR_INVALID_ARGUMENT);
        assert(vk_calibrate(&b,1280,960,views.data(),views.size(),&limits,4,&model,&rms,results.data())==VK_ERROR_INVALID_ARGUMENT);
        auto duplicate=points[0][1]; points[0][1]=points[0][0];
        assert(vk_calibrate(&b,1280,960,views.data(),views.size(),&limits,0,&model,&rms,results.data())==VK_ERROR_INVALID_ARGUMENT);
        points[0][1]=duplicate;
        auto sparse=points;
        for(size_t i=1;i<sparse.size();++i)for(auto &corner:sparse[i])
            corner.pixel=points[0][corner.id].pixel;
        for(size_t i=0;i<sparse.size();++i) views[i].corners=sparse[i].data();
        limits.minimum_coverage=0.2;
        assert(vk_calibrate(&b,1280,960,views.data(),views.size(),&limits,0,&model,&rms,results.data())==VK_ERROR_CALIBRATION_COVERAGE);
    }
    // Detection from generated fronto-parallel boards.
    for(auto kind:{VK_BOARD_CHESSBOARD,VK_BOARD_CHARUCO}){
        auto b=board(kind);cv::Mat image(600,800,CV_8UC1,cv::Scalar(255));cv::Mat pattern;
        if(kind==VK_BOARD_CHESSBOARD){pattern=cv::Mat(420,560,CV_8UC1);for(int y=0;y<420;++y)for(int x=0;x<560;++x)pattern.at<uint8_t>(y,x)=((x/70+y/70)%2)?255:0;b.columns=7;b.rows=5;}
        else{cv::aruco::CharucoBoard native(cv::Size(b.columns,b.rows),b.square_size_m,b.marker_size_m,cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));native.generateImage(cv::Size(560,420),pattern);}
        pattern.copyTo(image(cv::Rect(120,90,560,420)));
        vk_image_view view{sizeof(view),800,600,800,VK_PIXEL_GRAY8,800*600,image.data};uint32_t count=0;
        assert(vk_board_detect(&view,&b,nullptr,0,&count)==VK_ERROR_LIMIT);assert(count>=6);
        std::vector<vk_board_corner> found(count);assert(vk_board_detect(&view,&b,found.data(),count,&count)==VK_OK);
        if(kind==VK_BOARD_CHARUCO){
            std::vector<std::vector<vk_board_corner>> detected;
            cv::Point2f src[4]={{120,90},{680,90},{680,510},{120,510}};
            for(int i=0;i<12;++i){
                const float tilt=float(i-5);
                cv::Point2f dst[4]={{120+tilt*7,90+tilt*5},{680-tilt*6,90-tilt*4},
                    {680+tilt*4,510+tilt*6},{120-tilt*5,510-tilt*3}};
                cv::Mat warped;cv::warpPerspective(image,warped,
                    cv::getPerspectiveTransform(src,dst),image.size(),
                    cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255));
                auto warped_view=view;warped_view.data=warped.data;
                uint32_t n=0;auto detected_status=vk_board_detect(&warped_view,&b,nullptr,0,&n);
                if(detected_status!=VK_ERROR_LIMIT||n<6)continue;
                std::vector<vk_board_corner> corners(n);
                assert(vk_board_detect(&warped_view,&b,corners.data(),n,&n)==VK_OK);
                detected.push_back(std::move(corners));
            }
            assert(detected.size()>=5);
            std::vector<vk_calibration_view> input;
            std::vector<vk_calibration_view_result> output(detected.size());
            for(size_t i=0;i<detected.size();++i){
                input.push_back({sizeof(vk_calibration_view),uint32_t(detected[i].size()),detected[i].data()});
                output[i].struct_size=sizeof(vk_calibration_view_result);
            }
            vk_camera_model model{};model.struct_size=sizeof(model);double rms=0;
            vk_calibration_limits limits{sizeof(limits),0.05,10};
            assert(vk_calibrate(&b,800,600,input.data(),input.size(),&limits,0,
                &model,&rms,output.data())==VK_OK);
            assert(std::isfinite(model.fx)&&model.fx>0&&std::isfinite(rms));
        }
#ifdef VK_HAVE_STB
        if(kind==VK_BOARD_CHESSBOARD && argc>1){
            fs::path dir=fs::path(argv[1]).parent_path()/"visionkit-cli-fixture";fs::create_directories(dir);
            cv::Point2f src[4]={{120,90},{680,90},{680,510},{120,510}};
            for(int i=0;i<9;++i){
                const float tilt=float(i-4);
                cv::Point2f dst[4]={{120+tilt*6,90+tilt*4},{680-tilt*5,90-tilt*3},
                    {680+tilt*4,510+tilt*5},{120-tilt*3,510-tilt*4}};
                cv::Mat warped;cv::warpPerspective(image,warped,cv::getPerspectiveTransform(src,dst),
                    image.size(),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(255));
                auto p=dir/(std::to_string(i)+".png");
                assert(stbi_write_png(p.string().c_str(),800,600,1,warped.data,800));
            }
            std::string command=std::string("\"")+argv[1]+"\" \""+dir.string()+"\" \""+
                (dir/"calibration.json").string()+"\" chessboard 7 5 0.04 0.1 1.5 > /dev/null";
            assert(std::system(command.c_str())==0);
            std::ifstream json(dir/"calibration.json");assert(json.good());
            const std::string contents((std::istreambuf_iterator<char>(json)),{});
            auto numeric=[&](const std::string &key) {
                const auto at=contents.find("\""+key+"\":");
                assert(at!=std::string::npos);
                return std::stod(contents.substr(at+key.size()+3));
            };
            assert(numeric("version")==1);
            assert(std::isfinite(numeric("fx")) && numeric("fx")>0);
            assert(std::isfinite(numeric("rmsReprojectionError")) &&
                numeric("rmsReprojectionError")<1.5);
            fs::remove_all(dir);
        }
#endif
    }
}
