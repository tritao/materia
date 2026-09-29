#include "visionkit.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <algorithm>
#include <cmath>
#include <climits>
#include <vector>

namespace {
bool valid_board(const vk_board_spec *b) {
    return b && b->struct_size >= sizeof(*b) && b->columns >= 3 && b->rows >= 3 &&
        b->columns <= 100 && b->rows <= 100 && std::isfinite(b->square_size_m) &&
        b->square_size_m > 0 && (b->kind == VK_BOARD_CHESSBOARD ||
        (b->kind == VK_BOARD_CHARUCO && std::isfinite(b->marker_size_m) &&
        b->marker_size_m > 0 && b->marker_size_m < b->square_size_m &&
        (b->dictionary == VK_ARUCO_4X4_50 || b->dictionary == VK_ARUCO_5X5_100 ||
         b->dictionary == VK_APRILTAG_36H11)));
}
cv::aruco::PredefinedDictionaryType dictionary(uint32_t d) {
    if (d == VK_ARUCO_5X5_100) return cv::aruco::DICT_5X5_100;
    if (d == VK_APRILTAG_36H11) return cv::aruco::DICT_APRILTAG_36h11;
    return cv::aruco::DICT_4X4_50;
}
cv::aruco::CharucoBoard make_board(const vk_board_spec &b) {
    return cv::aruco::CharucoBoard(cv::Size(b.columns,b.rows), float(b.square_size_m),
        float(b.marker_size_m), cv::aruco::getPredefinedDictionary(dictionary(b.dictionary)));
}
uint32_t corner_count(const vk_board_spec &b) {
    return b.kind == VK_BOARD_CHESSBOARD ? b.columns*b.rows : (b.columns-1)*(b.rows-1);
}
bool valid_image(const vk_image_view *v) {
    if (!v || v->struct_size < sizeof(*v) || !v->data || !v->width || !v->height ||
        v->width >= 32767 || v->height >= 32767 ||
        (v->pixel_format != VK_PIXEL_GRAY8 && v->pixel_format != VK_PIXEL_RGB8)) return false;
    const uint64_t row = uint64_t(v->width)*(v->pixel_format == VK_PIXEL_RGB8 ? 3 : 1);
    return v->stride_bytes >= row && uint64_t(v->stride_bytes)*(v->height-1)+row <= v->buffer_bytes;
}
vk_pose3 pose(const cv::Mat &rv, const cv::Mat &tv) {
    cv::Mat rotation; cv::Rodrigues(rv,rotation);
    const cv::Matx33d basis(0,0,1,-1,0,0,0,-1,0);
    const cv::Matx33d board_basis(0,-1,0,0,0,-1,1,0,0);
    const cv::Matx33d r = basis*cv::Matx33d(rotation)*board_basis;
    const cv::Vec3d t = basis*cv::Vec3d(tv);
    vk_pose3 out{sizeof(vk_pose3),t[0],t[1],t[2],0,0,0,1};
    const double trace=r(0,0)+r(1,1)+r(2,2);
    if(trace>0){double s=2*sqrt(trace+1);out.qw=s/4;out.qx=(r(2,1)-r(1,2))/s;out.qy=(r(0,2)-r(2,0))/s;out.qz=(r(1,0)-r(0,1))/s;}
    else if(r(0,0)>r(1,1)&&r(0,0)>r(2,2)){double s=2*sqrt(1+r(0,0)-r(1,1)-r(2,2));out.qw=(r(2,1)-r(1,2))/s;out.qx=s/4;out.qy=(r(0,1)+r(1,0))/s;out.qz=(r(0,2)+r(2,0))/s;}
    else if(r(1,1)>r(2,2)){double s=2*sqrt(1+r(1,1)-r(0,0)-r(2,2));out.qw=(r(0,2)-r(2,0))/s;out.qx=(r(0,1)+r(1,0))/s;out.qy=s/4;out.qz=(r(1,2)+r(2,1))/s;}
    else {double s=2*sqrt(1+r(2,2)-r(0,0)-r(1,1));out.qw=(r(1,0)-r(0,1))/s;out.qx=(r(0,2)+r(2,0))/s;out.qy=(r(1,2)+r(2,1))/s;out.qz=s/4;}
    return out;
}
}
extern "C" VK_API vk_result vk_board_detect(const vk_image_view *image,const vk_board_spec *board,
    vk_board_corner *out_corners,uint32_t capacity,uint32_t *out_count) {
    if (!valid_image(image)||!valid_board(board)||!out_count||(capacity&&!out_corners)) return VK_ERROR_INVALID_ARGUMENT;
    try {
        vk_version();
        cv::Mat raw(image->height,image->width,image->pixel_format==VK_PIXEL_RGB8?CV_8UC3:CV_8UC1,image->data,image->stride_bytes),gray;
        if(image->pixel_format==VK_PIXEL_RGB8) cv::cvtColor(raw,gray,cv::COLOR_RGB2GRAY); else gray=raw;
        std::vector<cv::Point2f> points;std::vector<int> ids;
        if(board->kind==VK_BOARD_CHESSBOARD) {
            if(cv::findChessboardCorners(gray,cv::Size(board->columns,board->rows),points,
                cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE)) {
                cv::cornerSubPix(gray,points,cv::Size(5,5),cv::Size(-1,-1),
                    cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,40,0.001));
                for(size_t i=0;i<points.size();++i) ids.push_back(int(i));
            }
        } else {
            cv::aruco::CharucoDetector detector(make_board(*board));
            detector.detectBoard(gray,points,ids);
            if(!points.empty()) cv::cornerSubPix(gray,points,cv::Size(3,3),cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.001));
        }
        *out_count=uint32_t(points.size());
        if(points.size()>capacity) return VK_ERROR_LIMIT;
        for(size_t i=0;i<points.size();++i) out_corners[i]={uint32_t(ids[i]),{points[i].x,points[i].y}};
        return VK_OK;
    } catch(const std::bad_alloc&){return VK_ERROR_OUT_OF_MEMORY;} catch(const cv::Exception&){return VK_ERROR_BACKEND;}
}
extern "C" VK_API vk_result vk_calibrate(const vk_board_spec *board,uint32_t width,uint32_t height,
    const vk_calibration_view *views,uint32_t view_count,const vk_calibration_limits *limits,
    uint32_t flags,vk_camera_model *out_model,double *out_rms,vk_calibration_view_result *out_views) {
    if((flags & ~3u)!=0||!valid_board(board)||!width||!height||width>=32767||height>=32767||!views||view_count<5||
       view_count>INT_MAX||!limits||limits->struct_size<sizeof(*limits)||
       !std::isfinite(limits->minimum_coverage)||limits->minimum_coverage<0||limits->minimum_coverage>1||
       !std::isfinite(limits->maximum_rms_pixels)||limits->maximum_rms_pixels<=0||
       !out_model||out_model->struct_size<sizeof(*out_model)||!out_rms||!out_views) return VK_ERROR_INVALID_ARGUMENT;
    try {
        vk_version();
        std::vector<cv::Point3f> board_points;
        if(board->kind==VK_BOARD_CHESSBOARD) {
            board_points.reserve(corner_count(*board));
            for(uint32_t row=0;row<board->rows;++row)
                for(uint32_t col=0;col<board->columns;++col)
                    board_points.emplace_back(float(col*board->square_size_m),
                                              float(row*board->square_size_m),0);
        } else {
            board_points=make_board(*board).getChessboardCorners();
        }
        std::vector<std::vector<cv::Point3f>> objects(view_count);
        std::vector<std::vector<cv::Point2f>> pixels(view_count);
        double minx=width,miny=height,maxx=0,maxy=0;
        for(uint32_t i=0;i<view_count;++i){
            const auto &v=views[i];
            if(v.struct_size<sizeof(v)||!v.corners||v.corner_count<6||v.corner_count>corner_count(*board)||
               out_views[i].struct_size<sizeof(vk_calibration_view_result)) return VK_ERROR_INVALID_ARGUMENT;
            std::vector<bool> seen(corner_count(*board),false);
            for(uint32_t j=0;j<v.corner_count;++j){
                const auto &c=v.corners[j];
                if(c.id>=seen.size()||seen[c.id]||!std::isfinite(c.pixel.x)||!std::isfinite(c.pixel.y)||
                   c.pixel.x<0||c.pixel.x>=width||c.pixel.y<0||c.pixel.y>=height) return VK_ERROR_INVALID_ARGUMENT;
                seen[c.id]=true;objects[i].push_back(board_points[c.id]);
                pixels[i].emplace_back(float(c.pixel.x),float(c.pixel.y));
                minx=std::min(minx,c.pixel.x);miny=std::min(miny,c.pixel.y);
                maxx=std::max(maxx,c.pixel.x);maxy=std::max(maxy,c.pixel.y);
            }
        }
        if((maxx-minx)/width<limits->minimum_coverage||(maxy-miny)/height<limits->minimum_coverage)
            return VK_ERROR_CALIBRATION_COVERAGE;
        cv::Mat k=cv::Mat::eye(3,3,CV_64F),d,r,t;
        std::vector<cv::Mat> rvecs,tvecs;
        if(flags & VK_CALIB_FIX_PRINCIPAL_POINT){k.at<double>(0,2)=(width-1)/2.0;k.at<double>(1,2)=(height-1)/2.0;}
        int cvflags=cv::CALIB_FIX_K4|cv::CALIB_FIX_K5|cv::CALIB_FIX_K6;
        if(flags & VK_CALIB_ZERO_TANGENT_DIST) cvflags|=cv::CALIB_ZERO_TANGENT_DIST;
        if(flags & VK_CALIB_FIX_PRINCIPAL_POINT) cvflags|=cv::CALIB_FIX_PRINCIPAL_POINT;
        double rms=cv::calibrateCamera(objects,pixels,cv::Size(width,height),k,d,rvecs,tvecs,cvflags);
        if(!std::isfinite(rms)) return VK_ERROR_BACKEND;
        if(rms>limits->maximum_rms_pixels) return VK_ERROR_CALIBRATION_RMS;
        vk_camera_model model{sizeof(vk_camera_model),width,height,k.at<double>(0,0),k.at<double>(1,1),
            k.at<double>(0,2),k.at<double>(1,2),VK_DISTORTION_PLUMB_BOB,
            d.at<double>(0),d.at<double>(1),d.at<double>(2),d.at<double>(3),d.at<double>(4)};
        if(!std::isfinite(model.fx)||!std::isfinite(model.fy)||model.fx<=0||model.fy<=0) return VK_ERROR_BACKEND;
        std::vector<vk_calibration_view_result> results(view_count);
        for(uint32_t i=0;i<view_count;++i){
            std::vector<cv::Point2f> projected;
            cv::projectPoints(objects[i],rvecs[i],tvecs[i],k,d,projected);
            double sq=0;for(size_t j=0;j<projected.size();++j){auto q=projected[j]-pixels[i][j];sq+=q.dot(q);}
            results[i]={sizeof(vk_calibration_view_result),sqrt(sq/projected.size()),pose(rvecs[i],tvecs[i])};
        }
        *out_model=model;*out_rms=rms;
        for(uint32_t i=0;i<view_count;++i)out_views[i]=results[i];
        return VK_OK;
    } catch(const std::bad_alloc&){return VK_ERROR_OUT_OF_MEMORY;} catch(const cv::Exception&){return VK_ERROR_BACKEND;}
}
