#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip> // For std::setprecision
#include <numeric> // For std::accumulate
#include <algorithm> // For std::sort, std::min, std::max

// =======================================================
// 全局参数和变量
// =======================================================

// 视频路径
const std::string VIDEO_PATH = "D:/BaiduSyncdisk/200219.avi"; // <-- 请修改为您的视频路径

// ROI 选择完成后，将计算并存储此矩形 (用于裁剪视频帧)
cv::Rect fixed_roi_rect;
// 用户选择的梯形ROI的原始坐标 (相对于完整视频帧)
std::vector<cv::Point> trapezoid_points; // 这个向量将存储规范化后的4个梯形点
// ROI核心区域的收缩比例（用于计数）
const double CORE_REGION_SHRINK_FACTOR = 0.8; // 例如，缩小到原来尺寸的80%

// 轮廓面积过滤阈值
const double AREA_MIN = 20.0; // 最小面积阈值，防止噪声
const double AREA_MAX = 500.0; // 最大面积阈值，防止大块干扰物

// 追踪相关参数
const double MAX_DIST_SQ = 100.0 * 100.0; // 追踪对象质心距离平方最大值
const int MAX_LOST_FRAMES = 5;            // 最大连续丢失帧数
const int MAX_TRACK_HISTORY = 300;        // 最大追踪历史帧数 (用于显示面积图)

// === 新增结构体来封装 BGR 阈值参数 ===
struct BgrGreenThresholds {
    int g_threshold;
    int r_max;
    int b_max;
    int g_r_diff_min;
    int g_b_diff_min;
};

// 全局 Trackbar 绑定的 int 变量 (保持不变，因为 Trackbar 接口需要 int*)
int g_threshold_trackbar_val = 70;
int r_max_trackbar_val = 50;
int b_max_trackbar_val = 50;
int g_r_diff_min_trackbar_val = 20;
int g_b_diff_min_trackbar_val = 20;

// =======================================================
// 结构体和辅助工具函数
// =======================================================

// 用于追踪每个独立线粒体的结构
struct TrackedObject {
    int id;               // 唯一ID
    cv::Point2f centroid; // 当前质心
    std::vector<cv::Point2f> history_centroids; // 历史质心（用于追踪路径可视化，或平滑）
    int lost_frames;      // 连续丢失帧数
    bool counted;         // 是否已被计数过 (首次进入核心区域时设置为true)
    bool counted_in_current_frame; // 是否在当前帧被识别为“新计数”
    std::vector<cv::Point> contour; // 当前帧的轮廓 (用于面积计算和绘制)

    TrackedObject(int i, cv::Point2f c, const std::vector<cv::Point>& cont) :
        id(i), centroid(c), lost_frames(0), counted(false), counted_in_current_frame(false), contour(cont) {
        history_centroids.push_back(c);
        if (contour.empty()) {
            std::cerr << "警告：TrackedObject #" << id << " 以空轮廓创建或更新！" << std::endl;
        }
    }
};

// 当前线粒体ID计数器
int next_object_id = 0;
// 所有被追踪的线粒体列表
std::vector<TrackedObject> trackedObjects;
// 面积历史记录，用于绘制面积曲线图
std::vector<double> area_history;

// 计算两个质心之间的欧氏距离平方
double centroidDistSq(const cv::Point2f& p1, const cv::Point2f& p2) {
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 计算梯形ROI核心区域（缩小后），返回的是相对于原始视频帧的绝对坐标
cv::Rect computeCoreRegion(const cv::Rect& roi_rect_parent_frame, double shrink_factor) {
    int width = roi_rect_parent_frame.width;
    int height = roi_rect_parent_frame.height;

    int core_width = static_cast<int>(width * shrink_factor);
    int core_height = static_cast<int>(height * shrink_factor);

    int core_x = roi_rect_parent_frame.x + (width - core_width) / 2;
    int core_y = roi_rect_parent_frame.y + (height - core_height) / 2;

    return cv::Rect(core_x, core_y, core_width, core_height);
}

// 将绝对ROI点转换为相对于裁剪后ROI图像的相对点
std::vector<cv::Point> getRelativeTrapezoidPoints(const std::vector<cv::Point>& abs_points, const cv::Rect& roi_rect) {
    std::vector<cv::Point> relative_points(abs_points.size());
    for (size_t i = 0; i < abs_points.size(); ++i) {
        relative_points[i] = abs_points[i] - roi_rect.tl();
    }
    return relative_points;
}

// 绘制面积历史曲线图
cv::Mat generateAreaPlot(const std::vector<double>& history, int width, int height) {
    cv::Mat plot_image = cv::Mat::zeros(height, width, CV_8UC3);
    if (history.empty()) return plot_image;

    double max_area = 0.0;
    for (double area : history) {
        max_area = std::max(max_area, area);
    }

    if (max_area < 1.0) max_area = 1.0;

    double x_scale = static_cast<double>(width) / MAX_TRACK_HISTORY;
    double y_scale = static_cast<double>(height) / (max_area * 1.1);

    cv::line(plot_image, cv::Point(0, height - 1), cv::Point(width - 1, height - 1), cv::Scalar(255, 255, 255), 1);
    cv::line(plot_image, cv::Point(0, 0), cv::Point(0, height - 1), cv::Scalar(255, 255, 255), 1);

    for (size_t i = 0; i < history.size(); ++i) {
        if (i > 0) {
            cv::Point p1(static_cast<int>((i - 1) * x_scale), height - 1 - static_cast<int>(history[i - 1] * y_scale));
            cv::Point p2(static_cast<int>(i * x_scale), height - 1 - static_cast<int>(history[i] * y_scale));

            p1.x = std::max(0, std::min(width - 1, p1.x));
            p1.y = std::max(0, std::min(height - 1, p1.y));
            p2.x = std::max(0, std::min(width - 1, p2.x));
            p2.y = std::max(0, std::min(height - 1, p2.y));

            cv::line(plot_image, p1, p2, cv::Scalar(0, 255, 0), 1);
        }
    }
    return plot_image;
}

// =======================================================
// 图形界面和ROI选择回调函数
// =======================================================

void mouseCallback(int event, int x, int y, int flags, void* userdata) {
    cv::Mat* img = static_cast<cv::Mat*>(userdata);

    if (event == cv::EVENT_LBUTTONDOWN) {
        if (trapezoid_points.size() < 4) {
            trapezoid_points.push_back(cv::Point(x, y));
            cv::circle(*img, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), cv::FILLED);
            if (trapezoid_points.size() > 1) {
                cv::line(*img, trapezoid_points.at(trapezoid_points.size() - 2), trapezoid_points.back(), cv::Scalar(0, 255, 0), 1);
            }
            cv::imshow("点击选择4点构成ROI", *img);
        }
        if (trapezoid_points.size() == 4) {
            cv::line(*img, trapezoid_points.back(), trapezoid_points.front(), cv::Scalar(0, 255, 0), 1);
            cv::imshow("点击选择4点构成ROI", *img);
            std::cout << "ROI selection complete. Press any key to continue." << std::endl;
        }
    }
}

// =======================================================
// 图像处理函数 (BGR空间下的绿色提取)
// =======================================================

// 辅助函数：应用形态学操作
cv::Mat applyMorphology(const cv::Mat& binary_mask) {
    cv::Mat processed_mask = binary_mask.clone();
    cv::Mat kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    cv::morphologyEx(processed_mask, processed_mask, cv::MORPH_OPEN, kernel_open);
    cv::morphologyEx(processed_mask, processed_mask, cv::MORPH_CLOSE, kernel_close);

    return processed_mask;
}

// 核心功能：在 BGR 空间中提取绿色轮廓
// 确保返回的轮廓是有效的，即包含点的轮廓
std::vector<std::vector<cv::Point>> extractGreenContoursBGR(
    const cv::Mat& bgr_roi_image,
    const cv::Mat& trapezoid_mask,
    double min_area,
    double max_area,
    cv::Mat& output_mask_display,
    const BgrGreenThresholds& thresholds
) {
    cv::Mat bgr_green_mask = cv::Mat::zeros(bgr_roi_image.size(), CV_8UC1);

    for (int y = 0; y < bgr_roi_image.rows; ++y) {
        for (int x = 0; x < bgr_roi_image.cols; ++x) {
            cv::Vec3b pixel = bgr_roi_image.at<cv::Vec3b>(y, x);
            uchar B = pixel[0];
            uchar G = pixel[1];
            uchar R = pixel[2];

            if (G >= thresholds.g_threshold &&
                R <= thresholds.r_max &&
                B <= thresholds.b_max &&
                (G - R) >= thresholds.g_r_diff_min &&
                (G - B) >= thresholds.g_b_diff_min)
            {
                bgr_green_mask.at<uchar>(y, x) = 255;
            }
        }
    }

    cv::bitwise_and(bgr_green_mask, trapezoid_mask, bgr_green_mask);

    output_mask_display = applyMorphology(bgr_green_mask);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(output_mask_display, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> filtered_contours;
    for (const auto& contour : contours) {
        // 过滤空轮廓和点过少的轮廓
        if (contour.empty() || contour.size() < 3) { // 轮廓点至少3个才能构成有效区域
            continue;
        }

        double area = cv::contourArea(contour);
        if (area >= min_area && area <= max_area) {
            filtered_contours.push_back(contour);
        }
    }

    return filtered_contours;
}

// =======================================================
// 追踪逻辑
// =======================================================
void updateTrackedObjects(const std::vector<std::pair<cv::Point2f, std::vector<cv::Point>>>& detected_objects_with_contours, const cv::Rect& core_region_relative_roi, const cv::Mat& trapezoid_mask_for_check) {
    for (auto& obj : trackedObjects) {
        obj.counted_in_current_frame = false;
    }

    std::vector<bool> matched_detected(detected_objects_with_contours.size(), false);

    for (size_t i = 0; i < trackedObjects.size(); ++i) {
        double min_dist_sq = MAX_DIST_SQ;
        int best_match_idx = -1;

        for (size_t j = 0; j < detected_objects_with_contours.size(); ++j) {
            if (matched_detected[j]) continue;

            double dist_sq = centroidDistSq(trackedObjects[i].centroid, detected_objects_with_contours[j].first);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_match_idx = j;
            }
        }

        if (best_match_idx != -1) {
            // 更新前的额外检查：确保 incoming contour 不为空且点数足够
            if (!detected_objects_with_contours[best_match_idx].second.empty() &&
                detected_objects_with_contours[best_match_idx].second.size() >= 3)
            {
                trackedObjects[i].centroid = detected_objects_with_contours[best_match_idx].first;
                trackedObjects[i].contour = detected_objects_with_contours[best_match_idx].second;
                trackedObjects[i].history_centroids.push_back(trackedObjects[i].centroid);
                trackedObjects[i].lost_frames = 0;
                matched_detected[best_match_idx] = true;

                if (trackedObjects[i].centroid.x >= 0 && trackedObjects[i].centroid.x < trapezoid_mask_for_check.cols &&
                    trackedObjects[i].centroid.y >= 0 && trackedObjects[i].centroid.y < trapezoid_mask_for_check.rows &&
                    core_region_relative_roi.contains(trackedObjects[i].centroid) &&
                    trapezoid_mask_for_check.at<uchar>(static_cast<int>(trackedObjects[i].centroid.y),
                        static_cast<int>(trackedObjects[i].centroid.x)) == 255 &&
                    !trackedObjects[i].counted) {
                    trackedObjects[i].counted = true;
                    trackedObjects[i].counted_in_current_frame = true;
                }
            }
            else {
                // 如果匹配到的新轮廓是空的，或者点数不足，则视为丢失一帧
                trackedObjects[i].lost_frames++;
                // std::cout << "警告：匹配到的轮廓为空/无效，对象ID " << trackedObjects[i].id << " 被视为丢失一帧。\n";
            }
        }
        else {
            trackedObjects[i].lost_frames++;
        }
    }

    for (size_t j = 0; j < detected_objects_with_contours.size(); ++j) {
        if (!matched_detected[j]) {
            // 新增：确保新检测到的轮廓不为空且点数足够才能创建TrackedObject
            if (!detected_objects_with_contours[j].second.empty() &&
                detected_objects_with_contours[j].second.size() >= 3)
            {
                TrackedObject new_obj(next_object_id++,
                    detected_objects_with_contours[j].first,
                    detected_objects_with_contours[j].second);

                if (new_obj.centroid.x >= 0 && new_obj.centroid.x < trapezoid_mask_for_check.cols &&
                    new_obj.centroid.y >= 0 && new_obj.centroid.y < trapezoid_mask_for_check.rows &&
                    core_region_relative_roi.contains(new_obj.centroid) &&
                    trapezoid_mask_for_check.at<uchar>(static_cast<int>(new_obj.centroid.y),
                        static_cast<int>(new_obj.centroid.x)) == 255) {
                    new_obj.counted = true;
                    new_obj.counted_in_current_frame = true;
                }
                trackedObjects.push_back(new_obj);
            }
        }
    }

    trackedObjects.erase(std::remove_if(trackedObjects.begin(), trackedObjects.end(),
        [](const TrackedObject& obj) { return obj.lost_frames > MAX_LOST_FRAMES; }),
        trackedObjects.end());
}

// =======================================================
// 主函数
// =======================================================

int main() {
    cv::VideoCapture cap(VIDEO_PATH);
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开视频文件！请检查路径或编解码器。\n" << VIDEO_PATH << std::endl;
        return -1;
    }

    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);

    cv::Mat firstFrame;
    cap.read(firstFrame);
    if (firstFrame.empty()) {
        std::cerr << "错误：视频为空或无法读取第一帧。\n";
        return -1;
    }

    // 步骤1：用户选择ROI
    cv::Mat displayFrame = firstFrame.clone();
    const std::string roi_window_name = "点击选择4点构成ROI";
    cv::namedWindow(roi_window_name, cv::WINDOW_NORMAL);
    cv::setMouseCallback(roi_window_name, mouseCallback, &displayFrame);
    cv::imshow(roi_window_name, displayFrame);

    std::cout << "请在 '" << roi_window_name << "' 窗口中点击4个点来定义ROI。\n";
    std::cout << "这些点将规范化为左右侧边竖直，上下侧边倾斜的梯形（类似地铁站口）。\n";
    std::cout << "点击完4个点后，请在该窗口激活状态下，按任意键继续...\n";

    bool roi_selection_done = false;
    while (!roi_selection_done) {
        if (cv::getWindowProperty(roi_window_name, cv::WND_PROP_AUTOSIZE) < 0) {
            std::cerr << "错误：ROI选择窗口被关闭。程序退出。\n";
            return -1;
        }
        if (trapezoid_points.size() == 4) {
            if (cv::waitKey(0) != -1) {
                roi_selection_done = true;
            }
        }
        else {
            cv::waitKey(30);
        }
    }

    cv::destroyWindow(roi_window_name);
    // 您注释掉的这行，这里暂时保留注释，但推荐打开
    // cv::setMouseCallback(roi_window_name, NULL, NULL); 

    // 这一段因为新的循环逻辑已经确保了 trapezoid_points.size() == 4，所以可以继续注释。
    // if (trapezoid_points.size() != 4) { 
    //    std::cerr << "错误：ROI点选择不完整。程序退出。\n";
    //    return -1;
    // }

    // ========== ROI点规范化：根据用户所选原始点生成直角梯形 ==========
    std::vector<cv::Point> raw_clicked_points;
    raw_clicked_points.insert(raw_clicked_points.end(), trapezoid_points.begin(), trapezoid_points.end());

    trapezoid_points.clear();

    if (raw_clicked_points.empty()) {
        std::cerr << "致命错误：原始点击点集合为空（复制失败）。程序退出。\n";
        return -1;
    }

    int x_left = raw_clicked_points[0].x;
    int x_right = raw_clicked_points[0].x;
    for (const auto& p : raw_clicked_points) {
        x_left = std::min(x_left, p.x);
        x_right = std::max(x_right, p.x);
    }

    if (x_left >= x_right) {
        std::cerr << "错误：选择的ROI点X坐标范围无效。请选择具有不同X坐标的点。" << std::endl;
        return -1;
    }

    std::vector<int> left_ys;
    std::vector<int> right_ys;

    int x_tolerance = 5;

    for (const auto& p : raw_clicked_points) {
        if (std::abs(p.x - x_left) <= x_tolerance) {
            left_ys.push_back(p.y);
        }
        else if (std::abs(p.x - x_right) <= x_tolerance) {
            right_ys.push_back(p.y);
        }
    }

    // 确保有至少两个Y坐标，防止访问越界
    // 如果raw_clicked_points的点都聚集在同一侧，可能导致left_ys或right_ys不足两个
    if (left_ys.size() < 2) {
        // 如果选择的点都在右侧，或者只有少数点在左侧，导致left_ys少于2
        // 这里的逻辑需要更健壮，考虑所有点
        int min_y_all = firstFrame.rows; // 初始化为最大可能值
        int max_y_all = 0;               // 初始化为最小可能值
        for (const auto& p : raw_clicked_points) {
            min_y_all = std::min(min_y_all, p.y);
            max_y_all = std::max(max_y_all, p.y);
        }
        left_ys = { min_y_all, max_y_all };
        std::sort(left_ys.begin(), left_ys.end()); // 排序以确保 min_y_all 在前
    }
    else {
        std::sort(left_ys.begin(), left_ys.end());
    }

    if (right_ys.size() < 2) {
        // 同理，如果选择的点都在左侧，或者只有少数点在右侧，导致right_ys少于2
        int min_y_all = firstFrame.rows;
        int max_y_all = 0;
        for (const auto& p : raw_clicked_points) {
            min_y_all = std::min(min_y_all, p.y);
            max_y_all = std::max(max_y_all, p.y);
        }
        right_ys = { min_y_all, max_y_all };
        std::sort(right_ys.begin(), right_ys.end()); // 排序以确保 min_y_all 在前
    }
    else {
        std::sort(right_ys.begin(), right_ys.end());
    }

    trapezoid_points.push_back(cv::Point(x_left, left_ys[0]));
    trapezoid_points.push_back(cv::Point(x_right, right_ys[0]));
    trapezoid_points.push_back(cv::Point(x_right, right_ys[1]));
    trapezoid_points.push_back(cv::Point(x_left, left_ys[1]));

    fixed_roi_rect = cv::boundingRect(trapezoid_points);
    fixed_roi_rect = fixed_roi_rect & cv::Rect(0, 0, frame_width, frame_height);

    if (fixed_roi_rect.empty() || fixed_roi_rect.width <= 0 || fixed_roi_rect.height <= 0) {
        std::cerr << "错误：规范化后的ROI矩形为空或无效，请确保选择的点在图像内部且形成有效区域。" << std::endl;
        return -1;
    }
    std::cout << "Normalized ROI Trapezoid Points (Absolute): ";
    for (const auto& p : trapezoid_points) {
        std::cout << p << " ";
    }
    std::cout << std::endl;
    std::cout << "Fixed ROI Rect (Absolute): " << fixed_roi_rect << std::endl;

    cv::Size roi_video_size(fixed_roi_rect.width, fixed_roi_rect.height);

    cv::VideoWriter roi_writer("roi_output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, roi_video_size);
    cv::VideoWriter detect_writer("detect_output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, roi_video_size);
    cv::VideoWriter comparison_writer("comparison_output.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(roi_video_size.width * 2, roi_video_size.height));

    if (!roi_writer.isOpened() || !detect_writer.isOpened() || !comparison_writer.isOpened()) {
        std::cerr << "错误：VideoWriter 初始化失败！请尝试修改编码器（例如'MJPG'或'mp4v'），或检查安装的OpenCV是否支持AVI。\n";
        return -1;
    }

    std::ofstream csvFile("green_area_log.csv");
    if (!csvFile.is_open()) {
        std::cerr << "错误：无法创建 green_area_log.csv 文件。\n";
        return -1;
    }
    csvFile << "Frame,TotalNewMitochondriaArea\n";

    cv::Rect core_region_abs = computeCoreRegion(fixed_roi_rect, CORE_REGION_SHRINK_FACTOR);
    cv::Rect core_region_relative_roi(core_region_abs.x - fixed_roi_rect.x,
        core_region_abs.y - fixed_roi_rect.y,
        core_region_abs.width,
        core_region_abs.height);

    cv::namedWindow("BGR 绿色阈值调节", cv::WINDOW_NORMAL);
    cv::createTrackbar("G_Min", "BGR 绿色阈值调节", &g_threshold_trackbar_val, 255);
    cv::createTrackbar("R_Max", "BGR 绿色阈值调节", &r_max_trackbar_val, 255);
    cv::createTrackbar("B_Max", "BGR 绿色阈值调节", &b_max_trackbar_val, 255);
    cv::createTrackbar("G_R_Diff_Min", "BGR 绿色阈值调节", &g_r_diff_min_trackbar_val, 255);
    cv::createTrackbar("G_B_Diff_Min", "BGR 绿色阈值调节", &g_b_diff_min_trackbar_val, 255);

    cv::namedWindow("裁剪后的原始ROI区域", cv::WINDOW_NORMAL);
    cv::namedWindow("1. Current Mask (ROI size)", cv::WINDOW_NORMAL);
    cv::namedWindow("伪色检测结果", cv::WINDOW_NORMAL);
    cv::namedWindow("面积曲线图", cv::WINDOW_NORMAL);
    cv::namedWindow("ROI对比视图 (原始 vs 伪色)", cv::WINDOW_NORMAL);

    int frame_idx = 0;
    while (true) {
        cv::Mat current_frame;
        cap.read(current_frame);
        if (current_frame.empty()) {
            std::cout << "视频结束或读取失败。\n";
            break;
        }

        cv::Mat roi_cropped_content = current_frame(fixed_roi_rect).clone();

        std::vector<cv::Point> relative_trapezoid_points_display = getRelativeTrapezoidPoints(trapezoid_points, fixed_roi_rect);
        cv::polylines(roi_cropped_content, std::vector<std::vector<cv::Point>>{relative_trapezoid_points_display}, true, cv::Scalar(0, 255, 255), 1);
        cv::imshow("裁剪后的原始ROI区域", roi_cropped_content);

        cv::Mat trapezoid_mask_cropped_size = cv::Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        cv::fillPoly(trapezoid_mask_cropped_size, std::vector<std::vector<cv::Point>>{relative_trapezoid_points_display}, cv::Scalar(255));

        BgrGreenThresholds current_thresholds;
        current_thresholds.g_threshold = g_threshold_trackbar_val;
        current_thresholds.r_max = r_max_trackbar_val;
        current_thresholds.b_max = b_max_trackbar_val;
        current_thresholds.g_r_diff_min = g_r_diff_min_trackbar_val;
        current_thresholds.g_b_diff_min = g_b_diff_min_trackbar_val;

        cv::Mat current_processed_roi_mask;
        std::vector<std::vector<cv::Point>> detected_contours_raw = extractGreenContoursBGR(
            roi_cropped_content,
            trapezoid_mask_cropped_size,
            AREA_MIN, AREA_MAX,
            current_processed_roi_mask,
            current_thresholds
        );
        cv::imshow("1. Current Mask (ROI size)", current_processed_roi_mask);

        std::vector<std::pair<cv::Point2f, std::vector<cv::Point>>> detected_objects_for_tracking;
        for (const auto& contour : detected_contours_raw) {
            // 这里已经确保轮廓非空且点数>=3，所以直接处理即可
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) { // 避免除以零
                cv::Point2f centroid(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
                detected_objects_for_tracking.push_back({ centroid, contour });
            }
        }

        updateTrackedObjects(detected_objects_for_tracking, core_region_relative_roi, trapezoid_mask_cropped_size);

        int new_mitos_count = 0;
        double current_frame_total_new_area = 0.0;
        cv::Mat new_pixel_mask_for_display = cv::Mat::zeros(roi_cropped_content.size(), CV_8UC1);
        std::vector<double> new_areas_list;

        for (auto& obj : trackedObjects) {
            if (obj.counted_in_current_frame) {
                new_mitos_count++;
                // 绘制新计数的线粒体到掩码，用于伪色图 (填充效果)
                // 再次检查轮廓是否为空，理论上这里应该非空
                if (!obj.contour.empty()) { // **新增安全检查：在绘制之前检查轮廓是否为空**
                    double area = cv::contourArea(obj.contour);
                    current_frame_total_new_area += area;
                    new_areas_list.push_back(area);
                    // **核心修改：只有当轮廓非空时，才构造临时的 vector<vector<Point>> 传递给 drawContours**
                    cv::drawContours(new_pixel_mask_for_display, std::vector<std::vector<cv::Point>>{obj.contour}, 0, cv::Scalar(255), cv::FILLED);
                }
                else {
                    // 理论上不应该发生，但如果发生了，也要处理
                    std::cout << "警告: 发现空的已计数线粒体轮廓 (ID: " << obj.id << ")，跳过面积计算和绘制。\n";
                }
            }
        }

        area_history.push_back(current_frame_total_new_area);
        if (area_history.size() > static_cast<size_t>(MAX_TRACK_HISTORY)) {
            area_history.erase(area_history.begin());
        }

        std::cout << "帧 " << frame_idx << " - 检测到 " << new_mitos_count << " 个新线粒体。总面积: "
            << std::fixed << std::setprecision(2) << current_frame_total_new_area << std::endl;
        std::string areas_str = "面积: ";
        for (size_t i = 0; i < new_areas_list.size(); ++i) {
            areas_str += std::to_string(static_cast<int>(new_areas_list[i]));
            if (i < new_areas_list.size() - 1) areas_str += ", ";
        }
        std::cout << areas_str << std::endl;

        csvFile << frame_idx << "," << current_frame_total_new_area << "\n";

        cv::Mat pseudo_result_full = roi_cropped_content.clone();

        // 绘制所有被追踪线粒体的当前轮廓（黄色边框）和中心点（红色）
        for (const auto& obj : trackedObjects) {
            // **核心修改：只有当轮廓非空时，才构造临时的 vector<vector<Point>> 传递给 drawContours**
            if (!obj.contour.empty()) {
                cv::drawContours(pseudo_result_full, std::vector<std::vector<cv::Point>>{obj.contour}, 0, cv::Scalar(0, 255, 255), 1);
            }
            else {
                // std::cout << "警告: 追踪对象 " << obj.id << " 的轮廓为空，跳过绘制轮廓。\n";
            }
            cv::circle(pseudo_result_full, obj.centroid, 2, cv::Scalar(0, 0, 255), -1); // 红色中心点
        }

        cv::rectangle(pseudo_result_full, core_region_relative_roi, cv::Scalar(0, 255, 255), 0);//计数区域
        cv::polylines(pseudo_result_full, std::vector<std::vector<cv::Point>>{relative_trapezoid_points_display}, true, cv::Scalar(255, 0, 255), 1);

        pseudo_result_full.setTo(cv::Scalar(0, 255, 0), new_pixel_mask_for_display);

        cv::imshow("伪色检测结果", pseudo_result_full);

        cv::imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));

        roi_writer.write(roi_cropped_content);
        detect_writer.write(pseudo_result_full);

        cv::Mat combined_roi_frame;
        cv::hconcat(roi_cropped_content, pseudo_result_full, combined_roi_frame);
        cv::imshow("ROI对比视图 (原始 vs 伪色)", combined_roi_frame);
        comparison_writer.write(combined_roi_frame);

        frame_idx++;

        char key = (char)cv::waitKey(30);
        if (key == 'q' || key == 27)
            break;
    }

    cap.release();
    roi_writer.release();
    detect_writer.release();
    comparison_writer.release();
    csvFile.close();
    cv::destroyAllWindows();

    std::cout << "处理完成。结果视频和CSV数据已保存。\n";

    return 0;
}
