#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip> // For std::setprecision
#include <numeric> // For std::accumulate
#include <algorithm> // For std::sort, std::min, std::max
#include <map>     // For storing centroid history per object

// =======================================================
// 全局参数和变量
// =======================================================

// 视频路径
const std::string VIDEO_PATH = "D:/BaiduSyncdisk/193311.avi"; // <-- 请修改为您的视频路径

// 用户选择的矩形ROI (用于裁剪视频帧)
cv::Rect fixed_roi_rect;
// ROI核心区域的宽度相对主ROI宽度的比例（用于计数）。这里设置为 1/8。
const double CORE_REGION_WIDTH_FACTOR = 1.0 / 8.0; // 核心区域宽度为主ROI宽度的1/8

// 轮廓面积过滤阈值
const double AREA_MIN = 5.0; // 最小面积阈值，防止噪声
const double AREA_MAX = 5000.0; // 最大面积阈值，防止大块干扰物

// 追踪相关参数
const double MAX_DIST_SQ = 100.0 * 100.0; // 追踪对象质心距离平方最大值
const int MAX_LOST_FRAMES = 5;            // 最大连续丢失帧数
const int MAX_TRACK_HISTORY = 300;        // 最大追踪历史帧数 (用于显示面积图和质心Y图)

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
    cv::Point2f centroid; // 当前质心 (相对于ROI裁剪后图像)
    // 修改历史记录，存储 (帧号, 质心) 对，用于Y位置图
    std::vector<std::pair<int, cv::Point2f>> history;
    int lost_frames;      // 连续丢失帧数
    bool counted;         // 是否已被计数过 (首次进入核心区域时设置为true)
    bool counted_in_current_frame; // 是否在当前帧被识别为“新计数”
    std::vector<cv::Point> contour; // 当前帧的轮廓 (相对于ROI裁剪后图像)

    TrackedObject(int frame_idx, int i, cv::Point2f c, const std::vector<cv::Point>& cont) :
        id(i), centroid(c), lost_frames(0), counted(false), counted_in_current_frame(false), contour(cont) {
        history.push_back({frame_idx, c});
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

// 计算矩形ROI核心区域（相对于ROI裁剪后图像的相对坐标）
// 核心区域是一个垂直长条，高度等于ROI高度，宽度是ROI宽度乘以width_factor
cv::Rect computeCoreRegionRelative(const cv::Rect& roi_rect, double width_factor) {
    int roi_width = roi_rect.width;
    int roi_height = roi_rect.height;

    // 计算核心区域的宽度
    int core_width = static_cast<int>(roi_width * width_factor);
    // 确保宽度至少为1像素
    if (core_width < 1) core_width = 1;

    // 核心区域的高度与ROI高度相同
    int core_height = roi_height;

    // 计算核心区域的左上角坐标，使其在ROI中水平居中
    int core_x = (roi_width - core_width) / 2;
    int core_y = 0; // 从ROI顶部开始

    // 确保计算出的矩形有效
    if (core_width <= 0 || core_height <= 0) {
         // 返回一个无效矩形
         return cv::Rect(0, 0, 0, 0);
    }

    // 确保核心区域不超出ROI边界 (计算方法已基本保证，这里是额外的安全检查)
     core_x = std::max(0, core_x);
     core_y = std::max(0, core_y);
     core_width = std::min(core_width, roi_width - core_x);
     core_height = std::min(core_height, roi_height - core_y);

    return cv::Rect(core_x, core_y, core_width, core_height);
}

// 绘制面积历史曲线图
cv::Mat generateAreaPlot(const std::vector<double>& history, int width, int height) {
    cv::Mat plot_image = cv::Mat::zeros(height, width, CV_8UC3);
    if (history.empty()) return plot_image;

    double max_area = 0.0;
    for (double area : history) {
        max_area = std::max(max_area, area);
    }

    if (max_area < 1.0) max_area = 1.0; // 避免除以零或接近零的值

    // 调整 x 轴缩放，使其与实际历史帧数匹配
    double x_scale = static_cast<double>(width) / history.size();
    double y_scale = static_cast<double>(height) / (max_area * 1.1); // 在最大值基础上留10%的边距

    cv::line(plot_image, cv::Point(0, height - 1), cv::Point(width - 1, height - 1), cv::Scalar(255, 255, 255), 1); // X轴
    cv::line(plot_image, cv::Point(0, 0), cv::Point(0, height - 1), cv::Scalar(255, 255, 255), 1);                 // Y轴

    // 绘制历史曲线
    for (size_t i = 0; i < history.size(); ++i) {
        if (i > 0) {
             cv::Point p1(static_cast<int>((i - 1) * x_scale), height - 1 - static_cast<int>(history[i - 1] * y_scale));
             cv::Point p2(static_cast<int>(i * x_scale), height - 1 - static_cast<int>(history[i] * y_scale));

            // Clamp points to image boundaries to prevent drawing outside
            p1.x = std::max(0, std::min(width - 1, p1.x));
            p1.y = std::max(0, std::min(height - 1, p1.y));
            p2.x = std::max(0, std::min(width - 1, p2.x));
            p2.y = std::max(0, std::min(height - 1, p2.y));

            // 检查点是否在图像范围内，避免崩溃 (再次检查，更安全)
            if (p1.x >= 0 && p1.x < width && p1.y >= 0 && p1.y < height &&
                p2.x >= 0 && p2.x < width && p2.y >= 0 && p2.y < height)
            {
                 cv::line(plot_image, p1, p2, cv::Scalar(0, 255, 0), 1);
            }
        }
    }
    return plot_image;
}

// 绘制线粒体Y坐标历史曲线图
cv::Mat generateYPosPlot(const std::vector<TrackedObject>& objects, int plot_width, int plot_height, int roi_height, int current_frame_idx) {
     cv::Mat plot_image = cv::Mat::zeros(plot_height, plot_width, CV_8UC3);

     if (objects.empty()) return plot_image;

     // Y axis scale: map 0 to roi_height to plot_height to 0
     double y_scale = static_cast<double>(plot_height) / (roi_height > 0 ? roi_height : 1); // Avoid division by zero

     // X axis scale: map MAX_TRACK_HISTORY frames to plot_width pixels
     double x_scale = static_cast<double>(plot_width) / MAX_TRACK_HISTORY;

     // Determine the range of frame indices to display on the X axis
     int start_frame_for_plot = std::max(0, current_frame_idx - MAX_TRACK_HISTORY + 1);
     int end_frame_for_plot = current_frame_idx;

     // Draw Axes
     cv::line(plot_image, cv::Point(0, plot_height - 1), cv::Point(plot_width - 1, plot_height - 1), cv::Scalar(255, 255, 255), 1); // X axis (pseudo time/frame)
     cv::line(plot_image, cv::Point(0, 0), cv::Point(0, plot_height - 1), cv::Scalar(255, 255, 255), 1);                 // Y axis (position)

      // Draw Y axis labels (simplified: top=0, bottom=roi_height)
     cv::putText(plot_image, "0", cv::Point(5, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
     cv::putText(plot_image, std::to_string(roi_height), cv::Point(5, plot_height - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

     // Define colors for different object tracks (cycle through)
     std::vector<cv::Scalar> colors = {
         cv::Scalar(255, 0, 0),  // Blue
         cv::Scalar(0, 255, 0),  // Green
         cv::Scalar(0, 0, 255),  // Red
         cv::Scalar(255, 255, 0),// Cyan
         cv::Scalar(0, 255, 255),// Yellow
         cv::Scalar(255, 0, 255) // Magenta
     };

     // Draw each object's Y-position history
     for (const auto& obj : objects) {
          if (obj.history.empty()) continue;

          cv::Scalar track_color = colors[obj.id % colors.size()]; // Assign color based on object ID

          for (size_t i = 0; i < obj.history.size(); ++i) {
               int frame = obj.history[i].first;
               double y_pos = obj.history[i].second.y;

               // Map frame index to plot X coordinate
               int plot_x = static_cast<int>((frame - start_frame_for_plot) * x_scale);
               // Map Y position to plot Y coordinate (remember Y increases downwards)
               int plot_y = static_cast<int>(plot_height - 1 - y_pos * y_scale);

                // Clamp plot coordinates to image boundaries
               plot_x = std::max(0, std::min(plot_width - 1, plot_x));
               plot_y = std::max(0, std::min(plot_height - 1, plot_y));

               if (i > 0) {
                   int prev_frame = obj.history[i-1].first;
                   double prev_y_pos = obj.history[i-1].second.y;

                   int prev_plot_x = static_cast<int>((prev_frame - start_frame_for_plot) * x_scale);
                   int prev_plot_y = static_cast<int>(plot_height - 1 - prev_y_pos * y_scale);

                   // Clamp previous plot coordinates
                   prev_plot_x = std::max(0, std::min(plot_width - 1, prev_plot_x));
                   prev_plot_y = std::max(0, std::min(plot_height - 1, prev_plot_y));

                   // Draw line segment
                   cv::line(plot_image, cv::Point(prev_plot_x, prev_plot_y), cv::Point(plot_x, plot_y), track_color, 1);
               } else {
                   // Draw the first point as a circle
                   cv::circle(plot_image, cv::Point(plot_x, plot_y), 2, track_color, -1);
               }
          }
     }

     return plot_image;
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

// 核心功能：在 BGR 空间中提取绿色轮廓。输入为裁剪后的ROI图像。
std::vector<std::vector<cv::Point>> extractGreenContoursBGR(
    const cv::Mat& bgr_roi_image, // This IS the cropped ROI image
    double min_area,
    double max_area,
    cv::Mat& output_mask_display, // This will be the binary mask (size of bgr_roi_image)
    const BgrGreenThresholds& thresholds
) {
    cv::Mat bgr_green_mask = cv::Mat::zeros(bgr_roi_image.size(), CV_8UC1);

    // Apply BGR thresholds
#pragma omp parallel for collapse(2) // Optional: Use OpenMP for potential speedup on multi-core CPUs
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

    output_mask_display = applyMorphology(bgr_green_mask);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // findContours finds contours in the output_mask_display (size of ROI)
    cv::findContours(output_mask_display, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> filtered_contours;
    for (const auto& contour : contours) {
        // 过滤空轮廓和点过少的轮廓 - should not happen after morphology, but good safe guard
        if (contour.empty() || contour.size() < 3) {
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
// Uses core_region_relative_roi (a vertical rect) to check for counting trigger
void updateTrackedObjects(int current_frame_idx, const std::vector<std::pair<cv::Point2f, std::vector<cv::Point>>>& detected_objects_with_contours, const cv::Rect& core_region_relative_roi) {
    // Reset counted_in_current_frame flag for all tracked objects
    for (auto& obj : trackedObjects) {
        obj.counted_in_current_frame = false;
    }

    std::vector<bool> matched_detected(detected_objects_with_contours.size(), false);

    // Stage 1: Match existing tracks to detected objects
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
            // Found a match
            if (!detected_objects_with_contours[best_match_idx].second.empty()) // Check if contour is not empty before assigning
            {
                trackedObjects[i].centroid = detected_objects_with_contours[best_match_idx].first;
                trackedObjects[i].contour = detected_objects_with_contours[best_match_idx].second; // Update contour for area calculation
                // Add current frame index and centroid to history
                trackedObjects[i].history.push_back({current_frame_idx, trackedObjects[i].centroid});
                // Limit history size
                if (trackedObjects[i].history.size() > static_cast<size_t>(MAX_TRACK_HISTORY)) {
                    trackedObjects[i].history.erase(trackedObjects[i].history.begin());
                }

                trackedObjects[i].lost_frames = 0;
                matched_detected[best_match_idx] = true;

                // Check if the object is in the core vertical strip region and hasn't been counted yet
                // Centroid coordinates are relative to the ROI image (0,0)
                if (core_region_relative_roi.contains(trackedObjects[i].centroid) && !trackedObjects[i].counted) {
                    trackedObjects[i].counted = true;
                    trackedObjects[i].counted_in_current_frame = true; // Mark as newly counted in this frame
                } else {
                    trackedObjects[i].counted_in_current_frame = false; // Ensure this is false if not newly counted
                }
            } else {
                // Matched to an empty or invalid contour, treat as lost frame
                trackedObjects[i].lost_frames++;
                 trackedObjects[i].counted_in_current_frame = false; // Cannot be newly counted if contour is invalid
            }

        } else {
            // No match found (lost track), increment lost frames
            trackedObjects[i].lost_frames++;
            trackedObjects[i].counted_in_current_frame = false; // Cannot be newly counted if lost
        }
    }

    // Stage 2: Create new tracks for unmatched detected objects
    for (size_t j = 0; j < detected_objects_with_contours.size(); ++j) {
        if (!matched_detected[j]) {
            // New object detected
            if (!detected_objects_with_contours[j].second.empty()) // Check if contour is not empty before creating new track
            {
                 // Create new object, passing current frame index
                TrackedObject new_obj(current_frame_idx, next_object_id++, detected_objects_with_contours[j].first, detected_objects_with_contours[j].second);

                // Check if the new object is already in the core vertical strip region upon detection
                // Centroid coordinates are relative to the ROI image (0,0)
                if (core_region_relative_roi.contains(new_obj.centroid)) {
                    new_obj.counted = true;
                    new_obj.counted_in_current_frame = true; // Newly counted in this frame
                }

                trackedObjects.push_back(new_obj);
            } else {
                 // Detected an empty or invalid contour, do not create a new track
                // std::cerr << "Warning: Detected an empty or invalid contour, skipping new track creation.\n";
            }
        }
    }

    // Stage 3: Remove old tracks that have been lost for too many frames
    trackedObjects.erase(std::remove_if(trackedObjects.begin(), trackedObjects.end(),
        [](const TrackedObject& obj) { return obj.lost_frames > MAX_LOST_FRAMES; }),
        trackedObjects.end());
}

// =======================================================
// 主函数
// =======================================================

int main() {

    long long total_mitos_counted = 0; // 累计计数
    double total_mitos_area = 0.0;     // 累计面积 (累计新计数线粒体的总面积)

    // 为输出到文件准备
    std::ofstream summaryFile("mitochondria_summary.txt");
    if (!summaryFile.is_open()) {
        std::cerr << "错误: 无法创建 mitochondria_summary.txt 文件。\n";
        return -1;
    }
    summaryFile << "帧号, 本帧新计数线粒体个数, 本帧新计数线粒体总面积\n";

    // 准备质心输出文件
    std::ofstream centroidFile("centroids.csv");
    if (!centroidFile.is_open()) {
         std::cerr << "错误: 无法创建 centroids.csv 文件。\n";
         // Clean up already created files
         summaryFile.close();
         return -1;
    }
    centroidFile << "Frame,ObjectID,CentroidX,CentroidY,IsCounted\n"; // 添加IsCounted列

    // 准备面积 log 文件
    std::ofstream csvFile("green_area_log.csv");
    if (!csvFile.is_open()) {
        std::cerr << "错误：无法创建 green_area_log.csv 文件。\n";
        // Clean up already created files
        summaryFile.close();
        centroidFile.close();
        return -1;
    }
    csvFile << "Frame,TotalNewMitochondriaArea\n";

    cv::VideoCapture cap(VIDEO_PATH);
    if (!cap.isOpened()) {
        std::cerr << "错误：无法打开视频文件！请检查路径或编解码器。\n" << VIDEO_PATH << std::endl;
        return -1;
    }

    // 获取视频的原始尺寸
    int frame_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int frame_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);

    cv::Mat firstFrame;
    cap.read(firstFrame);
    if (firstFrame.empty()) {
        std::cerr << "错误：视频为空或无法读取第一帧。\n";
        return -1;
    }

    // 步骤1：用户选择矩形ROI
    const std::string roi_window_name = "请拖动鼠标选择主要ROI区域 (一个矩形)";
    cv::namedWindow(roi_window_name, cv::WINDOW_NORMAL);
    std::cout << "请在 '" << roi_window_name << "' 窗口中拖动鼠标选择一个矩形作为主要ROI区域。\n";
    std::cout << "将在这个区域内检测线粒体。\n";
    std::cout << "位于此区域中间 " << static_cast<int>(1/CORE_REGION_WIDTH_FACTOR) << " 分之一宽度的垂直长条区域将作为计数框。\n";
    std::cout << "确认选择后，请按Enter键或Space键。取消选择请按 'c' 键。\n";

    fixed_roi_rect = cv::selectROI(roi_window_name, firstFrame, true, false);

    cv::destroyWindow(roi_window_name);

    if (fixed_roi_rect.empty() || fixed_roi_rect.width <= 0 || fixed_roi_rect.height <= 0) {
        std::cerr << "错误：未选择有效ROI区域或取消选择。程序退出。\n";
        // Clean up already created files
        summaryFile.close();
        centroidFile.close();
        csvFile.close();
        return -1;
    }
    std::cout << "User selected Main ROI Rect (Absolute): " << fixed_roi_rect << std::endl;

     // Adjust fixed_roi_rect to be within frame boundaries just in case
     fixed_roi_rect = fixed_roi_rect & cv::Rect(0, 0, frame_width, frame_height);
     if (fixed_roi_rect.empty() || fixed_roi_rect.width <= 0 || fixed_roi_rect.height <= 0) {
         std::cerr << "错误：选择的ROI区域不完整或超出帧边界。程序退出。\n";
         // Clean up already created files
         summaryFile.close();
         centroidFile.close();
         csvFile.close();
         return -1;
     }

    cv::Size roi_video_size(fixed_roi_rect.width, fixed_roi_rect.height);

    // 使用合适的编码器，'XVID' 是比较常见的，但有时需要尝试其他如 'MJPG' 或 'mp4v'
    // For cross-platform compatibility or specific needs, consider alternatives.
    // Or just output image sequences and encode later.
    int fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    cv::VideoWriter roi_writer("roi_output.avi", fourcc, fps, roi_video_size);
    cv::VideoWriter detect_writer("detect_output.avi", fourcc, fps, roi_video_size);
    cv::VideoWriter comparison_writer("comparison_output.avi", fourcc, fps, cv::Size(roi_video_size.width * 2, roi_video_size.height));

    if (!roi_writer.isOpened() || !detect_writer.isOpened() || !comparison_writer.isOpened()) {
        std::cerr << "错误：VideoWriter 初始化失败！请尝试修改编码器（例如'MJPG'或'mp4v'），或检查安装的OpenCV是否支持AVI。\n";
        // Clean up already created files
        summaryFile.close();
        centroidFile.close();
        csvFile.close();
        return -1; // Exit if video writer failed
    }

    // 计算相对于裁剪后ROI图像的“核心区域”矩形 (垂直长条，宽度是主ROI的1/8)
    cv::Rect core_region_relative_roi = computeCoreRegionRelative(fixed_roi_rect, CORE_REGION_WIDTH_FACTOR);

    // 检查核心区域是否有效
    if (core_region_relative_roi.empty() || core_region_relative_roi.width <= 0 || core_region_relative_roi.height <= 0) {
         std::cerr << "错误: 计算出的核心计数区域无效。请检查 CORE_REGION_WIDTH_FACTOR 和选择的ROI尺寸。\n";
         // Clean up resources before exiting
         summaryFile.close();
         centroidFile.close();
         csvFile.close();
         cap.release();
         roi_writer.release();
         detect_writer.release();
         comparison_writer.release();
         cv::destroyAllWindows();
         return -1;
    }
    std::cout << "Core Counting Region (Relative to ROI): " << core_region_relative_roi << std::endl;

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
    cv::namedWindow("线粒体Y坐标曲线图", cv::WINDOW_NORMAL); // 新增Y坐标图窗口
    cv::namedWindow("ROI对比视图 (原始 vs 伪色)", cv::WINDOW_NORMAL);

    int frame_idx = 0;
    while (true) {
        cv::Mat current_frame;
        cap.read(current_frame);
        if (current_frame.empty()) {
            std::cout << "视频结束或读取失败。\n";
            break;
        }

        // 裁剪到用户选择的矩形ROI区域
        cv::Mat roi_cropped_content = current_frame(fixed_roi_rect).clone();

        // --- 绘制到原始ROI窗口 ---
        cv::Mat roi_display_frame = roi_cropped_content.clone();
        // 绘制裁剪区域本身的边界 (黄色)
        cv::rectangle(roi_display_frame, cv::Point(0, 0), cv::Point(roi_display_frame.cols-1, roi_display_frame.rows-1), cv::Scalar(0, 255, 255), 1);
        // 绘制核心计数区域 (绿色)
        cv::rectangle(roi_display_frame, core_region_relative_roi, cv::Scalar(0, 255, 0), 1);
        cv::imshow("裁剪后的原始ROI区域", roi_display_frame);

        BgrGreenThresholds current_thresholds;
        current_thresholds.g_threshold = g_threshold_trackbar_val;
        current_thresholds.r_max = r_max_trackbar_val;
        current_thresholds.b_max = b_max_trackbar_val;
        current_thresholds.g_r_diff_min = g_r_diff_min_trackbar_val;
        current_thresholds.g_b_diff_min = g_b_diff_min_trackbar_val;

        cv::Mat current_processed_roi_mask; // This will be the binary mask derived from color filtering within the ROI
        std::vector<std::vector<cv::Point>> detected_contours_raw = extractGreenContoursBGR(
            roi_cropped_content, // Pass the cropped image
            AREA_MIN, AREA_MAX,
            current_processed_roi_mask, // Output mask
            current_thresholds
        );
        cv::imshow("1. Current Mask (ROI size)", current_processed_roi_mask);

        std::vector<std::pair<cv::Point2f, std::vector<cv::Point>>> detected_objects_for_tracking;
        for (const auto& contour : detected_contours_raw) {
             // Ensure valid moments before calculating centroid
            cv::Moments m = cv::moments(contour);
            if (m.m00 > 0) {
                cv::Point2f centroid(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
                detected_objects_for_tracking.push_back({centroid, contour});
            }
        }

        // Update tracked objects using the detected ones and the core counting region (vertical strip)
        updateTrackedObjects(frame_idx, detected_objects_for_tracking, core_region_relative_roi); // Pass current_frame_idx

        int new_mitos_count = 0;
        double current_frame_total_new_area = 0.0;
        cv::Mat new_pixel_mask_for_display = cv::Mat::zeros(roi_cropped_content.size(), CV_8UC1);
        std::vector<double> new_areas_list; // For detailed area logging

        for (auto& obj : trackedObjects) {
            // Log centroids for all currently tracked objects in the current frame
            // Only log if centroid is within the main ROI bounds to avoid huge files from lost/out-of-frame objects
            if (obj.centroid.x >= 0 && obj.centroid.x < roi_cropped_content.cols &&
                obj.centroid.y >= 0 && obj.centroid.y < roi_cropped_content.rows) {
                 centroidFile << frame_idx << "," << obj.id << "," << std::fixed << std::setprecision(2) << obj.centroid.x << "," << std::fixed << std::setprecision(2) << obj.centroid.y << "," << (obj.counted ? "TRUE" : "FALSE") << "\n";
            }

            if (obj.counted_in_current_frame) {
                new_mitos_count++;
                // Draw newly counted mitochondria onto a mask for pseudocoloring the result image
                if (!obj.contour.empty()) {
                     double area = cv::contourArea(obj.contour);
                     current_frame_total_new_area += area;
                     new_areas_list.push_back(area);
                    cv::drawContours(new_pixel_mask_for_display, std::vector<std::vector<cv::Point>>{obj.contour}, 0, cv::Scalar(255), cv::FILLED);
                }
            }
        }
        centroidFile.flush(); // 确保数据写入文件

        // Update area history only for the newly counted objects (total area in this frame)
        area_history.push_back(current_frame_total_new_area);
        // Keep area history within MAX_TRACK_HISTORY limit
        if (area_history.size() > static_cast<size_t>(MAX_TRACK_HISTORY)) {
             area_history.erase(area_history.begin());
        }

        // Output summary for the frame
        std::cout << "帧 " << frame_idx << ": 新检测线粒体数量 = " << new_mitos_count
                   << ", 总面积 = " << std::fixed << std::setprecision(2) << current_frame_total_new_area << std::endl;
        // 暂不打印面积明细到控制台，只输出总数和总面积，避免输出过多
        // std::string areas_str = "本帧新计数面积明细: [";
        // for (size_t i = 0; i < new_areas_list.size(); ++i) {
        //     areas_str += std::to_string(static_cast<int>(new_areas_list[i]));
        //     if (i < new_areas_list.size() - 1) areas_str += ", ";
        // }
        // areas_str += "]";
        // std::cout << areas_str << std::endl;

        // Write summary to file
        summaryFile << frame_idx << "," << new_mitos_count << "," << current_frame_total_new_area << "\n";
        csvFile << frame_idx << "," << current_frame_total_new_area << "\n"; // Also csv for plotting

        // Accumulate total counts and area
        total_mitos_counted += new_mitos_count;
        total_mitos_area += current_frame_total_new_area;

        // Prepare the result image (pseudocolored)
         cv::Mat pseudo_result_full = roi_cropped_content.clone(); // Start with original ROI content (color)

        // Yellow contour and red centroid for ALL currently tracked objects on the pseudo result image
        for (const auto& obj : trackedObjects) {
            if (!obj.contour.empty()) {
                 cv::drawContours(pseudo_result_full, std::vector<std::vector<cv::Point>>{obj.contour}, 0, cv::Scalar(0, 255, 255), 1); // Yellow contour
            }
             // Check centroid is valid before drawing circle and history
             if (obj.centroid.x >= 0 && obj.centroid.x < pseudo_result_full.cols &&
                 obj.centroid.y >= 0 && obj.centroid.y < pseudo_result_full.rows) {
                  cv::circle(pseudo_result_full, obj.centroid, 2, cv::Scalar(0, 0, 255), -1); // Red centroid

                  // 绘制轨迹历史点 (在伪色图上)
                  for(size_t i = 0; i < obj.history.size(); ++i) {
                       // 确保历史点也是在ROI内的相对坐标
                       if (obj.history[i].second.x().x >= 0 && obj.history[i].second.x() < pseudo_result_full.cols &&
                           obj.history[i].second.y() >= 0 && obj.history[i].second.y() < pseudo_result_full.rows)
                       {
                            if (i > 0) {
                                 cv::line(pseudo_result_full, obj.history[i-1].second, obj.history[i].second, cv::Scalar(255, 0, 0), 1); // 蓝色轨迹
                            } else {
                                // Draw the first history point as a smaller circle
                                cv::circle(pseudo_result_full, obj.history[i].second, 1, cv::Scalar(255, 0, 0), -1);
                            }
                       }
                  }
             }
        }

        // Draw ROI rectangle and core counting area rectangle on the pseudo result image
        cv::rectangle(pseudo_result_full, cv::Point(0, 0), cv::Point(pseudo_result_full.cols-1, pseudo_result_full.rows-1), cv::Scalar(0, 255, 255), 1); // ROI boundary (Yellow)
        cv::rectangle(pseudo_result_full, core_region_relative_roi, cv::Scalar(0, 255, 0), 1); // Core counting region (Green)

        // Apply green pseudocolor for NEWLY COUNTED mitochondria pixels
        pseudo_result_full.setTo(cv::Scalar(0, 255, 0), new_pixel_mask_for_display); // Green color for counted pixels

        cv::imshow("伪色检测结果", pseudo_result_full);
        // Save pseudo result image (optional, can be toggled)
        // std::string output_filename = "pseudo_result_frame_" + std::to_string(frame_idx) + ".png";
        // cv::imwrite(output_filename, pseudo_result_full);

        // Generate and display the plots
        cv::imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));
        cv::imshow("线粒体Y坐标曲线图", generateYPosPlot(trackedObjects, 400, 200, roi_video_size.height, frame_idx)); // Pass ROI height and frame_idx

        // Write frames to output videos
        roi_writer.write(roi_display_frame); // Write the frame with ROI and core region drawn
        detect_writer.write(pseudo_result_full);

        // Create and display the comparison view
        cv::Mat combined_roi_frame;
        cv::hconcat(roi_display_frame, pseudo_result_full, combined_roi_frame);
        cv::imshow("ROI对比视图 (原始 vs 伪色)", combined_roi_frame);
        comparison_writer.write(combined_roi_frame);

        frame_idx++;

        // Handle key interrupts
        char key = (char)cv::waitKey(1); // Use waitKey(1) for smooth playback
        if (key == 'q' || key == 27) // 'q' or ESC key to exit
            break;
        if (key == ' ') // Space key to pause/resume
             cv::waitKey(0); // Wait indefinitely until another key press
    }

    // After the loop, save the final plots if desired
    cv::Mat final_area_plot = generateAreaPlot(area_history, 800, 400); // Larger plot for final output
    cv::imwrite("final_area_plot.png", final_area_plot);
    std::cout << "\n最终面积曲线图已保存为 final_area_plot.png\n";

    // Save the final Y position plot
    cv::Mat final_y_pos_plot = generateYPosPlot(trackedObjects, 800, 400, roi_video_size.height, frame_idx - 1); // Use last frame_idx
    cv::imwrite("final_y_pos_plot.png", final_y_pos_plot);
    std::cout << "最终Y坐标曲线图已保存为 final_y_pos_plot.png\n";

    std::cout << "\n-------------------------------------------------\n";
    std::cout << "处理完成！\n";
    std::cout << "总计检测到的新线粒体数量 (累计): " << total_mitos_counted << std::endl;
    std::cout << "总计检测到的新线粒体总面积 (累计): " << std::fixed << std::setprecision(2) << total_mitos_area << std::endl;
    std::cout << "数据已保存到 mitochondria_summary.txt 和 green_area_log.csv。\n";
    std::cout << "质心历史记录已保存到 centroids.csv。\n";
    std::cout << "处理视频输出到 roi_output.avi, detect_output.avi, comparison_output.avi。\n";
    std::cout << "曲线图已保存到 final_area_plot.png 和 final_y_pos_plot.png。\n";
    std::cout << "-------------------------------------------------\n\n";

    // Release resources
    summaryFile.close();
    centroidFile.close();
    csvFile.close();
    cap.release();
    roi_writer.release();
    detect_writer.release();
    comparison_writer.release();
    cv::destroyAllWindows();

    return 0;
}
