#include <iostream>
#include <string>
#include <fstream>
#include <numeric>
#include <algorithm> 
#include <vector>    
#include <map>       // For tracking individual mitochondria (old approach, for comparison)
#include <cmath>     // For std::isnan

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// ========== 全局变量 ==========
vector<double> area_history;
vector<Point> trapezoid_points;
Mat hsv_lower = (Mat_<int>(1, 3) << 0, 0, 0); 
Mat hsv_upper = (Mat_<int>(1, 3) << 180, 255, 255); 
int area_min = 20; 
int area_max = 20000;

// 定义裁剪区域的矩形，一旦确定，视频输出尺寸就固定了
Rect fixed_roi_rect;

// ========== 新的轻量级目标追踪器结构体和全局变量 ==========
struct TrackedObject {
    int id;
    Point2f centroid;
    // float area; // 可以添加其他属性，如面积
    vector<Point> contour; // 保存轮廓信息以便后续绘制
    int frames_since_seen; // 多少帧未被发现
    bool has_entered_core = false; // 标志：是否已进入核心区域并被计数
};

int nextTrackID = 0; // 下一个可用的追踪ID
vector<TrackedObject> trackedObjects; // 当前活跃的追踪对象列表
// double max_centroid_distance = 20.0; // Max distance for centroids to be considered the same object between frames (已移入函数参数)

// 这是您原有的一部分追踪代码，暂时保留，但不再主动使用
// map<int, MitochondrionState> tracked_mitochondria; // 旧的追踪方法
// int next_object_id = 0; // 旧的ID

// ========== 函数声明 ==========
void mouseCallback(int event, int x, int y, int, void* userdata);
Mat createTrapezoidMask(Size size, const vector<Point>& points_input);
Mat excludeWhiteLight(const Mat& hsv);
Mat applyMorphology(const Mat& mask);

// 辅助函数：计算质心距离
float centroidDist(const Point2f& a, const Point2f& b);

// 新的追踪更新函数
// current_detected_objects: 当前帧所有检测到的有效轮廓及质心
// coreRegion: 核心区域的矩形
// max_distance: 两个质心被认为是同一目标的最小距离
// max_inactive_frames: 目标在多少帧未被发现后移除
// new_counted_mitochondria_infos: 输出参数，存储本帧新计数的线粒体的信息 (质心、面积、轮廓)
void updateTrackedObjects(const vector<pair<Point2f, vector<Point>>>& current_detected_objects, Rect coreRegion,
                          double max_distance, int max_inactive_frames,
                          vector<tuple<Point2f, double, vector<Point>>>& new_counted_mitochondria_infos);

Mat generateAreaPlot(const vector<double>& history, int width, int height);
void overlayText(Mat& frame, const string& text);

Scalar computeHSVRangeWithKMeans(const Mat& hsv_roi, const Mat& mask, int k_clusters); 

// ========== 主函数 ==========
int main() {
#ifdef _WIN32
    system("if not exist output mkdir output > nul 2>&1");
    system("if not exist masks mkdir masks > nul 2>&1");
    system("if not exist newmasks mkdir newmasks > nul 2>&1");
#else
    system("mkdir -p output masks newmasks");
#endif

    VideoCapture cap("D:/BaiduSyncdisk/200219.avi");
    if (!cap.isOpened()) {
        cerr << "无法打开视频文件" << endl;
        return -1;
    }

    ofstream csvFile("green_area_log.csv");
    csvFile << "Frame,NewGreenArea\n";

    Mat firstFrame;
    cap.read(firstFrame);
    if (firstFrame.empty()) {
        cerr << "无法读取第一帧" << endl;
        return -1;
    }
    
    Mat displayFrame = firstFrame.clone();
    namedWindow("点击选择4点构成ROI", WINDOW_NORMAL);
    setMouseCallback("点击选择4点构成ROI", mouseCallback, &displayFrame);
    imshow("点击选择4点构成ROI", displayFrame);

    cout << "请在 '点击选择4点构成ROI' 窗口中点击4个点来定义ROI。\n";
    cout << "这些点将规范化为左右侧边竖直，上下侧边倾斜的梯形（类似地铁站口）。\n";
    while (trapezoid_points.size() < 4) {
        waitKey(30);
    }

    // ========== ROI点规范化：根据图片示例生成直角梯形 ==========
    vector<Point> raw_points = trapezoid_points;

    // Find min/max X to define left/right vertical lines
    int x_left = raw_points[0].x, x_right = raw_points[0].x;
    for (const auto& p : raw_points) {
        x_left = min(x_left, p.x);
        x_right = max(x_right, p.x);
    }

    // Sort to easily pick points for top/bottom Y coordinates
    sort(raw_points.begin(), raw_points.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
    });

    int p1_y = min(raw_points[0].y, raw_points[1].y); 
    int p4_y = max(raw_points[0].y, raw_points[1].y); 
    
    int p2_y = min(raw_points[2].y, raw_points[3].y); 
    int p3_y = max(raw_points[2].y, raw_points[3].y); 
    
    trapezoid_points.clear();
    trapezoid_points.push_back(Point(x_left, p1_y)); // P1 (左上)
    trapezoid_points.push_back(Point(x_right, p2_y)); // P2 (右上)
    trapezoid_points.push_back(Point(x_right, p3_y)); // P3 (右下)
    trapezoid_points.push_back(Point(x_left, p4_y)); // P4 (左下)

    fixed_roi_rect = boundingRect(trapezoid_points);
    fixed_roi_rect = fixed_roi_rect & Rect(0, 0, firstFrame.cols, firstFrame.rows);

    // 检查 fixed_roi_rect 是否为空
    if (fixed_roi_rect.empty() || fixed_roi_rect.width <= 0 || fixed_roi_rect.height <= 0) {
        cerr << "错误：ROI矩形为空或无效，请确保选择的点在图像内部且形成有效区域。" << endl;
        return -1;
    }
    cout << "Fixed ROI Rect: " << fixed_roi_rect << endl;

    destroyWindow("点击选择4点构成ROI");

    // ====================================================================
    // ====== K-means 自适应颜色阈值设置 (当前活跃) ======
    // ====================================================================
    cout << "正在进行K-means自适应颜色阈值分析，请稍候...\n";
    cap.set(CAP_PROP_POS_FRAMES, 0); // 确保从第一帧开始分析

    Mat frame_for_kmeans_analysis;
    cap.read(frame_for_kmeans_analysis); // 读取第一帧供K-means分析
    if (frame_for_kmeans_analysis.empty()) {
        cerr << "无法读取第一帧进行K-means分析，将使用默认HSV阈值。\n";
        // Fallback to default
        hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
        hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
    } else {
        Mat roi_for_analysis_content = frame_for_kmeans_analysis(fixed_roi_rect);
        if (roi_for_analysis_content.empty()) {
            cerr << "K-means分析：裁剪出的ROI为空，将使用默认HSV阈值。\n";
            hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
            hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
        } else {
            Mat hsv_for_analysis;
            cvtColor(roi_for_analysis_content, hsv_for_analysis, COLOR_BGR2HSV);

            // 为 K-means 创建梯形掩码 (相对于 ROI 坐标)
            Mat trapezoid_mask_cropped_size_for_kmeans = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
            vector<Point> relative_trapezoid_points_for_kmeans(4); 
            for(int j = 0; j < 4; ++j) {
                relative_trapezoid_points_for_kmeans[j] = trapezoid_points[j] - fixed_roi_rect.tl();
            }
            fillPoly(trapezoid_mask_cropped_size_for_kmeans, vector<vector<Point>>{relative_trapezoid_points_for_kmeans}, Scalar(255));
            
            // 调用 K-means 函数来设置全局的 hsv_lower 和 hsv_upper
            computeHSVRangeWithKMeans(hsv_for_analysis, trapezoid_mask_cropped_size_for_kmeans, 3); 
        }
    }
    
    // 确保将视频帧位置重置回开头
    cap.set(CAP_PROP_POS_FRAMES, 0); 

    // ====================================================================
    // ====== 直方图法自适应颜色阈值设置 (已注释，用于对比) ======
    // ====================================================================
    /*
    cout << "正在进行直方图法自适应颜色阈值分析，请稍候...\n";
    vector<int> h_values_histogram; // 存储H值用于直方图分析
    int num_frames_for_analysis_histogram = min(50, (int)cap.get(CAP_PROP_FRAME_COUNT)); // 分析前50帧或视频总帧数
    cap.set(CAP_PROP_POS_FRAMES, 0); // 确保从第一帧开始分析

    for (int i = 0; i < num_frames_for_analysis_histogram; ++i) {
        Mat frame_for_analysis_histogram;
        cap.read(frame_for_analysis_histogram);
        if (frame_for_analysis_histogram.empty()) break;

        // 检查 fixed_roi_rect 是否与当前帧交叉，或者是否完全在图像之外
        if (fixed_roi_rect.x >= frame_for_analysis_histogram.cols || fixed_roi_rect.y >= frame_for_analysis_histogram.rows ||
            fixed_roi_rect.x + fixed_roi_rect.width <= 0 || fixed_roi_rect.y + fixed_roi_rect.height <= 0) {
            continue; // ROI 完全在图像之外，跳过此帧
        }

        Mat roi_for_analysis_histogram = frame_for_analysis_histogram(fixed_roi_rect);
        if (roi_for_analysis_histogram.empty()) {
            cerr << "警告：直方图法分析帧 #" << i << " 裁剪出的ROI为空，跳过此帧。\n";
            continue;
        }

        Mat hsv_for_analysis_histogram;
        cvtColor(roi_for_analysis_histogram, hsv_for_analysis_histogram, COLOR_BGR2HSV);

        Mat trapezoid_mask_cropped_size_histogram = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        vector<Point> relative_trapezoid_points_for_display_histogram(4);
        for (int j = 0; j < 4; ++j) {
            relative_trapezoid_points_for_display_histogram[j] = trapezoid_points[j] - fixed_roi_rect.tl();
        }
        fillPoly(trapezoid_mask_cropped_size_histogram, vector<vector<Point>>{relative_trapezoid_points_for_display_histogram}, Scalar(255));

        for (int r = 0; r < hsv_for_analysis_histogram.rows; ++r) {
            for (int c = 0; c < hsv_for_analysis_histogram.cols; ++c) {
                if (trapezoid_mask_cropped_size_histogram.at<uchar>(r, c) > 0) {
                    if (hsv_for_analysis_histogram.at<Vec3b>(r, c)[1] > 20 && hsv_for_analysis_histogram.at<Vec3b>(r, c)[2] > 50) {
                        h_values_histogram.push_back(hsv_for_analysis_histogram.at<Vec3b>(r, c)[0]);
                    }
                }
            }
        }
    }

    if (!h_values_histogram.empty()) {
        Mat h_values_float_mat_histogram(h_values_histogram.size(), 1, CV_32F);
        for (size_t i = 0; i < h_values_histogram.size(); ++i) {
            h_values_float_mat_histogram.at<float>(i, 0) = static_cast<float>(h_values_histogram[i]);
        }

        if (h_values_float_mat_histogram.empty()) {
            cerr << "警告：直方图法转换后的 h_values_float_mat 为空，无法计算直方图。\n";
            // Fallback to default, already handled by the K-means section if it's uncommented
        } else {
            Mat hist_histogram;
            int histSize_histogram = 181;
            float h_ranges_histogram[] = { 0, 181 };
            int channels_histogram[] = { 0 };
            int hist_sizes_histogram[] = { histSize_histogram };
            const float* ranges_ptr_histogram[] = { h_ranges_histogram };

            // 计算H通道直方图
            calcHist(&h_values_float_mat_histogram, 1, channels_histogram, Mat(), hist_histogram, 1, hist_sizes_histogram, ranges_ptr_histogram, true, false);

            int h_peak_min_histogram = -1, h_peak_max_histogram = -1;
            float max_hist_val_histogram = 0;

            int green_h_start_guess_histogram = 15; 
            int green_h_end_guess_histogram = 120;  

            for (int h = 0; h < histSize_histogram; ++h) {
                float current_hist_val = hist_histogram.at<float>(h);
                if (h >= green_h_start_guess_histogram && h <= green_h_end_guess_histogram) {
                    if (current_hist_val > max_hist_val_histogram) {
                        max_hist_val_histogram = current_hist_val;
                        h_peak_min_histogram = h;
                        h_peak_max_histogram = h;
                    }
                }
            }

            if (h_peak_min_histogram != -1) {
                int initial_peak_h_histogram = h_peak_min_histogram;
                for (int h = initial_peak_h_histogram - 1; h >= green_h_start_guess_histogram; --h) {
                    if (hist_histogram.at<float>(h) > max_hist_val_histogram * 0.07) { 
                        h_peak_min_histogram = h;
                    } else {
                        break;
                    }
                }
                for (int h = initial_peak_h_histogram + 1; h <= green_h_end_guess_histogram; ++h) {
                    if (hist_histogram.at<float>(h) > max_hist_val_histogram * 0.07) { 
                        h_peak_max_histogram = h;
                    } else {
                        break;
                    }
                }
                // 使用直方图法的结果更新全局HSV阈值
                hsv_lower.at<int>(0) = max(0, h_peak_min_histogram - 5);
                hsv_upper.at<int>(0) = min(180, h_peak_max_histogram + 5);

                hsv_lower.at<int>(1) = max(0, hsv_lower.at<int>(1) > 0 ? hsv_lower.at<int>(1) : 43); // Smin
                hsv_lower.at<int>(2) = max(0, hsv_lower.at<int>(2) > 0 ? hsv_lower.at<int>(2) : 46); // Vmin
                // Vmax 和 Smax 一般直接取255
                hsv_upper.at<int>(1) = 255;
                hsv_upper.at<int>(2) = 255;

                cout << "直方图法自适应HSV阈值已设置：\nHmin: " << hsv_lower.at<int>(0) << ", Hmax: " << hsv_upper.at<int>(0) << endl;
                cout << "Smin: " << hsv_lower.at<int>(1) << ", Vmin: " << hsv_lower.at<int>(2) << endl;
            } else {
                cout << "直方图法：未能在分析帧中检测到明显的绿色峰值，使用默认HSV阈值。\n";
                hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
                hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
            }
        }
    } else {
        cout << "直方图法：无法进行Hsv分析，ROI区域内没有足够的有效像素，使用默认HSV阈值。\n";
        hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
        hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
    }
    // 确保将视频帧位置重置回开头
    cap.set(CAP_PROP_POS_FRAMES, 0); 
    */
    // =================================== 结束直方图法 ===================================

    namedWindow("调节参数", WINDOW_NORMAL);
    createTrackbar("Hmin", "调节参数", &hsv_lower.at<int>(0), 180);
    createTrackbar("Smin", "调节参数", &hsv_lower.at<int>(1), 255);
    createTrackbar("Vmin", "调节参数", &hsv_lower.at<int>(2), 255);
    createTrackbar("Hmax", "调节参数", &hsv_upper.at<int>(0), 180);
    createTrackbar("Smax", "调节参数", &hsv_upper.at<int>(1), 255);
    createTrackbar("Vmax", "调节参数", &hsv_upper.at<int>(2), 255);
    createTrackbar("Amin", "调节参数", &area_min, 5000);
    createTrackbar("Amax", "调节参数", &area_max, 30000);

    // 根据自适应结果更新Trackbar初始值
    setTrackbarPos("Hmin", "调节参数", hsv_lower.at<int>(0));
    setTrackbarPos("Smin", "调节参数", hsv_lower.at<int>(1));
    setTrackbarPos("Vmin", "调节参数", hsv_lower.at<int>(2));
    setTrackbarPos("Hmax", "调节参数", hsv_upper.at<int>(0));
    setTrackbarPos("Smax", "调节参数", hsv_upper.at<int>(1));
    setTrackbarPos("Vmax", "调节参数", hsv_upper.at<int>(2));

    Size roi_video_size = fixed_roi_rect.size();
    // 尝试不同的 FourCC 编码器
    int fourcc_codec = VideoWriter::fourcc('D', 'I', 'V', 'X'); // 尝试 DIVX，通常兼容性较好
    // int fourcc_codec = VideoWriter::fourcc('M', 'J', 'P', 'G'); // 您原来的
    // int fourcc_codec = VideoWriter::fourcc('X', 'V', 'I', 'D'); // 尝试 XVID

    VideoWriter roi_writer("roi_output.avi", fourcc_codec, 30, roi_video_size);
    // detect_writer 的尺寸是原始帧的尺寸
    VideoWriter detect_writer("detect_output.avi", fourcc_codec, 30, firstFrame.size());
    // comparison_writer 的尺寸是 ROI 尺寸的两倍宽
    VideoWriter comparison_writer("comparison_output.avi", fourcc_codec, 30, Size(roi_video_size.width * 2, roi_video_size.height));

    if (!roi_writer.isOpened() || !detect_writer.isOpened() || !comparison_writer.isOpened()) {
        cerr << "无法创建视频写入器，请检查FourCC编码器或FFmpeg支持。" << endl;
        // 如果 FourCC 失败，尝试打印可用的编解码器
        // vector<string> backends = VideoWriter::getDefaultEngines();
        // for(const auto& b : backends) cerr << "Available backend: " << b << endl;
        return -1;
    }

    Mat frame, hsv_frame, processed_roi_mask;
    Mat white_mask;
    // previous_cumulative_mask 仅用于可视化，不再用于计数逻辑累积，因为它会被 TrackedObject 的 has_entered_core 替代。
    // 但是，为了保持伪色图和新区域的显示逻辑，我们仍然需要它。
    Mat previous_cumulative_mask = Mat::zeros(fixed_roi_rect.size(), CV_8UC1); 

    int frame_idx = 0;

    // 核心区域的相对坐标计算，用于 updateTrackedObjects
    // 1. 定义核心区域 (X轴方向的三等分，取中间1/3)
    int x_line_left = trapezoid_points[0].x - fixed_roi_rect.tl().x;
    int x_line_right = trapezoid_points[1].x - fixed_roi_rect.tl().x;
    int core_x_start = x_line_left + (x_line_right - x_line_left) / 3;
    int core_x_end = x_line_left + 2 * (x_line_right - x_line_left) / 3;
    Rect core_region_relative_roi(core_x_start, 0, core_x_end - core_x_start, fixed_roi_rect.height);

    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        Mat roi_cropped_content = frame(fixed_roi_rect).clone();

        vector<Point> relative_trapezoid_points_display(4);
        for (int i = 0; i < 4; ++i) {
            relative_trapezoid_points_display[i] = trapezoid_points[i] - fixed_roi_rect.tl();
        }
        // ROI轮廓线粗细调整为1
        polylines(roi_cropped_content, vector<vector<Point>>{relative_trapezoid_points_display}, true, Scalar(0, 255, 255), 1);
        
        imshow("裁剪后的原始ROI区域", roi_cropped_content);

        Mat current_roi_hsv_input = frame(fixed_roi_rect).clone();
        cvtColor(current_roi_hsv_input, hsv_frame, COLOR_BGR2HSV);

        // 从全局变量 hsv_lower 和 hsv_upper 中获取实时（可能是用户微调过的）阈值
        Scalar lower(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2));
        Scalar upper(hsv_upper.at<int>(0), hsv_upper.at<int>(1), hsv_upper.at<int>(2));

        white_mask = excludeWhiteLight(hsv_frame);
        inRange(hsv_frame, lower, upper, processed_roi_mask);
        processed_roi_mask.setTo(0, white_mask); // 排除白色光干扰

        Mat trapezoid_mask_cropped_size = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        fillPoly(trapezoid_mask_cropped_size, vector<vector<Point>>{relative_trapezoid_points_display}, Scalar(255));
        bitwise_and(processed_roi_mask, trapezoid_mask_cropped_size, processed_roi_mask);

        processed_roi_mask = applyMorphology(processed_roi_mask);

        imshow("1. Current Mask (ROI size)", processed_roi_mask);

        // ========== 新的追踪和计数逻辑 ==========
        vector<vector<Point>> current_frame_contours_raw;
        findContours(processed_roi_mask, current_frame_contours_raw, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<pair<Point2f, vector<Point>>> current_detected_objects; // 存储当前帧检测到的有效线粒体信息
        for (const auto& contour : current_frame_contours_raw) {
            double area = contourArea(contour);
            if (area >= area_min && area <= area_max) {
                Moments m = moments(contour);
                Point2f centroid;
                if (m.m00 != 0) {
                    centroid = Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
                    current_detected_objects.push_back({centroid, contour});
                }
            }
        }
        
        // 存储本帧新计数的线粒体的信息 (质心、面积、轮廓)
        vector<tuple<Point2f, double, vector<Point>>> new_counted_mitochondria_infos; 
        // 调用追踪更新函数
        // 这里的 max_distance 对应您之前的 max_centroid_distance
        updateTrackedObjects(current_detected_objects, core_region_relative_roi, 20.0, 10, new_counted_mitochondria_infos);

        // 根据新计数的线粒体信息，生成 new_pixel_mask 和 total_new_area
        Mat new_pixel_mask = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        double current_frame_total_new_area = 0.0;
        vector<vector<Point>> detected_inner_contours_for_display; // 只用于当前帧的显示

        for(const auto& info_tuple : new_counted_mitochondria_infos) {
            Point2f centroid_info = get<0>(info_tuple);
            double area_info = get<1>(info_tuple);
            vector<Point> contour_info = get<2>(info_tuple);

            current_frame_total_new_area += area_info;
            // 绘制到 new_pixel_mask，用于伪色显示
            drawContours(new_pixel_mask, vector<vector<Point>>{contour_info}, 0, Scalar(255), FILLED);
            detected_inner_contours_for_display.push_back(contour_info);
        }

        area_history.push_back(current_frame_total_new_area); // 历史记录存储的是每帧新计数的总面积

        // previous_cumulative_mask 逻辑调整：
        // 1. 对于新的计数，添加到 previous_cumulative_mask（用于累积显示）
        // 2. 对于已追踪但未新计数的，可以更新其在 previous_cumulative_mask 上的位置（如果需要持续显示所有已追踪目标）
        // 这里为了简化，我们只让 new_pixel_mask 累积到 previous_cumulative_mask
        bitwise_or(previous_cumulative_mask, new_pixel_mask, previous_cumulative_mask);

        imshow("2. New Mask (ROI size)", new_pixel_mask); // 红色高亮部分
        imshow("3. Previous Mask (ROI size)", previous_cumulative_mask); // 累积的已计数线粒体

        overlayText(frame, "New Green Area: " + to_string(static_cast<int>(current_frame_total_new_area)) + " px");

        Mat red_highlight_full = Mat::zeros(frame.size(), CV_8UC3);
        Mat red_roi_part = red_highlight_full(fixed_roi_rect);
        red_roi_part.setTo(Scalar(0, 0, 255), new_pixel_mask); // 在 ROI 范围内，将新检测的像素设为红色

        Mat pseudo_result_full = frame.clone();
        Mat frame_roi_clone = pseudo_result_full(fixed_roi_rect);
        addWeighted(frame_roi_clone, 0.5, red_roi_part, 0.5, 0.0, frame_roi_clone);

        imshow("伪色检测结果", pseudo_result_full);
        imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));

        // ========== 新增：保存图片和输出信息 ==========
        if (!new_counted_mitochondria_infos.empty()) { // 如果当前帧有新的线粒体被计数
            cout << "帧 " << frame_idx << " - 检测到 " << new_counted_mitochondria_infos.size() << " 个新线粒体。\n";
            string areas_str = "面积: ";
            for (size_t i = 0; i < new_counted_mitochondria_infos.size(); ++i) {
                areas_str += to_string(static_cast<int>(get<1>(new_counted_mitochondria_infos[i])));
                if (i < new_counted_mitochondria_infos.size() - 1) areas_str += ", ";
            }
            cout << areas_str << endl;

            Mat frame_with_yellow_contours = frame.clone();
            // 注意：detected_inner_contours_for_display 的点是相对于 ROI 区域的，需要加回 ROI 的偏移量
            vector<vector<Point>> global_contours;
            for (const auto& roi_contour : detected_inner_contours_for_display) {
                vector<Point> global_contour;
                for (const auto& pt : roi_contour) {
                    global_contour.push_back(pt + fixed_roi_rect.tl());
                }
                global_contours.push_back(global_contour);
            }
            drawContours(frame_with_yellow_contours, global_contours, -1, Scalar(0, 255, 255), 2); // 黄色轮廓，粗细为2

            // 添加文本说明到图片上方便查看
            string text_frame_idx = "Frame: " + to_string(frame_idx);
            string text_count = "Count: " + to_string(new_counted_mitochondria_infos.size());
            string text_total_area = "Total Area: " + to_string(static_cast<int>(current_frame_total_new_area)) + " px";

            putText(frame_with_yellow_contours, text_frame_idx, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            putText(frame_with_yellow_contours, text_count, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            putText(frame_with_yellow_contours, text_total_area, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);

            string all_areas_text = "Areas: ";
            for (const auto& info_tuple : new_counted_mitochondria_infos) {
                all_areas_text += to_string(static_cast<int>(get<1>(info_tuple))) + " ";
            }
            // 文本可能过长，需要分行或限制长度
            if (all_areas_text.length() > 50) all_areas_text = all_areas_text.substr(0, 47) + "..."; // 截断以免出界
            putText(frame_with_yellow_contours, all_areas_text, Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 1);

            string filename_base = "frame_" + to_string(frame_idx) + "_count_" + to_string(new_counted_mitochondria_infos.size()) + "_area_" + to_string(static_cast<int>(current_frame_total_new_area));
            imwrite("output/" + filename_base + "_yellow_contours.png", frame_with_yellow_contours);
            imwrite("output/" + filename_base + "_pseudo.png", pseudo_result_full);
        }
        // ========== 结束图片保存 ==========

        imwrite("output/plot_" + to_string(frame_idx) + ".png", generateAreaPlot(area_history, 400, 200));
        imwrite("masks/mask_" + to_string(frame_idx) + ".png", processed_roi_mask);
        imwrite("newmasks/newmask_" + to_string(frame_idx) + ".png", new_pixel_mask);

        csvFile << frame_idx << "," << current_frame_total_new_area << "\n";

        roi_writer.write(roi_cropped_content);
        detect_writer.write(pseudo_result_full);

        Mat pseudo_result_cropped_roi = frame(fixed_roi_rect).clone();
        Mat red_roi_highlight_cropped = Mat::zeros(roi_video_size.height, roi_video_size.width, CV_8UC3);
        red_roi_highlight_cropped.setTo(Scalar(0, 0, 255), new_pixel_mask);

        addWeighted(pseudo_result_cropped_roi, 0.5, red_roi_highlight_cropped, 0.5, 0.0, pseudo_result_cropped_roi);

        Mat combined_roi_frame;
        hconcat(roi_cropped_content, pseudo_result_cropped_roi, combined_roi_frame);

        imshow("ROI对比视图 (原始 vs 伪色)", combined_roi_frame);
        comparison_writer.write(combined_roi_frame);

        frame_idx++;

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    roi_writer.release();
    detect_writer.release();
    comparison_writer.release();
    csvFile.close();
    destroyAllWindows();

    double total_area_sum = accumulate(area_history.begin(), area_history.end(), 0.0); 
    cout << "==============================\n";
    cout << "Total Green Area accumulated over count: " << total_area_sum << " px\n"; 
    cout << "==============================\n";

    return 0;
}

// ========== 函数定义 ==========
void mouseCallback(int event, int x, int y, int, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && trapezoid_points.size() < 4) {
        trapezoid_points.push_back(Point(x, y));
        circle(*(Mat*)userdata, Point(x, y), 5, Scalar(0, 0, 255), FILLED);
        imshow("点击选择4点构成ROI", *(Mat*)userdata);
    }
}

Mat createTrapezoidMask(Size size, const vector<Point>& points_input) {
    Mat mask = Mat::zeros(size, CV_8UC1);
    vector<vector<Point>> contour{ points_input };
    fillPoly(mask, contour, Scalar(255));
    return mask;
}

Mat excludeWhiteLight(const Mat& hsv) {
    Scalar lower_white(0, 0, 200);   
    Scalar upper_white(180, 25, 255); 
    Mat white_mask;
    inRange(hsv, lower_white, upper_white, white_mask);
    return white_mask;
}

Mat applyMorphology(const Mat& mask) {
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat result;
    morphologyEx(mask, result, MORPH_OPEN, kernel, Point(-1, -1), 1);
    morphologyEx(result, result, MORPH_CLOSE, kernel, Point(-1, -1), 1); 
    return result;
}

// 辅助函数：计算质心距离
float centroidDist(const Point2f& a, const Point2f& b) {
    return static_cast<float>(sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
}

// 新的追踪更新函数实现
void updateTrackedObjects(const vector<pair<Point2f, vector<Point>>>& current_detected_objects, Rect coreRegion,
                          double max_distance, int max_inactive_frames,
                          vector<tuple<Point2f, double, vector<Point>>>& new_counted_mitochondria_infos) {
    
    vector<bool> matched_current_frame_detections(current_detected_objects.size(), false);
    
    // Step 1: 尝试将当前帧的检测与现有追踪对象匹配
    for (auto& obj : trackedObjects) {
        double minDist = max_distance + 1.0; // 初始化为比最大距离更大的值
        int bestIdx = -1;

        for (size_t i = 0; i < current_detected_objects.size(); ++i) {
            if (matched_current_frame_detections[i]) continue; // 这个检测已经被匹配过了

            float dist = centroidDist(obj.centroid, current_detected_objects[i].first);
            if (dist < minDist && dist <= max_distance) { // 找到一个更近的匹配
                minDist = dist;
                bestIdx = static_cast<int>(i);
            }
        }

        if (bestIdx != -1) {
            // 匹配成功，更新追踪对象信息
            obj.centroid = current_detected_objects[bestIdx].first;
            obj.contour = current_detected_objects[bestIdx].second; // 更新轮廓信息
            obj.frames_since_seen = 0; // 重置未见帧数
            matched_current_frame_detections[bestIdx] = true; // 标记此检测已匹配

            // 如果该目标是第一次进入核心区域，则计数
            if (!obj.has_entered_core && coreRegion.contains(obj.centroid)) {
                obj.has_entered_core = true;
                // 记录新计数的线粒体的质心、面积和轮廓
                new_counted_mitochondria_infos.emplace_back(obj.centroid, contourArea(obj.contour), obj.contour);
            }
        } else {
            // 没有匹配到，增加未见帧数
            obj.frames_since_seen++;
        }
    }

    // Step 2: 添加新的未匹配到的检测为新的追踪对象
    for (size_t i = 0; i < current_detected_objects.size(); ++i) {
        if (!matched_current_frame_detections[i]) {
            TrackedObject newObj;
            newObj.id = nextTrackID++; // 分配新的追踪ID
            newObj.centroid = current_detected_objects[i].first;
            newObj.contour = current_detected_objects[i].second; // 保存轮廓
            newObj.frames_since_seen = 0;
            // 如果新对象首次出现就在核心区域内，也立即计数
            newObj.has_entered_core = coreRegion.contains(newObj.centroid);
            if (newObj.has_entered_core) {
                new_counted_mitochondria_infos.emplace_back(newObj.centroid, contourArea(newObj.contour), newObj.contour);
            }
            trackedObjects.push_back(newObj);
        }
    }

    // Step 3: 移除长时间未被发现的追踪对象
    // 使用 erase-remove idiom 移除符合条件的元素
    trackedObjects.erase(
        remove_if(trackedObjects.begin(), trackedObjects.end(), 
            [max_inactive_frames](const TrackedObject& obj) {
                return obj.frames_since_seen > max_inactive_frames;
            }),
        trackedObjects.end());
}

// generateAreaPlot, overlayText, computeHSVRangeWithKMeans 保持不变
Mat generateAreaPlot(const vector<double>& history, int width, int height) {
    Mat plot = Mat::zeros(height, width, CV_8UC3);
    if (history.empty()) return plot;

    double max_val = *max_element(history.begin(), history.end());
    double min_val = *min_element(history.begin(), history.end());

    max_val = max(1.0, max_val);
    if (abs(max_val - min_val) < 1e-6) min_val = 0.0; 

    double range = max_val - min_val;
    if (abs(range) < 1e-6) range = 1.0; 

    size_t count = history.size();
    if (count < 2) {
        Point pt(static_cast<int>((width - 1.0) / 2.0),
            height - 1 - static_cast<int>((history.back() - min_val) * (height - 1.0) / range));
        circle(plot, pt, 3, Scalar(0, 0, 255), FILLED);
        putText(plot, "Current: " + to_string(static_cast<int>(history.back())), Point(5, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
        return plot;
    }

    line(plot, Point(0, height - 1), Point(width - 1, height - 1), Scalar(255, 255, 255), 1);
    line(plot, Point(0, 0), Point(0, height - 1), Scalar(255, 255, 255), 1);

    for (size_t i = 1; i < count; ++i) {
        Point pt1(static_cast<int>((i - 1) * (width - 1.0) / (count - 1.0)),
            height - 1 - static_cast<int>((history[i - 1] - min_val) * (height - 1.0) / range));
        Point pt2(static_cast<int>(i * (width - 1.0) / (count - 1.0)),
            height - 1 - static_cast<int>((history[i] - min_val) * (height - 1.0) / range));

        line(plot, pt1, pt2, Scalar(0, 255, 0), 1);
    }

    Point last_pt(static_cast<int>((count - 1) * (width - 1.0) / (count - 1.0)),
        height - 1 - static_cast<int>((history.back() - min_val) * (height - 1.0) / range));
    circle(plot, last_pt, 3, Scalar(0, 0, 255), FILLED);

    putText(plot, "Max: " + to_string(static_cast<int>(max_val)), Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
    putText(plot, "Current: " + to_string(static_cast<int>(history.back())), Point(5, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

    return plot;
}

void overlayText(Mat& frame, const string& text) {
    putText(frame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
}

Scalar computeHSVRangeWithKMeans(const Mat& hsv_roi, const Mat& mask, int k_clusters) { 
    vector<Vec3f> hsv_pixels;

    for (int y = 0; y < hsv_roi.rows; ++y) {
        for (int x = 0; x < hsv_roi.cols; ++x) {
            if (mask.at<uchar>(y, x) > 0) {
                Vec3b pixel = hsv_roi.at<Vec3b>(y, x);
                if (pixel[1] > 30 && pixel[2] > 50) { 
                    hsv_pixels.emplace_back(static_cast<float>(pixel[0]), static_cast<float>(pixel[1]), static_cast<float>(pixel[2]));
                }
            }
        }
    }

    if (hsv_pixels.empty()) {
        cout << "K-means：未提取到有效HSV像素，将使用默认阈值。\n";
        hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
        hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
        return Scalar(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2)); 
    }

    Mat samples((int)hsv_pixels.size(), 3, CV_32F, hsv_pixels.data());
    samples = samples.clone(); 

    Mat labels, centers;
    kmeans(samples, k_clusters, labels,
        TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 100, 1.0),
        3, KMEANS_PP_CENTERS, centers);

    int green_index = -1;
    float green_score_max = -1.0f;
    for (int i = 0; i < centers.rows; ++i) {
        float h = centers.at<float>(i, 0);
        float s = centers.at<float>(i, 1);
        float v = centers.at<float>(i, 2);

        float green_score = 0.0f;
        if (h >= 40 && h <= 90) { 
           green_score = (s / 255.0f) * (v / 255.0f); 
        }
        green_score *= (1.0f - abs(h - 65.0f) / 65.0f); 

        if (green_score > green_score_max) {
            green_score_max = green_score;
            green_index = i;
        }
    }

    if (green_index == -1 || green_score_max < 0.1) { 
        cout << "K-means：未找到合适的绿色聚类（得分不足），将使用默认阈值。\n";
        hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
        hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
        return Scalar(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2)); 
    }

    float h_center = centers.at<float>(green_index, 0);
    float s_center = centers.at<float>(green_index, 1);
    float v_center = centers.at<float>(green_index, 2);

    int hmin = max(0, int(h_center - 15)); 
    int hmax = min(180, int(h_center + 15));
    int smin = max(20, int(s_center - 50)); 
    int smax = min(255, int(s_center + 50));
    int vmin = max(30, int(v_center - 50)); 
    int vmax = min(255, int(v_center + 50));

    hmin = max(hmin, 20); 
    hmax = min(hmax, 100); 

    hsv_lower.at<int>(0) = hmin;
    hsv_lower.at<int>(1) = smin;
    hsv_lower.at<int>(2) = vmin;
    hsv_upper.at<int>(0) = hmax;
    hsv_upper.at<int>(1) = smax;
    hsv_upper.at<int>(2) = vmax;

    cout << "K-means 自动设定 HSV阈值：" << endl;
    cout << "H: [" << hmin << ", " << hmax << "]\n";
    cout << "S: [" << smin << ", " << smax << "]\n";
    cout << "V: [" << vmin << ", " << vmax << "]\n";

    return Scalar(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2)); 
}
