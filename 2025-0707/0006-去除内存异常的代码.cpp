# 自适应的处理检测绿色荧光线粒体有疏漏，只检测到了一次

#include <iostream>
#include <string>
#include <fstream>
#include <numeric>
#include <algorithm> 
#include <vector>    
#include <map>       // For tracking individual mitochondria
#include <cmath>     // For std::isnan

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// ========== 全局变量 ==========
vector<double> area_history;
vector<Point> trapezoid_points;
Mat hsv_lower = (Mat_<int>(1, 3) << 0, 0, 0); // 初始值设为0，由自适应调整
Mat hsv_upper = (Mat_<int>(1, 3) << 180, 255, 255); // 初始值设为最大，由自适应调整
int area_min = 50;
int area_max = 20000;

// 定义裁剪区域的矩形，一旦确定，视频输出尺寸就固定了
Rect fixed_roi_rect;

// Track individual mitochondria (simple tracking using centroid distance)
struct MitochondrionState {
    Point2f centroid;
    bool has_passed_core_and_counted; // true if this mitochondrion has been counted while in core region
};
map<int, MitochondrionState> tracked_mitochondria;
int next_object_id = 0;
double max_centroid_distance = 20.0; // Max distance for centroids to be considered the same object between frames

// ========== 函数声明 ==========
void mouseCallback(int event, int x, int y, int, void* userdata);
Mat createTrapezoidMask(Size size, const vector<Point>& points_input);
Mat excludeWhiteLight(const Mat& hsv);
Mat applyMorphology(const Mat& mask);
double computeNewArea(const Mat& processed_roi_mask, Mat& previous_cumulative_mask, Mat& new_pixel_mask,
    const vector<Point>& trapezoid_pts_rel_roi, const Rect& roi_rect);
Mat generateAreaPlot(const vector<double>& history, int width, int height);
void overlayText(Mat& frame, const string& text);

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

    int x_left = raw_points[0].x, x_right = raw_points[0].x;
    for (const auto& p : raw_points) {
        x_left = min(x_left, p.x);
        x_right = max(x_right, p.x);
    }

    sort(raw_points.begin(), raw_points.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
        });
    vector<Point> left_points = { raw_points[0], raw_points[1] };
    vector<Point> right_points = { raw_points[2], raw_points[3] };

    int p1_y = min(left_points[0].y, left_points[1].y);
    int p4_y = max(left_points[0].y, left_points[1].y);

    int p2_y = min(right_points[0].y, right_points[1].y);
    int p3_y = max(right_points[0].y, right_points[1].y);

    trapezoid_points.clear();
    trapezoid_points.push_back(Point(x_left, p1_y)); // P1 (左上)
    trapezoid_points.push_back(Point(x_right, p2_y)); // P2 (右上)
    trapezoid_points.push_back(Point(x_right, p3_y)); // P3 (右下)
    trapezoid_points.push_back(Point(x_left, p4_y)); // P4 (左下)

    fixed_roi_rect = boundingRect(trapezoid_points);
    fixed_roi_rect = fixed_roi_rect & Rect(0, 0, firstFrame.cols, firstFrame.rows);

    // 检查 fixed_roi_rect 是否为空
    if (fixed_roi_rect.empty()) {
        cerr << "错误：ROI矩形为空，请确保选择的点在图像内部且形成有效区域。" << endl;
        return -1;
    }
    cout << "Fixed ROI Rect: " << fixed_roi_rect << endl;

    destroyWindow("点击选择4点构成ROI");

    // ========== 自适应颜色阈值设置 ==========
    cout << "正在进行自适应颜色阈值分析，请稍候...\n";
    vector<int> h_values;
    int num_frames_for_analysis = min(50, (int)cap.get(CAP_PROP_FRAME_COUNT)); // 分析前50帧或视频总帧数
    cap.set(CAP_PROP_POS_FRAMES, 0); // 确保从第一帧开始分析

    for (int i = 0; i < num_frames_for_analysis; ++i) {
        Mat frame_for_analysis;
        cap.read(frame_for_analysis);
        if (frame_for_analysis.empty()) break; // 如果视频短于 num_frames_for_analysis，将提前退出

        // 检查 fixed_roi_rect 是否与当前帧交叉，或者是否完全在图像之外
        if (fixed_roi_rect.x >= frame_for_analysis.cols || fixed_roi_rect.y >= frame_for_analysis.rows ||
            fixed_roi_rect.x + fixed_roi_rect.width <= 0 || fixed_roi_rect.y + fixed_roi_rect.height <= 0) {
            continue; // ROI 完全在图像之外，跳过此帧
        }

        Mat roi_for_analysis = frame_for_analysis(fixed_roi_rect);
        if (roi_for_analysis.empty()) { // 如果裁剪出的ROI是空的，可能是fixed_roi_rect与frame不交叉
            cerr << "警告：分析帧 #" << i << " 裁剪出的ROI为空，跳过此帧。\n";
            continue;
        }

        Mat hsv_for_analysis;
        cvtColor(roi_for_analysis, hsv_for_analysis, COLOR_BGR2HSV);

        Mat trapezoid_mask_cropped_size = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        vector<Point> relative_trapezoid_points_for_display(4);
        for (int j = 0; j < 4; ++j) {
            relative_trapezoid_points_for_display[j] = trapezoid_points[j] - fixed_roi_rect.tl();
        }
        fillPoly(trapezoid_mask_cropped_size, vector<vector<Point>>{relative_trapezoid_points_for_display}, Scalar(255));

        for (int r = 0; r < hsv_for_analysis.rows; ++r) {
            for (int c = 0; c < hsv_for_analysis.cols; ++c) {
                if (trapezoid_mask_cropped_size.at<uchar>(r, c) > 0) { // 只分析梯形内部的像素
                    // 确保像素是较亮的或者有饱和度
                    if (hsv_for_analysis.at<Vec3b>(r, c)[1] > 20 && hsv_for_analysis.at<Vec3b>(r, c)[2] > 50) {
                        h_values.push_back(hsv_for_analysis.at<Vec3b>(r, c)[0]);
                    }
                }
            }
        }
    }

    Mat hist;
    int histSize = 181;
    float h_ranges[] = { 0, 181 }; // H通道的范围

    if (!h_values.empty()) {
        // 将 h_values 向量转换为 cv::Mat
        // 确保 Mat 的类型是 float，因为 calcHist 通常对浮点型数据操作更稳定
        Mat h_values_float_mat(h_values.size(), 1, CV_32F);
        for (size_t i = 0; i < h_values.size(); ++i) {
            h_values_float_mat.at<float>(i, 0) = static_cast<float>(h_values[i]);
        }

        // 再次检查 h_values_float_mat 是否为空（尽管在h_values不为空的情况下通常不会）
        if (h_values_float_mat.empty()) {
            cerr << "警告：转换后的 h_values_float_mat 为空，无法计算直方图。\n";
            // 这里将直接进入 else 块，使用默认值
            goto end_adaptive_setup; // 跳到自适应设置结束的地方
        }

        // 定义 calcHist 的通道和直方图参数
        int channels[] = { 0 };       // 我们对 h_values_float_mat 的第0个通道进行直方图计算
        int hist_sizes[] = { histSize }; // 直方图的每个维度的大小
        const float* ranges_ptr[] = { h_ranges }; // 每个通道的范围

        // 调用 calcHist
        // 参数: images (Mat*), nimages (int), channels (int*), mask (InputArray), hist (OutputArray),
        //       dims (int), histSize (int*), ranges (const float**), uniform (bool), accumulate (bool)
        calcHist(&h_values_float_mat, 1, channels, Mat(), hist, 1, hist_sizes, ranges_ptr, true, false);

        int h_peak_min = -1, h_peak_max = -1;
        float max_hist_val = 0;

        int green_h_start_guess = 25;
        int green_h_end_guess = 95;

        for (int h = 0; h < histSize; ++h) {
            float current_hist_val = hist.at<float>(h);
            if (h >= green_h_start_guess && h <= green_h_end_guess) {
                if (current_hist_val > max_hist_val) {
                    max_hist_val = current_hist_val;
                    h_peak_min = h;
                    h_peak_max = h;
                }
            }
        }

        if (h_peak_min != -1) {
            int initial_peak_h = h_peak_min;
            for (int h = initial_peak_h - 1; h >= green_h_start_guess; --h) {
                if (hist.at<float>(h) > max_hist_val * 0.1) {
                    h_peak_min = h;
                }
                else {
                    break;
                }
            }
            for (int h = initial_peak_h + 1; h <= green_h_end_guess; ++h) {
                if (hist.at<float>(h) > max_hist_val * 0.1) {
                    h_peak_max = h;
                }
                else {
                    break;
                }
            }

            hsv_lower.at<int>(0) = max(0, h_peak_min - 5);
            hsv_upper.at<int>(0) = min(180, h_peak_max + 5);

            // 确保S和V的最小值不会太低，除非用户设定为0
            hsv_lower.at<int>(1) = max(0, hsv_lower.at<int>(1) > 0 ? hsv_lower.at<int>(1) : 43);
            hsv_lower.at<int>(2) = max(0, hsv_lower.at<int>(2) > 0 ? hsv_lower.at<int>(2) : 46);

            cout << "自适应HSV阈值已设置：\nHmin: " << hsv_lower.at<int>(0) << ", Hmax: " << hsv_upper.at<int>(0) << endl;
            cout << "Smin: " << hsv_lower.at<int>(1) << ", Vmin: " << hsv_lower.at<int>(2) << endl;
        }
        else {
            cout << "未能在分析帧中检测到明显的绿色峰值，使用默认HSV阈值。\n";
            hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
            hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
        }
    }
    else {
        cout << "无法进行Hsv分析，ROI区域内没有足够的有效像素，使用默认HSV阈值。\n";
        hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
        hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
    }

end_adaptive_setup:; // 用于 goto 语句的标签
    cap.set(CAP_PROP_POS_FRAMES, 0); // 确保将视频帧位置重置回开头

    namedWindow("调节参数", WINDOW_NORMAL);
    createTrackbar("Hmin", "调节参数", &hsv_lower.at<int>(0), 180);
    createTrackbar("Smin", "调节参数", &hsv_lower.at<int>(1), 255);
    createTrackbar("Vmin", "调节参数", &hsv_lower.at<int>(2), 255);
    createTrackbar("Hmax", "调节参数", &hsv_upper.at<int>(0), 180);
    createTrackbar("Smax", "调节参数", &hsv_upper.at<int>(1), 255);
    createTrackbar("Vmax", "调节参数", &hsv_upper.at<int>(2), 255);
    createTrackbar("Amin", "调节参数", &area_min, 5000);
    createTrackbar("Amax", "调节参数", &area_max, 30000);

    // 根据自适应结果更新Trackbar初始值 (这句代码要放在createTrackbar之后)
    setTrackbarPos("Hmin", "调节参数", hsv_lower.at<int>(0));
    setTrackbarPos("Smin", "调节参数", hsv_lower.at<int>(1));
    setTrackbarPos("Vmin", "调节参数", hsv_lower.at<int>(2));
    setTrackbarPos("Hmax", "调节参数", hsv_upper.at<int>(0));
    setTrackbarPos("Smax", "调节参数", hsv_upper.at<int>(1));
    setTrackbarPos("Vmax", "调节参数", hsv_upper.at<int>(2));

    Size roi_video_size = fixed_roi_rect.size();
    VideoWriter roi_writer("roi_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, roi_video_size);
    VideoWriter detect_writer("detect_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, firstFrame.size());
    // 水平拼接的视频尺寸：宽度*2, 高度不变
    VideoWriter comparison_writer("comparison_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, Size(roi_video_size.width * 2, roi_video_size.height));

    if (!roi_writer.isOpened() || !detect_writer.isOpened() || !comparison_writer.isOpened()) {
        cerr << "无法创建视频写入器" << endl;
        return -1;
    }

    Mat frame, hsv_frame, processed_roi_mask;
    Mat white_mask;
    Mat previous_cumulative_mask; // 保持累积的、已计算的像素

    int frame_idx = 0;

    vector<Point> relative_trapezoid_points_for_roi(4);
    for (int i = 0; i < 4; ++i) {
        relative_trapezoid_points_for_roi[i] = trapezoid_points[i] - fixed_roi_rect.tl();
    }

    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        Mat trapezoid_mask_full = createTrapezoidMask(frame.size(), trapezoid_points);

        Mat roi_cropped_content = frame(fixed_roi_rect).clone();

        vector<Point> relative_trapezoid_points_display(4);
        for (int i = 0; i < 4; ++i) {
            relative_trapezoid_points_display[i] = trapezoid_points[i] - fixed_roi_rect.tl();
        }
        polylines(roi_cropped_content, vector<vector<Point>>{relative_trapezoid_points_display}, true, Scalar(0, 255, 255), 2);

        // 绘制核心区域辅助线，在 roi_cropped_content 上
        int x_line_left_roi = relative_trapezoid_points_for_roi[0].x; // 在ROI坐标系下
        int x_line_right_roi = relative_trapezoid_points_for_roi[1].x; // 在ROI坐标系下
        int core_x_start_disp = x_line_left_roi + (x_line_right_roi - x_line_left_roi) / 3;
        int core_x_end_disp = x_line_left_roi + 2 * (x_line_right_roi - x_line_left_roi) / 3;

        line(roi_cropped_content, Point(core_x_start_disp, 0), Point(core_x_start_disp, roi_cropped_content.rows - 1), Scalar(0, 255, 0), 1);
        line(roi_cropped_content, Point(core_x_end_disp, 0), Point(core_x_end_disp, roi_cropped_content.rows - 1), Scalar(0, 255, 0), 1);

        imshow("裁剪后的原始ROI区域", roi_cropped_content);

        Mat current_roi_hsv_input = frame(fixed_roi_rect).clone();
        cvtColor(current_roi_hsv_input, hsv_frame, COLOR_BGR2HSV);

        // 每次循环获取最新的trackbar值
        Scalar lower(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2));
        Scalar upper(hsv_upper.at<int>(0), hsv_upper.at<int>(1), hsv_upper.at<int>(2));

        white_mask = excludeWhiteLight(hsv_frame);
        inRange(hsv_frame, lower, upper, processed_roi_mask);
        processed_roi_mask.setTo(0, white_mask);

        Mat trapezoid_mask_cropped_size = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        fillPoly(trapezoid_mask_cropped_size, vector<vector<Point>>{relative_trapezoid_points_display}, Scalar(255));
        bitwise_and(processed_roi_mask, trapezoid_mask_cropped_size, processed_roi_mask);

        processed_roi_mask = applyMorphology(processed_roi_mask);

        imshow("1. Current Mask (ROI size)", processed_roi_mask);

        Mat new_pixel_mask = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        double new_area = computeNewArea(processed_roi_mask, previous_cumulative_mask, new_pixel_mask,
            relative_trapezoid_points_for_roi, fixed_roi_rect);
        area_history.push_back(new_area);

        imshow("2. New Mask (ROI size)", new_pixel_mask);
        imshow("3. Previous Mask (ROI size)", previous_cumulative_mask);

        overlayText(frame, "New Green Area: " + to_string(static_cast<int>(new_area)) + " px");

        Mat red_highlight_full = Mat::zeros(frame.size(), CV_8UC3); // Type needs to be CV_8UC3 for color
        Mat red_roi_part = red_highlight_full(fixed_roi_rect);
        red_roi_part.setTo(Scalar(0, 0, 255), new_pixel_mask); // new_pixel_mask 是ROI尺寸, 单通道

        Mat pseudo_result_full = frame.clone();
        // 叠加时确保伪色只在梯形内部，ROI外部是原始帧（伪色区域外保持原始图像区域）
        // 首先在整个图像上裁剪出伪色的ROI部分 (red_highlight_full)
        // 然后使用 addWeighted 叠加到完整的原始帧上

        // original_frame_outside_roi.copyTo(combined_frame_full_size, trapezoid_mask_full_inverted); // 也可以这样
        // addWeighted(pseudo_result_full, 1.0, red_highlight_full, 0.5, 0.0, pseudo_result_full); 

        // 更准确的做法是，先将原始frame的ROI区域克隆出来，然后只对这个区域叠加伪色
        // 再将伪色后的ROI区域放回原始frame的同一位置
        Mat frame_roi_clone = pseudo_result_full(fixed_roi_rect);
        addWeighted(frame_roi_clone, 0.5, red_roi_part, 0.5, 0.0, frame_roi_clone);

        imshow("伪色检测结果", pseudo_result_full);
        imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));

        imwrite("output/pseudo_frame_" + to_string(frame_idx) + ".png", pseudo_result_full);
        imwrite("output/plot_" + to_string(frame_idx) + ".png", generateAreaPlot(area_history, 400, 200));
        imwrite("masks/mask_" + to_string(frame_idx) + ".png", processed_roi_mask);
        imwrite("newmasks/newmask_" + to_string(frame_idx) + ".png", new_pixel_mask);

        csvFile << frame_idx << "," << new_area << "\n";

        roi_writer.write(roi_cropped_content);
        detect_writer.write(pseudo_result_full);

        // ========== 拼接视频帧和对比窗口 ==========
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
    comparison_writer.release(); // 释放新的视频写入器
    csvFile.close();
    destroyAllWindows();

    double total_area_sum = accumulate(area_history.begin(), area_history.end(), 0.0);
    cout << "==============================\n";
    cout << "Total Green Area (sum of new areas): " << total_area_sum << " px\n";
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
    Scalar upper_white(180, 20, 255);
    Mat white_mask;
    inRange(hsv, lower_white, upper_white, white_mask);
    return white_mask;
}

Mat applyMorphology(const Mat& mask) {
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    Mat result;
    morphologyEx(mask, result, MORPH_OPEN, kernel, Point(-1, -1), 1);
    morphologyEx(result, result, MORPH_CLOSE, kernel, Point(-1, -1), 1);
    return result;
}

double computeNewArea(const Mat& processed_roi_mask, Mat& previous_cumulative_mask, Mat& new_pixel_mask,
    const vector<Point>& trapezoid_pts_rel_roi, const Rect& roi_rect) {

    if (previous_cumulative_mask.empty() || previous_cumulative_mask.size() != processed_roi_mask.size() || previous_cumulative_mask.type() != processed_roi_mask.type()) {
        previous_cumulative_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
    }

    new_pixel_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
    double current_frame_new_area = 0.0;

    // 1. 定义核心区域 (X轴方向的三等分，取中间1/3)
    int x_line_left = trapezoid_pts_rel_roi[0].x;
    int x_line_right = trapezoid_pts_rel_roi[1].x;

    int core_x_start = x_line_left + (x_line_right - x_line_left) / 3;
    int core_x_end = x_line_left + 2 * (x_line_right - x_line_left) / 3;

    // 2. 查找当前帧中所有独立的绿色线粒体
    vector<vector<Point>> contours;
    findContours(processed_roi_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    map<int, MitochondrionState> current_frame_matched_mitochondria;

    for (const auto& contour : contours) {
        double area = contourArea(contour);

        if (area >= area_min && area <= area_max) {
            Moments m = moments(contour);
            Point2f centroid;
            if (m.m00 != 0) {
                centroid = Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
            }
            else {
                continue;
            }

            bool in_core_region = (centroid.x >= core_x_start && centroid.x <= core_x_end);

            int current_object_id = -1;
            double min_dist = max_centroid_distance;

            for (const auto& pair_item : tracked_mitochondria) {
                int obj_id = pair_item.first;
                const MitochondrionState& state = pair_item.second;

                double dist = norm(state.centroid - centroid);
                if (dist < min_dist) {
                    min_dist = dist;
                    current_object_id = obj_id;
                }
            }

            if (current_object_id == -1) {
                current_object_id = next_object_id++;
                tracked_mitochondria[current_object_id] = { centroid, false };
            }
            else {
                tracked_mitochondria[current_object_id].centroid = centroid;
            }

            if (in_core_region && !tracked_mitochondria[current_object_id].has_passed_core_and_counted) {
                Mat contour_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
                drawContours(contour_mask, vector<vector<Point>>{contour}, 0, Scalar(255), FILLED);

                bitwise_or(new_pixel_mask, contour_mask, new_pixel_mask);
                current_frame_new_area += area;

                tracked_mitochondria[current_object_id].has_passed_core_and_counted = true;

                bitwise_or(previous_cumulative_mask, contour_mask, previous_cumulative_mask);
            }
            current_frame_matched_mitochondria[current_object_id] = tracked_mitochondria[current_object_id];
        }
    }

    tracked_mitochondria = current_frame_matched_mitochondria;

    return current_frame_new_area;
}

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
