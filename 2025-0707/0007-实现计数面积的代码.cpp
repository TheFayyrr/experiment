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
// 修改函数签名，返回检测到的单个线粒体面积列表和轮廓
vector<double> computeNewArea(const Mat& processed_roi_mask, Mat& previous_cumulative_mask, Mat& new_pixel_mask, 
                              const vector<Point>& trapezoid_pts_rel_roi, const Rect& roi_rect, 
                              vector<vector<Point>>& detected_inner_contours);
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

    // Find min/max X to define left/right vertical lines
    int x_left = raw_points[0].x, x_right = raw_points[0].x;
    for (const auto& p : raw_points) {
        x_left = min(x_left, p.x);
        x_right = max(x_right, p.x);
    }

    // Sort to easily pick points for top/bottom Y coordinates
    // Assuming trapezoid_points[0] and [3] are on left side, [1] and [2] on right side after sorting
    sort(raw_points.begin(), raw_points.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
    });

    // The two points with smallest X (raw_points[0], raw_points[1]) form the left side
    // The two points with largest X (raw_points[2], raw_points[3]) form the right side
    
    int p1_y = min(raw_points[0].y, raw_points[1].y); // min Y of left side points
    int p4_y = max(raw_points[0].y, raw_points[1].y); // max Y of left side points
    
    int p2_y = min(raw_points[2].y, raw_points[3].y); // min Y of right side points
    int p3_y = max(raw_points[2].y, raw_points[3].y); // max Y of right side points
    
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

    // ========== 自适应颜色阈值设置 ==========
    cout << "正在进行自适应颜色阈值分析，请稍候...\n";
    vector<int> h_values;
    int num_frames_for_analysis = min(50, (int)cap.get(CAP_PROP_FRAME_COUNT)); // 分析前50帧或视频总帧数
    cap.set(CAP_PROP_POS_FRAMES, 0); // 确保从第一帧开始分析

    for (int i = 0; i < num_frames_for_analysis; ++i) {
        Mat frame_for_analysis;
        cap.read(frame_for_analysis);
        if (frame_for_analysis.empty()) break; 

        // 检查 fixed_roi_rect 是否与当前帧交叉，或者是否完全在图像之外
        if (fixed_roi_rect.x >= frame_for_analysis.cols || fixed_roi_rect.y >= frame_for_analysis.rows ||
            fixed_roi_rect.x + fixed_roi_rect.width <= 0 || fixed_roi_rect.y + fixed_roi_rect.height <= 0) {
            continue; // ROI 完全在图像之外，跳过此帧
        }
        
        Mat roi_for_analysis = frame_for_analysis(fixed_roi_rect);
        if (roi_for_analysis.empty()) { 
            cerr << "警告：分析帧 #" << i << " 裁剪出的ROI为空，跳过此帧。\n";
            continue;
        }

        Mat hsv_for_analysis;
        cvtColor(roi_for_analysis, hsv_for_analysis, COLOR_BGR2HSV);

        Mat trapezoid_mask_cropped_size = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        vector<Point> relative_trapezoid_points_for_display(4); 
        for(int j = 0; j < 4; ++j) {
            relative_trapezoid_points_for_display[j] = trapezoid_points[j] - fixed_roi_rect.tl();
        }
        fillPoly(trapezoid_mask_cropped_size, vector<vector<Point>>{relative_trapezoid_points_for_display}, Scalar(255));
        
        for (int r = 0; r < hsv_for_analysis.rows; ++r) {
            for (int c = 0; c < hsv_for_analysis.cols; ++c) {
                if (trapezoid_mask_cropped_size.at<uchar>(r, c) > 0) { 
                    if (hsv_for_analysis.at<Vec3b>(r, c)[1] > 20 && hsv_for_analysis.at<Vec3b>(r, c)[2] > 50) { 
                        h_values.push_back(hsv_for_analysis.at<Vec3b>(r, c)[0]);
                    }
                }
            }
        }
    }

    Mat hist;
    int histSize = 181; 
    float h_ranges[] = {0, 181}; 

    if (!h_values.empty()) {
        Mat h_values_float_mat(h_values.size(), 1, CV_32F); 
        for(size_t i = 0; i < h_values.size(); ++i) {
            h_values_float_mat.at<float>(i, 0) = static_cast<float>(h_values[i]);
        }
        
        if (h_values_float_mat.empty()) {
            cerr << "警告：转换后的 h_values_float_mat 为空，无法计算直方图。\n";
            goto end_adaptive_setup; 
        }

        int channels[] = {0};       
        int hist_sizes[] = {histSize}; 
        const float* ranges_ptr[] = {h_ranges}; 

        calcHist(&h_values_float_mat, 1, channels, Mat(), hist, 1, hist_sizes, ranges_ptr, true, false);

        int h_peak_min = -1, h_peak_max = -1;
        float max_hist_val = 0;

        // 调整了绿色H值的猜测范围
        int green_h_start_guess = 20; // 扩大范围
        int green_h_end_guess = 100;  // 扩大范围

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
        
        if(h_peak_min != -1) {
            int initial_peak_h = h_peak_min;
            // 调整了峰值扩展的阈值
            for(int h = initial_peak_h - 1; h >= green_h_start_guess; --h) {
                if(hist.at<float>(h) > max_hist_val * 0.07) { // 稍微放宽阈值
                    h_peak_min = h;
                } else {
                    break;
                }
            }
            for(int h = initial_peak_h + 1; h <= green_h_end_guess; ++h) {
                if(hist.at<float>(h) > max_hist_val * 0.07) { // 稍微放宽阈值
                    h_peak_max = h;
                } else {
                    break;
                }
            }

            hsv_lower.at<int>(0) = max(0, h_peak_min - 5); 
            hsv_upper.at<int>(0) = min(180, h_peak_max + 5);
            
            hsv_lower.at<int>(1) = max(0, hsv_lower.at<int>(1) > 0 ? hsv_lower.at<int>(1) : 43); 
            hsv_lower.at<int>(2) = max(0, hsv_lower.at<int>(2) > 0 ? hsv_lower.at<int>(2) : 46); 

            cout << "自适应HSV阈值已设置：\nHmin: " << hsv_lower.at<int>(0) << ", Hmax: " << hsv_upper.at<int>(0) << endl;
            cout << "Smin: " << hsv_lower.at<int>(1) << ", Vmin: " << hsv_lower.at<int>(2) << endl;
        } else {
            cout << "未能在分析帧中检测到明显的绿色峰值，使用默认HSV阈值。\n";
            hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
            hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
        }
    } else {
        cout << "无法进行Hsv分析，ROI区域内没有足够的有效像素，使用默认HSV阈值。\n";
        hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
        hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;
    }

    end_adaptive_setup:; 
    cap.set(CAP_PROP_POS_FRAMES, 0); 

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
    VideoWriter roi_writer("roi_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, roi_video_size);
    VideoWriter detect_writer("detect_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, firstFrame.size());
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
    for(int i = 0; i < 4; ++i) {
        relative_trapezoid_points_for_roi[i] = trapezoid_points[i] - fixed_roi_rect.tl();
    }

    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        Mat trapezoid_mask_full = createTrapezoidMask(frame.size(), trapezoid_points);
        
        Mat roi_cropped_content = frame(fixed_roi_rect).clone(); 

        vector<Point> relative_trapezoid_points_display(4); 
        for(int i = 0; i < 4; ++i) {
            relative_trapezoid_points_display[i] = trapezoid_points[i] - fixed_roi_rect.tl();
        }
        // ROI轮廓线粗细调整为1
        polylines(roi_cropped_content, vector<vector<Point>>{relative_trapezoid_points_display}, true, Scalar(0, 255, 255), 1); 
        
        // 移除绿色辅助线
        /*
        int x_line_left_roi = relative_trapezoid_points_for_roi[0].x; 
        int x_line_right_roi = relative_trapezoid_points_for_roi[1].x; 
        int core_x_start_disp = x_line_left_roi + (x_line_right_roi - x_line_left_roi) / 3;
        int core_x_end_disp = x_line_left_roi + 2 * (x_line_right_roi - x_line_left_roi) / 3;

        line(roi_cropped_content, Point(core_x_start_disp, 0), Point(core_x_start_disp, roi_cropped_content.rows -1), Scalar(0, 255, 0), 1); 
        line(roi_cropped_content, Point(core_x_end_disp, 0), Point(core_x_end_disp, roi_cropped_content.rows -1), Scalar(0, 255, 0), 1); 
        */
        
        imshow("裁剪后的原始ROI区域", roi_cropped_content);

        Mat current_roi_hsv_input = frame(fixed_roi_rect).clone(); 
        cvtColor(current_roi_hsv_input, hsv_frame, COLOR_BGR2HSV);
        
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

        Mat new_pixel_mask = Mat::zeros(fixed_roi_rect.size(), CV_8UC1); 
        vector<vector<Point>> current_frame_detected_contours; // 存储当前帧检测到的新轮廓

        vector<double> new_areas_list = computeNewArea(processed_roi_mask, previous_cumulative_mask, new_pixel_mask, 
                                                        relative_trapezoid_points_for_roi, fixed_roi_rect, 
                                                        current_frame_detected_contours); 
        
        double current_frame_total_new_area = 0.0;
        for (double area_val : new_areas_list) {
            current_frame_total_new_area += area_val;
        }

        area_history.push_back(current_frame_total_new_area); 

        imshow("2. New Mask (ROI size)", new_pixel_mask);       
        imshow("3. Previous Mask (ROI size)", previous_cumulative_mask); 
        
        overlayText(frame, "New Green Area: " + to_string(static_cast<int>(current_frame_total_new_area)) + " px"); 

        Mat red_highlight_full = Mat::zeros(frame.size(), CV_8UC3); 
        Mat red_roi_part = red_highlight_full(fixed_roi_rect); 
        red_roi_part.setTo(Scalar(0,0,255), new_pixel_mask); 

        Mat pseudo_result_full = frame.clone(); 
        Mat frame_roi_clone = pseudo_result_full(fixed_roi_rect);
        addWeighted(frame_roi_clone, 0.5, red_roi_part, 0.5, 0.0, frame_roi_clone);

        imshow("伪色检测结果", pseudo_result_full);
        imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));
        
        // ========== 新增：保存图片和输出信息 ==========
        if (!new_areas_list.empty()) { // 如果当前帧有新的线粒体被计数
            cout << "帧 " << frame_idx << " - 检测到 " << new_areas_list.size() << " 个新线粒体。\n";
            string areas_str = "面积: ";
            for (size_t i = 0; i < new_areas_list.size(); ++i) {
                areas_str += to_string(static_cast<int>(new_areas_list[i]));
                if (i < new_areas_list.size() - 1) areas_str += ", ";
            }
            cout << areas_str << endl;

            // 在原始帧上绘制黄色轮廓并保存
            Mat frame_with_yellow_contours = frame.clone();
            // 注意：current_frame_detected_contours 的点是相对于 ROI 区域的，需要加回 ROI 的偏移量
            vector<vector<Point>> global_contours;
            for (const auto& roi_contour : current_frame_detected_contours) {
                vector<Point> global_contour;
                for (const auto& pt : roi_contour) {
                    global_contour.push_back(pt + fixed_roi_rect.tl());
                }
                global_contours.push_back(global_contour);
            }
            drawContours(frame_with_yellow_contours, global_contours, -1, Scalar(0, 255, 255), 2); // 黄色轮廓，粗细为2
            
            // 添加文本说明到图片上方便查看
            string text_frame_idx = "Frame: " + to_string(frame_idx);
            string text_count = "Count: " + to_string(new_areas_list.size());
            string text_total_area = "Total Area: " + to_string(static_cast<int>(current_frame_total_new_area)) + " px";
            
            putText(frame_with_yellow_contours, text_frame_idx, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            putText(frame_with_yellow_contours, text_count, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            putText(frame_with_yellow_contours, text_total_area, Point(10, 90), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2);
            
            string all_areas_text = "Areas: ";
            for (double area_val : new_areas_list) {
                all_areas_text += to_string(static_cast<int>(area_val)) + " ";
            }
            // 文本可能过长，需要分行或限制长度
            if (all_areas_text.length() > 50) all_areas_text = all_areas_text.substr(0, 47) + "..."; // 截断以免出界
            putText(frame_with_yellow_contours, all_areas_text, Point(10, 120), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 1);

            string filename_base = "frame_" + to_string(frame_idx) + "_count_" + to_string(new_areas_list.size()) + "_area_" + to_string(static_cast<int>(current_frame_total_new_area));
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
        red_roi_highlight_cropped.setTo(Scalar(0,0,255), new_pixel_mask); 

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

    double total_area_sum = accumulate(area_history.begin(), area_history.end(), 0.0); // area_history 累积的是每帧新出现的总面积
    cout << "==============================\n";
    cout << "Total Green Area accumulated over count: " << total_area_sum << " px\n"; // 更名以明确其定义
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
    morphologyEx(mask, result, MORPH_OPEN, kernel, Point(-1,-1), 1);
    morphologyEx(result, result, MORPH_CLOSE, kernel, Point(-1,-1), 1);
    return result;
}

// 修改函数签名，返回检测到的单个线粒体面积列表和轮廓
vector<double> computeNewArea(const Mat& processed_roi_mask, Mat& previous_cumulative_mask, Mat& new_pixel_mask,
                   const vector<Point>& trapezoid_pts_rel_roi, const Rect& roi_rect, 
                   vector<vector<Point>>& detected_inner_contours) {

    if (previous_cumulative_mask.empty() || previous_cumulative_mask.size() != processed_roi_mask.size() || previous_cumulative_mask.type() != processed_roi_mask.type()) {
        previous_cumulative_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
    }
    
    new_pixel_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1); 
    vector<double> current_frame_new_areas; // 存储当前帧每个新线粒体的面积

    // 1. 定义核心区域 (X轴方向的三等分，取中间1/3)
    int x_line_left = trapezoid_pts_rel_roi[0].x;
    int x_line_right = trapezoid_pts_rel_roi[1].x;

    int core_x_start = x_line_left + (x_line_right - x_line_left) / 3;
    int core_x_end = x_line_left + 2 * (x_line_right - x_line_left) / 3;

    // 2. 查找当前帧中所有独立的绿色线粒体
    vector<vector<Point>> contours;
    findContours(processed_roi_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    map<int, MitochondrionState> current_frame_matched_mitochondria; // 用于下一帧匹配

    for (const auto& contour : contours) {
        double area = contourArea(contour);

        if (area >= area_min && area <= area_max) {
            Moments m = moments(contour);
            Point2f centroid;
            if (m.m00 != 0) { // 避免除以零
                centroid = Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
            } else {
                continue; // 无效轮廓，跳过
            }

            bool in_core_region = (centroid.x >= core_x_start && centroid.x <= core_x_end);

            int current_object_id = -1;
            double min_dist = max_centroid_distance;

            // 尝试匹配已有追踪对象
            for (const auto& pair_item : tracked_mitochondria) { 
                int obj_id = pair_item.first;
                const MitochondrionState& state = pair_item.second;

                double dist = norm(state.centroid - centroid);
                if (dist < min_dist) {
                    min_dist = dist;
                    current_object_id = obj_id;
                }
            }

            if (current_object_id == -1) { // 如果没有匹配到现有对象，则是一个新对象
                current_object_id = next_object_id++;
                tracked_mitochondria[current_object_id] = {centroid, false}; // 标记为未计数
            } else { // 如果匹配到现有对象，更新其质心
                tracked_mitochondria[current_object_id].centroid = centroid;
            }
            
            // 如果线粒体在核心区域并且之前未被计数
            if (in_core_region && !tracked_mitochondria[current_object_id].has_passed_core_and_counted) {
                Mat contour_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
                drawContours(contour_mask, vector<vector<Point>>{contour}, 0, Scalar(255), FILLED);
                
                bitwise_or(new_pixel_mask, contour_mask, new_pixel_mask); // 更新新像素蒙版
                current_frame_new_areas.push_back(area); // 记录单个线粒体的新面积
                
                tracked_mitochondria[current_object_id].has_passed_core_and_counted = true; // 标记为已计数

                bitwise_or(previous_cumulative_mask, contour_mask, previous_cumulative_mask); // 累积已计数像素
                detected_inner_contours.push_back(contour); // 添加轮廓到输出列表
            }
            current_frame_matched_mitochondria[current_object_id] = tracked_mitochondria[current_object_id]; 
        }
    }

    tracked_mitochondria = current_frame_matched_mitochondria; // 更新追踪对象列表

    return current_frame_new_areas; // 返回本帧新线粒体面积列表
}

Mat generateAreaPlot(const vector<double>& history, int width, int height) { 
    Mat plot = Mat::zeros(height, width, CV_8UC3);
    if (history.empty()) return plot;

    double max_val = *max_element(history.begin(), history.end()); 
    double min_val = *min_element(history.begin(), history.end()); 
    
    max_val = max(1.0, max_val); 
    if (abs(max_val - min_val) < 1e-6) min_val = 0.0; // 防止max_val和min_val相等导致除0

    double range = max_val - min_val;
    if (abs(range) < 1e-6) range = 1.0; // 避免除以零

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
        Point pt1(static_cast<int>((i - 1) * (width - 1.0) / (count -1.0)), 
                  height - 1 - static_cast<int>((history[i - 1] - min_val) * (height - 1.0) / range));
        Point pt2(static_cast<int>(i * (width - 1.0) / (count -1.0)), 
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
