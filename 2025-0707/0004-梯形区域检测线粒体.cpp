'''
  核心改变和解释：
computeNewArea 函数大改：
* 参数增加：现在它接收 trapezoid_pts_rel_roi (ROI内部的梯形点) 和 roi_rect (ROI的外接矩形)，以便计算核心区域的X范围。
* 核心区域定义：通过 core_x_start 和 core_x_end 定义了梯形ROI中间1/3的X轴范围。
* findContours ：不再直接对整个 processed_roi_mask 进行 countNonZero，而是先找到所有独立的“线粒体”轮廓。
* 个体追踪 (tracked_mitochondria map) ：
map<int, MitochondrionState> tracked_mitochondria; 用于存储每个被识别出的线粒体的状态（质心、是否已计数）。
max_centroid_distance ：定义了在连续帧中，两个轮廓何时被认为是同一个线粒体的阈值（质心距离）。您可能需要根据线粒体的移动速度和图像分辨率调整这个值。
当找到一个线粒体轮廓时，它会尝试匹配到 tracked_mitochondria 中最近的现有对象。
has_passed_core_and_counted 标志：每个线粒体对象都有一个 bool 标志。只有当一个线粒体首次进入并位于核心区域 (in_core_region 为 true) 且其 has_passed_core_and_counted 仍为 false 时，它才会被计入新的面积，并将其 has_passed_core_and_counted 设为 true。 这样就保证了每个线粒体只在它“通过正中央”时被计数一次。
bitwise_or(previous_cumulative_mask, contour_mask, previous_cumulative_mask);：这行将该线粒体的像素添加到 previous_cumulative_mask 中。这样做的目的是为了确保该线粒即便在核心区域持续存在，也不会再次被标记为“新”的伪色像素。
* new_pixel_mask 的生成：new_pixel_mask 只包含符合条件的新出现的线粒体像素。
main 函数中变量名的调整：
* mask 更名为 processed_roi_mask，表示这是经过HSV和形态学处理后，在ROI内的绿色区域掩膜。
* previous_mask 更名为 previous_cumulative_mask，表示它累积了所有计算过的像素，用于防止重复。
* new_mask 更名为 new_pixel_mask，表示它承载的是本帧被计入为“新”的像素点。
这个修改后的版本，应该能更好地实现您对“地铁站口”和“只计算正中央通过”的理解。

可能的注意事项和调试点：
max_centroid_distance 的调整：这是最关键的参数之一。如果设置太小，快速移动的线粒体可能会被认为是不同的对象，导致重复计数。如果设置太大，不同的线粒体可能会被认为是同一个对象，导致漏计数。您可能需要观察视频帧间的线粒体移动距离来确定。
“核心区域”的定义：目前是简单的 X 轴三等分。如果您对“正中央”有更精确的定义（例如，梯形中心线附近某个区域），可以进一步调整 core_x_start 和 core_x_end 的计算方式。
对象消失和重新进入：目前的简单追踪机制在对象暂时离开ROI后又重新进入时，可能会将其视为新对象而再次计数（如果它再次进入核心区域）。如果这是问题，需要更复杂的卡尔曼滤波等高级对象跟踪算法，但会显著增加复杂度。对于“流量”的累加，这种偶尔的重复可能是可接受的，取决于您的“流量”定义。
area_min/area_max ：这些参数仍然非常重要，用于过滤掉噪声和过大或粘连的伪线粒体。请在调试时仔细调整。
内存管理：tracked_mitochondria Map 会随着视频中出现的独立线粒体数量增加而增长。对于非常长的视频，可能需要定期清理那些很久没出现（“消失”）的对象。目前的清理逻辑比较简单，只是保留本帧识别到的对象。
请尝试运行这个新版本，并密切观察 Current Mask, New Mask, Previous Mask 各自窗口的表现，以及最终伪色视频中红色标记的出现时机，以验证是否符合您的预期。
'''



'''
  请务必先在 Visual Studio 项目属性中设置 C++ 语言标准为 /std:c++17 ，然后再用这份新代码替换您的 main.cpp 文件。这将解决所有的编译错误和警告。

新增调试辅助功能
在 main 函数的 while 循环里，我在 imshow("裁剪后的原始ROI区域", roi_cropped_content); 
后面增加了两行代码，用来在 ROI 窗口中绘制绿色的核心区域辅助线 (core_x_start_disp 和 core_x_end_disp)，这样您可以直观地看到“核心区域”在哪里，以便更好地理解和调整。
现在，您应该能够编译并运行代码了。运行时请仔细观察三个新的 imshow 窗口 (1. Current Mask, 2. New Mask, 3. Previous Mask) 以及 裁剪后的原始ROI区域 窗口中新增的绿色辅助线，它们将帮助您理解和调试线粒体识别和计数的过程。

'''



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
vector<double> area_history; // 修改为 double 类型
//vector<int> area_history; // Old: vector<int> area_history;
vector<Point> trapezoid_points; 
Mat hsv_lower = (Mat_<int>(1, 3) << 35, 43, 46);
Mat hsv_upper = (Mat_<int>(1, 3) << 77, 255, 255);
int area_min = 50;
int area_max = 20000;

// 定义裁剪区域的矩形，一旦确定，视频输出尺寸就固定了
Rect fixed_roi_rect; 

// Track individual mitochondria (simple tracking using centroid distance)
struct MitochondrionState {
    Point2f centroid;
    bool has_passed_core_and_counted; // true if this mitochondrion has been counted while in core region
    // double total_area; // Accumulated area for this specific mitochondrion, might not be needed for 'flow'
};
map<int, MitochondrionState> tracked_mitochondria;
int next_object_id = 0;
double max_centroid_distance = 20.0; // Max distance for centroids to be considered the same object between frames

// ========== 函数声明 ==========
void mouseCallback(int event, int x, int y, int, void* userdata);
Mat createTrapezoidMask(Size size, const vector<Point>& points_input); 
Mat excludeWhiteLight(const Mat& hsv);
Mat applyMorphology(const Mat& mask);
// 改变 computeNewArea 的签名和逻辑
double computeNewArea(const Mat& processed_roi_mask, Mat& previous_cumulative_mask, Mat& new_pixel_mask, 
                      const vector<Point>& trapezoid_pts_rel_roi, const Rect& roi_rect); // 返回值改为 double
Mat generateAreaPlot(const vector<double>& history, int width, int height); // 修改参数类型
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
    vector<Point> left_points = {raw_points[0], raw_points[1]};
    vector<Point> right_points = {raw_points[2], raw_points[3]};

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

    destroyWindow("点击选择4点构成ROI");

    //==== Trackbar ---默认值
    hsv_lower.at<int>(0) = 35; hsv_lower.at<int>(1) = 43; hsv_lower.at<int>(2) = 46;
    hsv_upper.at<int>(0) = 77; hsv_upper.at<int>(1) = 255; hsv_upper.at<int>(2) = 255;

    namedWindow("调节参数", WINDOW_NORMAL);
    createTrackbar("Hmin", "调节参数", &hsv_lower.at<int>(0), 180);
    createTrackbar("Smin", "调节参数", &hsv_lower.at<int>(1), 255);
    createTrackbar("Vmin", "调节参数", &hsv_lower.at<int>(2), 255);
    createTrackbar("Hmax", "调节参数", &hsv_upper.at<int>(0), 180);
    createTrackbar("Smax", "调节参数", &hsv_upper.at<int>(1), 255);
    createTrackbar("Vmax", "调节参数", &hsv_upper.at<int>(2), 255);
    createTrackbar("Amin", "调节参数", &area_min, 5000);
    createTrackbar("Amax", "调节参数", &area_max, 30000); 

    Size roi_video_size = fixed_roi_rect.size();
    VideoWriter roi_writer("roi_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, roi_video_size);
    VideoWriter detect_writer("detect_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, firstFrame.size());

    if (!roi_writer.isOpened() || !detect_writer.isOpened()) {
        cerr << "无法创建视频写入器" << endl;
        return -1;
    }

    Mat frame, hsv_frame, processed_roi_mask; 
    Mat white_mask;
    Mat previous_cumulative_mask; 
    int frame_idx = 0;

    // 将全局 trapezoid_points 转换成相对于 fixed_roi_rect 左上角的相对坐标
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
        polylines(roi_cropped_content, vector<vector<Point>>{relative_trapezoid_points_display}, true, Scalar(0, 255, 255), 2); 
        imshow("裁剪后的原始ROI区域", roi_cropped_content);

        // 可以在这里绘制 core_area，以便观察
        int x_line_left = relative_trapezoid_points_for_roi[0].x;
        int x_line_right = relative_trapezoid_points_for_roi[1].x;
        int core_x_start_disp = x_line_left + (x_line_right - x_line_left) / 3;
        int core_x_end_disp = x_line_left + 2 * (x_line_right - x_line_left) / 3;

        // 在 roi_cropped_content 上绘制核心区域的矩形辅助线
        // 注意：这是辅助绘制，不影响实际计算
        line(roi_cropped_content, Point(core_x_start_disp, 0), Point(core_x_start_disp, roi_cropped_content.rows -1), Scalar(0, 255, 0), 1); // 绿色垂直线
        line(roi_cropped_content, Point(core_x_end_disp, 0), Point(core_x_end_disp, roi_cropped_content.rows -1), Scalar(0, 255, 0), 1); // 绿色垂直线
        // 重新显示观察，因为我们加了辅助线
        imshow("裁剪后的原始ROI区域", roi_cropped_content);

        Mat current_roi_hsv_input = frame(fixed_roi_rect).clone(); 
        cvtColor(current_roi_hsv_input, hsv_frame, COLOR_BGR2HSV);
        
        white_mask = excludeWhiteLight(hsv_frame); 
        
        Scalar lower(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2));
        Scalar upper(hsv_upper.at<int>(0), hsv_upper.at<int>(1), hsv_upper.at<int>(2));
        inRange(hsv_frame, lower, upper, processed_roi_mask); 
        
        processed_roi_mask.setTo(0, white_mask);

        Mat trapezoid_mask_cropped_size = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        fillPoly(trapezoid_mask_cropped_size, vector<vector<Point>>{relative_trapezoid_points_display}, Scalar(255));
        bitwise_and(processed_roi_mask, trapezoid_mask_cropped_size, processed_roi_mask); 

        processed_roi_mask = applyMorphology(processed_roi_mask); 
        
        // 1. 观察原始颜色分割出来的区域（尚未排除旧面积）
        imshow("1. Current Mask (ROI size)", processed_roi_mask); 

        Mat new_pixel_mask = Mat::zeros(fixed_roi_rect.size(), CV_8UC1); 
        double new_area = computeNewArea(processed_roi_mask, previous_cumulative_mask, new_pixel_mask, 
                                     relative_trapezoid_points_for_roi, fixed_roi_rect); 
        area_history.push_back(new_area);

        // 2. 观察排除旧面积后，真正被认为是“新”的区域
        imshow("2. New Mask (ROI size)", new_pixel_mask);       
        // 3. 观察历史累积的蒙版，它会逐渐变白
        imshow("3. Previous Mask (ROI size)", previous_cumulative_mask); 
        
        overlayText(frame, "New Green Area: " + to_string(static_cast<int>(new_area)) + " px"); // 显示时转为int

        Mat red_highlight_full = Mat::zeros(frame.size(), frame.type()); 
        Mat red_roi_part = red_highlight_full(fixed_roi_rect); 
        red_roi_part.setTo(Scalar(0,0,255), new_pixel_mask); 

        Mat pseudo_result_full = frame.clone(); 
        addWeighted(frame, 0.5, red_highlight_full, 0.5, 0.0, pseudo_result_full); 

        Mat non_trapezoid_mask_full;
        bitwise_not(trapezoid_mask_full, non_trapezoid_mask_full);
        frame.copyTo(pseudo_result_full, non_trapezoid_mask_full); 
        
        imshow("伪色检测结果", pseudo_result_full);
        imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));
        
        imwrite("output/pseudo_frame_" + to_string(frame_idx) + ".png", pseudo_result_full);
        imwrite("output/plot_" + to_string(frame_idx) + ".png", generateAreaPlot(area_history, 400, 200));
        imwrite("masks/mask_" + to_string(frame_idx) + ".png", processed_roi_mask); 
        imwrite("newmasks/newmask_" + to_string(frame_idx) + ".png", new_pixel_mask); 
        
        csvFile << frame_idx << "," << new_area << "\n";

        roi_writer.write(roi_cropped_content); 
        detect_writer.write(pseudo_result_full); 
        frame_idx++;

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break; 
    }

    cap.release();
    roi_writer.release();
    detect_writer.release();
    csvFile.close();
    destroyAllWindows();

    double total_area_sum = accumulate(area_history.begin(), area_history.end(), 0.0); // 累加起点改为 0.0
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
    morphologyEx(mask, result, MORPH_OPEN, kernel, Point(-1,-1), 1);
    morphologyEx(result, result, MORPH_CLOSE, kernel, Point(-1,-1), 1);
    return result;
}

// 核心函数：计算新面积
double computeNewArea(const Mat& processed_roi_mask, Mat& previous_cumulative_mask, Mat& new_pixel_mask,
                   const vector<Point>& trapezoid_pts_rel_roi, const Rect& roi_rect) {

    if (previous_cumulative_mask.empty() || previous_cumulative_mask.size() != processed_roi_mask.size() || previous_cumulative_mask.type() != processed_roi_mask.type()) {
        previous_cumulative_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
    }
    
    new_pixel_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1); 
    double current_frame_new_area = 0.0; // 修改为 double

    // 1. 定义核心区域 (X轴方向的三等分，取中间1/3)
    int x_line_left = trapezoid_pts_rel_roi[0].x;
    int x_line_right = trapezoid_pts_rel_roi[1].x;

    int core_x_start = x_line_left + (x_line_right - x_line_left) / 3;
    int core_x_end = x_line_left + 2 * (x_line_right - x_line_left) / 3;

    // 2. 查找当前帧中所有独立的绿色线粒体
    vector<vector<Point>> contours;
    findContours(processed_roi_mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 用于本帧中成功匹配到的对象，方便下一帧清理
    map<int, MitochondrionState> current_frame_matched_mitochondria;

    for (const auto& contour : contours) {
        double area = contourArea(contour);

        if (area >= area_min && area <= area_max) {
            Moments m = moments(contour);
            Point2f centroid;
            // 检查 m.m00 是否为0，以避免除以零
            if (m.m00 != 0) {
                centroid = Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
            } else {
                // 如果面积为0，质心无法计算，跳过此轮廓
                continue; 
            }

            // 检查质心是否在核心区域内
            bool in_core_region = (centroid.x >= core_x_start && centroid.x <= core_x_end);

            // 简单追踪逻辑：寻找最近的前一帧对象
            int current_object_id = -1;
            double min_dist = max_centroid_distance;

            // 遍历tracked_mitochondria用pair
            for (const auto& pair_item : tracked_mitochondria) { // 解决C2429错误
                int obj_id = pair_item.first;
                const MitochondrionState& state = pair_item.second;

                double dist = norm(state.centroid - centroid);
                if (dist < min_dist) {
                    min_dist = dist;
                    current_object_id = obj_id;
                }
            }

            if (current_object_id == -1) { // 新对象
                current_object_id = next_object_id++;
                tracked_mitochondria[current_object_id] = {centroid, false}; // 不再维护 total_area
            } else { // 已存在的对象
                tracked_mitochondria[current_object_id].centroid = centroid;
                // tracked_mitochondria[current_object_id].total_area += area; // 暂停累积个体面积
            }
            
            // 重要逻辑：如果该线粒体质心在核心区域，且尚未被标记为“已计数”
            if (in_core_region && !tracked_mitochondria[current_object_id].has_passed_core_and_counted) {
                Mat contour_mask = Mat::zeros(processed_roi_mask.size(), CV_8UC1);
                drawContours(contour_mask, vector<vector<Point>>{contour}, 0, Scalar(255), FILLED);
                
                // 将这个线粒体的区域添加到 new_pixel_mask (用于显示)，并累积到计数
                bitwise_or(new_pixel_mask, contour_mask, new_pixel_mask); 
                current_frame_new_area += area; // 累加面积
                
                // 标记该线粒体为已计数
                tracked_mitochondria[current_object_id].has_passed_core_and_counted = true;

                // 将这些像素添加到 previous_cumulative_mask，防止后续帧再次计算同一像素
                bitwise_or(previous_cumulative_mask, contour_mask, previous_cumulative_mask);
            }
            current_frame_matched_mitochondria[current_object_id] = tracked_mitochondria[current_object_id]; // 存储本帧已匹配的对象
        }
    }

    // 清理不再存在的对象：只保留本帧中成功匹配到的对象
    tracked_mitochondria = current_frame_matched_mitochondria;

    // 最终的 new_pixel_mask 需要与 previous_cumulative_mask 进行一次 AND NOT 运算
    // 确保伪色只显示尚未被标记过的像素
    // 确保 new_pixel_mask 只有当前帧刚刚被计算的像素（且未被 previous_cumulative_mask 覆盖）
    Mat final_new_pixel_mask_for_display;
    bitwise_and(new_pixel_mask, ~previous_cumulative_mask, final_new_pixel_mask_for_display);
    new_pixel_mask = final_new_pixel_mask_for_display; 

    return current_frame_new_area;
}

Mat generateAreaPlot(const vector<double>& history, int width, int height) { // 修改参数类型
    Mat plot = Mat::zeros(height, width, CV_8UC3);
    if (history.empty()) return plot;

    double max_val = *max_element(history.begin(), history.end()); // 统计double
    double min_val = *min_element(history.begin(), history.end()); // 统计double
    
    max_val = max(1.0, max_val); // Use 1.0 for double
    if (abs(max_val - min_val) < 1e-6) min_val = 0.0; // Use epsilon for double comparison

    double range = max_val - min_val;
    if (abs(range) < 1e-6) range = 1.0;

    size_t count = history.size(); // 修改为 size_t
    if (count < 2) { 
        Point pt(static_cast<int>((width - 1.0) / 2.0), 
                 height - 1 - static_cast<int>((history.back() - min_val) * (height - 1.0) / range));
        circle(plot, pt, 3, Scalar(0, 0, 255), FILLED); 
        putText(plot, "Current: " + to_string(static_cast<int>(history.back())), Point(5, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
        return plot;
    }

    line(plot, Point(0, height - 1), Point(width - 1, height - 1), Scalar(255, 255, 255), 1); 
    line(plot, Point(0, 0), Point(0, height - 1), Scalar(255, 255, 255), 1); 

    for (size_t i = 1; i < count; ++i) { // 循环索引改为 size_t
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
