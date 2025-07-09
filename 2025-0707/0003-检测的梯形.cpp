# '''这个代码还是有点问题，我是想要通过地铁口的而不是前几帧的图像对比'''
  #include <iostream>
#include <string>
#include <fstream>
#include <numeric>
#include <algorithm> // for std::sort, std::min, std::max
#include <vector>    // for std::vector

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// ========== 全局变量 ==========
vector<int> area_history;
vector<Point> trapezoid_points;
Mat hsv_lower = (Mat_<int>(1, 3) << 35, 43, 46);
Mat hsv_upper = (Mat_<int>(1, 3) << 77, 255, 255);
int area_min = 50;
int area_max = 20000;

// 定义裁剪区域的矩形，一旦确定，视频输出尺寸就固定了
Rect fixed_roi_rect;

// ========== 函数声明 ==========
void mouseCallback(int event, int x, int y, int, void* userdata);
Mat createTrapezoidMask(Size size, const vector<Point>& points_input);
Mat excludeWhiteLight(const Mat& hsv);
Mat applyMorphology(const Mat& mask);
int computeNewArea(const Mat& mask, Mat& previous_mask, Mat& new_mask);
Mat generateAreaPlot(const vector<int>& history, int width, int height);
void overlayText(Mat& frame, const string& text);

// ========== 主函数 ==========
int main() {
    // 确保 output, masks, newmasks 目录存在
    // 使用 std::filesystem (C++17) 或 _mkdir (Windows)/mkdir (Linux) 替换 system() 更健壮
    // 为了兼容性和简洁，这里仍然使用 system()，并尝试抑制输出错误
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

    ofstream csvFile("green_area_log1300.csv");
    csvFile << "Frame,NewGreenArea\n";

    Mat firstFrame;
    cap.read(firstFrame);
    if (firstFrame.empty()) {
        cerr << "无法读取第一帧" << endl;
        return -1;
    }

    Mat displayFrame = firstFrame.clone();
    namedWindow("点击选择4点构成ROI", WINDOW_NORMAL); // 窗口名改回通用ROI
    setMouseCallback("点击选择4点构成ROI", mouseCallback, &displayFrame);
    imshow("点击选择4点构成ROI", displayFrame);

    cout << "请在 '点击选择4点构成ROI' 窗口中点击4个点来定义ROI。\n";
    cout << "这些点将规范化为左右侧边竖直，上下侧边倾斜的梯形。\n";
    while (trapezoid_points.size() < 4) {
        waitKey(30);
    }

    // ========== ROI点规范化：根据图片示例生成直角梯形 ==========
    // 复制用户点击的点
    vector<Point> raw_points = trapezoid_points;

    // 1. 找到所有点击点中X的min/max值，作为最终梯形的左右垂直边界
    int x_left = raw_points[0].x, x_right = raw_points[0].x;
    for (const auto& p : raw_points) {
        x_left = min(x_left, p.x);
        x_right = max(x_right, p.x);
    }

    // 2. 将点击点根据X坐标分组 (左侧和右侧)
    vector<Point> left_points, right_points;
    double mid_x = (double)(x_left + x_right) / 2.0;

    for (const auto& p : raw_points) {
        if (p.x <= mid_x) { // 靠近左侧的点
            left_points.push_back(p);
        }
        else { // 靠近右侧的点
            right_points.push_back(p);
        }
    }

    // 确保左右各至少有两个点，防止用户点击过于偏斜
    // 这种处理不够健壮，如果用户只点击了三点在一侧，一点在另一侧，则会出错
    // 更好的做法是对 raw_points 整体按X排序，然后前两个是左侧，后两个是右侧
    sort(raw_points.begin(), raw_points.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
        });
    left_points = { raw_points[0], raw_points[1] };
    right_points = { raw_points[2], raw_points[3] };

    // 3. 确定左右垂直边的上下Y坐标
    int p1_y = min(left_points[0].y, left_points[1].y); // P1的Y (左上角Y)
    int p4_y = max(left_points[0].y, left_points[1].y); // P4的Y (左下角Y)

    int p2_y = min(right_points[0].y, right_points[1].y); // P2的Y (右上角Y)
    int p3_y = max(right_points[0].y, right_points[1].y); // P3的Y (右下角Y)

    // 4. 构造最终的梯形顶点 (P1, P2, P3, P4 顺时针顺序)
    trapezoid_points.clear();
    trapezoid_points.push_back(Point(x_left, p1_y)); // P1 (左上)
    trapezoid_points.push_back(Point(x_right, p2_y)); // P2 (右上)
    trapezoid_points.push_back(Point(x_right, p3_y)); // P3 (右下)
    trapezoid_points.push_back(Point(x_left, p4_y)); // P4 (左下)

    // 重新计算 fixed_roi_rect，它仍然是梯形的外接矩形
    fixed_roi_rect = boundingRect(trapezoid_points);
    fixed_roi_rect = fixed_roi_rect & Rect(0, 0, firstFrame.cols, firstFrame.rows);

    destroyWindow("点击选择4点构成ROI"); // 销毁ROI选择窗口

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

    Mat frame, hsv_frame, mask, white_mask, previous_mask;
    int frame_idx = 0;

    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        // 根据梯形点创建多边形掩膜（全尺寸）。此掩膜在处理和显示中都会用到。
        Mat trapezoid_mask_full = createTrapezoidMask(frame.size(), trapezoid_points);

        // 从原始帧中裁剪ROI外接矩形区域
        Mat roi_cropped_content = frame(fixed_roi_rect).clone();

        // 在裁剪后的ROI图像上绘制梯形轮廓，以便观察ROI的精确形状
        // 将全局梯形点转换为相对于裁剪区域的坐标
        vector<Point> relative_trapezoid_points(4);
        for (int i = 0; i < 4; ++i) {
            relative_trapezoid_points[i] = trapezoid_points[i] - fixed_roi_rect.tl();
        }
        polylines(roi_cropped_content, vector<vector<Point>>{relative_trapezoid_points}, true, Scalar(0, 255, 255), 2); // 黄色轮廓
        imshow("裁剪后的原始ROI区域", roi_cropped_content); // 用于观察ROI区域及轮廓

        // 对裁剪后的ROI区域进行处理 (HSV转换、白色排除、颜色阈值)
        Mat current_roi_hsv_input = frame(fixed_roi_rect).clone(); // 确保HSV转换基于正确内容
        cvtColor(current_roi_hsv_input, hsv_frame, COLOR_BGR2HSV);

        white_mask = excludeWhiteLight(hsv_frame); // ROI尺寸

        Scalar lower(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2));
        Scalar upper(hsv_upper.at<int>(0), hsv_upper.at<int>(1), hsv_upper.at<int>(2));
        inRange(hsv_frame, lower, upper, mask); // mask是ROI尺寸

        mask.setTo(0, white_mask); // 排除白色

        // 为确保Mask只在精确的梯形区域内部，应用裁剪尺寸的梯形掩膜
        Mat trapezoid_mask_cropped_size = Mat::zeros(fixed_roi_rect.size(), CV_8UC1);
        fillPoly(trapezoid_mask_cropped_size, vector<vector<Point>>{relative_trapezoid_points}, Scalar(255));
        bitwise_and(mask, trapezoid_mask_cropped_size, mask);

        mask = applyMorphology(mask); // Mask是ROI尺寸
    // 添加新的 imshow 窗口
    imshow("1. Current Mask (ROI size)", mask); // 观察形态学处理后的当前帧检测结果
        Mat new_mask;
        int new_area = computeNewArea(mask, previous_mask, new_mask);
        area_history.push_back(new_area);
 // 再次添加新的 imshow 窗口
    imshow("2. New Mask (ROI size)", new_mask);       // 观察本帧中真正“新”出现的区域
    imshow("3. Previous Mask (ROI size)", previous_mask); // 观察累积的“已计算”区域

        overlayText(frame, "New Green Area: " + to_string(new_area) + " px");

        // 将 new_mask 叠加回原始尺寸的 frame 上进行伪色显示
        Mat red_highlight_full = Mat::zeros(frame.size(), frame.type());
        Mat red_roi_part = red_highlight_full(fixed_roi_rect);
        red_roi_part.setTo(Scalar(0, 0, 255), new_mask); // new_mask 是裁剪尺寸的

        Mat pseudo_result_full = frame.clone();
        addWeighted(frame, 0.5, red_highlight_full, 0.5, 0.0, pseudo_result_full);

        // 确保非梯形区域显示原始帧内容
        // 创建一个全尺寸的非梯形区域掩膜
        Mat non_trapezoid_mask_full;
        bitwise_not(trapezoid_mask_full, non_trapezoid_mask_full);
        frame.copyTo(pseudo_result_full, non_trapezoid_mask_full);

        imshow("伪色检测结果", pseudo_result_full);
        imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));

        // 保存图像
        imwrite("output/1/pseudo_frame_" + to_string(frame_idx) + ".png", pseudo_result_full);
        imwrite("output/1/plot_" + to_string(frame_idx) + ".png", generateAreaPlot(area_history, 400, 200));
        imwrite("masks/1/mask_" + to_string(frame_idx) + ".png", mask);
        imwrite("newmasks/1/newmask_" + to_string(frame_idx) + ".png", new_mask);

        csvFile << frame_idx << "," << new_area << "\n";

        roi_writer.write(roi_cropped_content); // 写入裁剪后的ROI视频帧
        detect_writer.write(pseudo_result_full); // 写入全尺寸伪色叠加检测帧
        frame_idx++;

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    roi_writer.release();
    detect_writer.release();
    csvFile.close();
    destroyAllWindows();

    int total_area_sum = accumulate(area_history.begin(), area_history.end(), 0);
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

// 创建多边形掩膜
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

int computeNewArea(const Mat& mask, Mat& previous_mask, Mat& new_mask) {
    if (previous_mask.empty() || previous_mask.size() != mask.size() || previous_mask.type() != mask.type()) {
        previous_mask = Mat::zeros(mask.size(), CV_8UC1);
    }

    bitwise_and(mask, ~previous_mask, new_mask);
    int area = countNonZero(new_mask);

    if (area < area_min || area > area_max) {
        new_mask.setTo(0);
        bitwise_or(previous_mask, mask, previous_mask);
        return 0;
    }

    bitwise_or(previous_mask, mask, previous_mask);
    return area;
}

Mat generateAreaPlot(const vector<int>& history, int width, int height) {
    Mat plot = Mat::zeros(height, width, CV_8UC3);
    if (history.empty()) return plot;

    int max_val = *max_element(history.begin(), history.end());
    int min_val = *min_element(history.begin(), history.end());

    max_val = max(1, max_val);
    if (max_val == min_val) min_val = 0;

    double range = max_val - min_val;
    if (range == 0) range = 1;

    int count = history.size();
    if (count < 2) {
        Point pt(static_cast<int>((width - 1.0) / 2.0),
            height - 1 - static_cast<int>((history.back() - min_val) * (height - 1.0) / range));
        circle(plot, pt, 3, Scalar(0, 0, 255), FILLED);
        putText(plot, "Current: " + to_string(history.back()), Point(5, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
        return plot;
    }

    line(plot, Point(0, height - 1), Point(width - 1, height - 1), Scalar(255, 255, 255), 1);
    line(plot, Point(0, 0), Point(0, height - 1), Scalar(255, 255, 255), 1);

    for (int i = 1; i < count; ++i) {
        Point pt1(static_cast<int>((i - 1) * (width - 1.0) / (count - 1.0)),
            height - 1 - static_cast<int>((history[i - 1] - min_val) * (height - 1.0) / range));
        Point pt2(static_cast<int>(i * (width - 1.0) / (count - 1.0)),
            height - 1 - static_cast<int>((history[i] - min_val) * (height - 1.0) / range));

        line(plot, pt1, pt2, Scalar(0, 255, 0), 1);
    }

    Point last_pt(static_cast<int>((count - 1) * (width - 1.0) / (count - 1.0)),
        height - 1 - static_cast<int>((history.back() - min_val) * (height - 1.0) / range));
    circle(plot, last_pt, 3, Scalar(0, 0, 255), FILLED);

    putText(plot, "Max: " + to_string(max_val), Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
    putText(plot, "Current: " + to_string(history.back()), Point(5, 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);

    return plot;
}

void overlayText(Mat& frame, const string& text) {
    putText(frame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
}
