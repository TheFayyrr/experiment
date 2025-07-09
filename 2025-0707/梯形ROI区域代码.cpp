
#include <iostream>
#include <string>
#include <fstream>
#include <numeric>
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

// ========== 函数声明 ==========
void mouseCallback(int event, int x, int y, int, void* userdata);
Mat createTrapezoidMask(Size size, const vector<Point>& points);
Mat createHSVGreenMask(const Mat& hsv);
Mat excludeWhiteLight(const Mat& hsv);
Mat applyMorphology(const Mat& mask);
int computeNewArea(const Mat& mask, Mat& previous_mask, Mat& new_mask);
Mat generateAreaPlot(const vector<int>& history, int width, int height);
void overlayText(Mat& frame, const string& text);

// ========== 主函数 ==========
int main() {
    VideoCapture cap("D:/BaiduSyncdisk/200219.avi");
    if (!cap.isOpened()) {
        cerr << "无法打开视频文件" << endl;
        return -1;
    }

    ofstream csvFile("green_area_log.csv");
    csvFile << "Frame,NewGreenArea\n";

    Mat firstFrame;
    cap.read(firstFrame);
    if (firstFrame.empty()) return -1;

    namedWindow("点击选择4点构成梯形ROI", WINDOW_NORMAL);
    setMouseCallback("点击选择4点构成梯形ROI", mouseCallback, &firstFrame);
    imshow("点击选择4点构成梯形ROI", firstFrame);
    while (trapezoid_points.size() < 4) waitKey(30);
    destroyWindow("点击选择4点构成梯形ROI");
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

    Size video_size = firstFrame.size();
    //VideoWriter roi_writer("roi_output.mp4", VideoWriter::fourcc('a', 'v', 'c', '1'), 30, video_size);格式不匹配
    //VideoWriter detect_writer("detect_output.mp4", VideoWriter::fourcc('a', 'v', 'c', '1'), 30, video_size);

    VideoWriter roi_writer("roi_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, video_size);
    VideoWriter detect_writer("detect_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, video_size);


    Mat frame, hsv_frame, mask, white_mask, previous_mask;
    int frame_idx = 0;

    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        Mat trapezoid_mask = createTrapezoidMask(frame.size(), trapezoid_points);
        Mat roi = Mat::zeros(frame.size(), frame.type());
        frame.copyTo(roi, trapezoid_mask);
        Mat roi_masked;
        frame.copyTo(roi_masked, trapezoid_mask); // 仅 ROI 区域图
        imshow("原始ROI区域", roi_masked);


        cvtColor(roi, hsv_frame, COLOR_BGR2HSV);
        white_mask = excludeWhiteLight(hsv_frame);

        Scalar lower(hsv_lower.at<int>(0), hsv_lower.at<int>(1), hsv_lower.at<int>(2));
        Scalar upper(hsv_upper.at<int>(0), hsv_upper.at<int>(1), hsv_upper.at<int>(2));
        inRange(hsv_frame, lower, upper, mask);
        mask.setTo(0, white_mask);
        mask = applyMorphology(mask);

        Mat new_mask;
        int new_area = computeNewArea(mask, previous_mask, new_mask);
        area_history.push_back(new_area);

        overlayText(frame, "New Green Area: " + to_string(new_area) + " px");

        // 将 new_mask 显示为红色（线粒体区域）伪色
        Mat red_highlight(frame.size(), frame.type(), Scalar(0, 0, 255)); // 红色
        Mat pseudo_result = frame.clone();
        red_highlight.copyTo(pseudo_result, new_mask);

        Mat blended;
        addWeighted(frame, 2.0, pseudo_result, 0.1, 0.0, blended); // 仅ROI内变化

        imshow("伪色检测结果", blended);
        imshow("面积曲线图", generateAreaPlot(area_history, 400, 200));
        imwrite("output/pseudo_frame_" + to_string(frame_idx) + ".png", blended);
        imwrite("output/plot_" + to_string(frame_idx) + ".png", generateAreaPlot(area_history, 400, 200));


        imwrite("masks/mask_" + to_string(frame_idx) + ".png", mask);
        imwrite("newmasks/newmask_" + to_string(frame_idx) + ".png", new_mask);
        csvFile << frame_idx << "," << new_area << "\n";

        roi_writer.write(roi);           // 原始 ROI 视频帧
        detect_writer.write(blended);    // 伪色叠加检测帧
        frame_idx++;

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    destroyAllWindows();

    int total_area = accumulate(area_history.begin(), area_history.end(), 0);
    cout << "==============================\n";
    cout << "Total Green Area: " << total_area << " px\n";
    //574 
    cout << "==============================\n";

    return 0;
}

// ========== 函数定义 ==========
void mouseCallback(int event, int x, int y, int, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && trapezoid_points.size() < 4) {
        trapezoid_points.push_back(Point(x, y));
        circle(*(Mat*)userdata, Point(x, y), 5, Scalar(0, 0, 255), FILLED);
        imshow("点击选择4点构成梯形ROI", *(Mat*)userdata);
    }
}

Mat createTrapezoidMask(Size size, const vector<Point>& points) {
    Mat mask = Mat::zeros(size, CV_8UC1);
    vector<vector<Point>> contour{ points };
    fillPoly(mask, contour, Scalar(255));
    return mask;
}

Mat excludeWhiteLight(const Mat& hsv) {
    Scalar lower_white(0, 0, 200), upper_white(180, 30, 255);
    Mat white_mask;
    inRange(hsv, lower_white, upper_white, white_mask);
    return white_mask;
}

Mat createHSVGreenMask(const Mat& hsv) {
    Scalar lower(35, 43, 46);
    Scalar upper(77, 255, 255);
    Mat mask;
    inRange(hsv, lower, upper, mask);
    return mask;
}

Mat applyMorphology(const Mat& mask) {
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat result;
    morphologyEx(mask, result, MORPH_OPEN, kernel);
    morphologyEx(result, result, MORPH_CLOSE, kernel);
    return result;
}

int computeNewArea(const Mat& mask, Mat& previous_mask, Mat& new_mask) {
    if (previous_mask.empty() || previous_mask.size() != mask.size())
        previous_mask = Mat::zeros(mask.size(), CV_8UC1);

    bitwise_and(mask, ~previous_mask, new_mask);
    int area = countNonZero(new_mask);

    if (area < area_min || area > area_max) {
        new_mask.setTo(0);
        return 0;
    }

    bitwise_or(previous_mask, mask, previous_mask);
    return area;
}

Mat generateAreaPlot(const vector<int>& history, int width, int height) {
    Mat plot = Mat::zeros(height, width, CV_8UC3);
    int max_val = *max_element(history.begin(), history.end());
    max_val = max(1, max_val);

    int count = min((int)history.size(), width);
    for (int i = 1; i < count; ++i) {
        Point pt1(i - 1, height - history[i - 1] * height / max_val);
        Point pt2(i, height - history[i] * height / max_val);
        line(plot, pt1, pt2, Scalar(0, 255, 0), 1);
    }
    return plot;
}

void overlayText(Mat& frame, const string& text) {
    putText(frame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
}
