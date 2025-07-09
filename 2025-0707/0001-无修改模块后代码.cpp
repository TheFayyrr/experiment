#include <iostream>
#include <string>
#include <fstream>
#include <numeric>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 全局变量
vector<int> area_history;
Rect roi_rect;

// ========== 函数声明 ==========
Rect selectROIRegion(Mat& frame);
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
        cerr << "\u9519\u8bef: \u65e0\u6cd5\u6253\u5f00\u89c6\u9891\u6587\u4ef6" << endl;
        return -1;
    }

    ofstream csvFile("green_area_log.csv");
    csvFile << "Frame,NewGreenArea\n";

    Mat firstFrame;
    cap.read(firstFrame);
    if (firstFrame.empty()) return -1;

    roi_rect = selectROIRegion(firstFrame);
    if (roi_rect.area() <= 0) return -1;

    Mat frame, roi, hsv_roi, mask, white_mask, previous_mask;
    while (true) {
        cap.read(frame);
        if (frame.empty()) break;

        roi = frame(roi_rect);
        cvtColor(roi, hsv_roi, COLOR_BGR2HSV);

        white_mask = excludeWhiteLight(hsv_roi);
        mask = createHSVGreenMask(hsv_roi);
        mask.setTo(0, white_mask);
        mask = applyMorphology(mask);

        Mat new_mask;
        int new_area = computeNewArea(mask, previous_mask, new_mask);
        area_history.push_back(new_area);

        rectangle(frame, roi_rect, Scalar(0, 255, 0), 2);
        overlayText(frame, "New Green Area: " + to_string(new_area) + " px");

        Mat green_area;
        roi.copyTo(green_area, new_mask);

        imshow("ROI + Result", frame);
        imshow("Mask", mask);
        imshow("Detected Green", green_area);

        Mat plot_img = generateAreaPlot(area_history, 400, 200);
        imshow("Area Plot", plot_img);

        static int frame_idx = 0;
        imwrite("plots/plot.png", plot_img);
        imwrite("masks/mask_" + to_string(frame_idx) + ".png", mask);
        imwrite("newmasks/newmask_" + to_string(frame_idx) + ".png", new_mask);
        csvFile << frame_idx++ << "," << new_area << "\n";

        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    destroyAllWindows();

    int total_area = accumulate(area_history.begin(), area_history.end(), 0);
    cout << "==============================\n";
    cout << "Total Green Area: " << total_area << " px\n";
    cout << "==============================\n";

    return 0;
}

// ========== 函数定义 ==========
Rect selectROIRegion(Mat& frame) {
    Rect roi = selectROI("Select ROI", frame);
    destroyWindow("Select ROI");
    return roi;
}

Mat createHSVGreenMask(const Mat& hsv) {
    Scalar lower(35, 43, 46);
    Scalar upper(77, 255, 255);
    Mat mask;
    inRange(hsv, lower, upper, mask);
    return mask;
}

Mat excludeWhiteLight(const Mat& hsv) {
    Scalar lower_white(0, 0, 200), upper_white(180, 30, 255);
    Mat white_mask;
    inRange(hsv, lower_white, upper_white, white_mask);
    return white_mask;
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

    if (area < 50 || area > 20000) {
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
        Point pt2(i,     height - history[i]     * height / max_val);
        line(plot, pt1, pt2, Scalar(0, 255, 0), 1);
    }
    return plot;
}

void overlayText(Mat& frame, const string& text) {
    putText(frame, text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 2);
}
