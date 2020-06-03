#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

using namespace std;

void descKeypoints1()
{
    // load image from file and convert to grayscale
    cv::Mat imgGray;
    cv::Mat img = cv::imread("../images/img1.png");
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // BRISK detector / descriptor
    cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
    vector<cv::KeyPoint> kptsBRISK;

    double t = (double)cv::getTickCount();
    detector->detect(imgGray, kptsBRISK);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BRISK detector with n= " << kptsBRISK.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::BRISK::create();
    cv::Mat descBRISK;
    t = (double)cv::getTickCount();
    descriptor->compute(imgGray, kptsBRISK, descBRISK);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "BRISK descriptor in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, kptsBRISK, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "BRISK Results";
    cv::namedWindow(windowName, 1);
    imshow(windowName, visImage);
    cv::waitKey(0);

    // TODO: Add the SIFT detector / descriptor, compute the 
    // time for both steps and compare both BRISK and SIFT
    // with regard to processing speed and the number and 
    // visual appearance of keypoints.

    // SURF detector / descriptor
    detector = cv::xfeatures2d::SURF::create();
    vector<cv::KeyPoint> kptsSURF;

    double t1 = (double)cv::getTickCount();
    detector->detect(imgGray, kptsSURF);
    t1 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
    cout << "SURF detector with n= " << kptsSURF.size() << " keypoints in " << 1000 * t1 / 1.0 << " ms" << endl;

    descriptor = cv::xfeatures2d::SURF::create();
    cv::Mat descSURF;
    t = (double)cv::getTickCount();
    descriptor->compute(imgGray, kptsSURF, descSURF);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "SURF descriptor in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    cv::Mat visImage1 = img.clone();
    cv::drawKeypoints(img, kptsSURF, visImage1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName1 = "SURF Results";
    cv::namedWindow(windowName1, 1);
    imshow(windowName1, visImage1);
    cv::waitKey(0);
}

int main()
{
    descKeypoints1();
    return 0;
}