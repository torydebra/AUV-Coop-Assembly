#ifndef DETECTOR_H
#define DETECTOR_H

#include <iostream>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


class Detector
{

public:



  static int templateMatching(cv::Mat img, std::vector<cv::Mat> templVector,
                       std::vector<std::vector<cv::Point>> *found4CornersVector, std::vector<double> *bestValues,
                       int templ_method = cv::TM_SQDIFF,
                       std::vector<double> scaleFactors = std::vector<double>(),
                       bool showDisplay = true);

  static int templateMatching(cv::Mat img, cv::Mat templ,
                       std::vector<cv::Point> *found4Corners, double* bestValue = NULL,
                       int templ_method = cv::TM_SQDIFF,
                       std::vector<double> scaleFactors = std::vector<double>(),
                       bool showDisplay = true);

  static void drawSquares( cv::Mat image, const std::vector<std::vector<cv::Point> > squares,
                           const char* wndname = "Square Detection Demo");


  static int findSquare(cv::Mat &image, std::vector<std::vector<cv::Point>> *found4CornersVector,
                           int threshLevels = 11, int cannyThresh = 50);

private:
    // Disallow creating an instance of this object, it is useless since all functions are static
   Detector();

   static void MatchingMethod(int match_method, cv::Mat img, cv::Mat templ,
                              cv::Point *bestMatch, double *minMaxVal);


   static void _findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares,
                             int N, int thresh);
   static int orderAngles(std::vector<std::vector<cv::Point>> angles, std::vector<std::vector<cv::Point>> *orderedAngles);
   static int orderAngles(std::vector<cv::Point> angles, std::vector<cv::Point> *orderedAngles);
   static cv::Point getCenter(std::vector<cv::Point> points);
   static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 );


};

#endif // DETECTOR_H
