#include "header/detector.h"

Detector::Detector(){} //private costructor not to be used


/** functions for METHOD FIND SQUARES *************************************************************************************/

/**
 * @brief Detector::findSquare find all square in images, exploiting functions of opencv.
 *   In practice is a blob detector but specific for the squares, e.g. it check if polygon founded have 4 side and
 *   four 90 degree angle (with some margins), and of a certain minimum area (to ot consider false positive).
 *   Code from opencv tutorial: https://docs.opencv.org/3.4/db/d00/samples_2cpp_2squares_8cpp-example.html#a20
 * @param image in which look for squares. Both gray and colored image are accepted
 * @param *found4CornersVector OUT each element contain a vector of 4 point (ie, defining a square).
 * @param threshLevels OPTIONAL number of threshold level. The more they are the more the result is good,
 *   but obviously also more computation time
 * @param threshold for opencv Canny function, it is the maximum threshold.
 * Canny is used only once, for the thresholdLevel equal to zero. Check _findSquares comments.
 * @return 0 correct execution
 * @note Please note that there is no way to understand which is the more precise square (if they are more than one).
 * Various trial show that is the squares are more than one, they are almost on the same position,
 * so they are not really false positive.
 * @note If a colored image is used, the result will be more precise because detection is done on each color
 * channel.
 * @todo various parameters are hardcoded in _findSquares. Maybe they can be setted from external caller.
 */
int Detector::findSquare(cv::Mat &image, std::vector<std::vector<cv::Point>> *found4CornersVector,
                         int threshLevels, int cannyThresh){

    std::vector<std::vector<cv::Point> > squares;

    //cv::cvtColor(image, image, cv::COLOR_GRAY2BGR); //actually not real conv in colors

    Detector::_findSquares(image, squares, threshLevels, cannyThresh);

    //Detector::drawSquares(image, squares, "asdasd");

    found4CornersVector->resize(squares.size());
    Detector::orderAngles(squares, found4CornersVector);

    return 0;

}

/**
 * @brief Detector::drawSquares function to draw square in images
 * @todo check if original image is modified
 * @param image
 * @param squares
 * @param wndname
 */
void Detector::drawSquares( cv::Mat image, const std::vector<std::vector<cv::Point> > squares,
                         const char* wndname)
{

  using namespace std;
  using namespace cv;
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
       // std::cout << "number of point:" << n << "\n";
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 2, LINE_AA);
        circle(image, squares[i][0], 5, Scalar(0,0,0), FILLED);
        circle(image, squares[i][1], 5, Scalar(255,255,0), FILLED);
        circle(image, squares[i][2], 5, Scalar(0,255,255), FILLED);
        circle(image, squares[i][3], 5, Scalar(255,255,255), FILLED);

    }
    imshow(wndname, image);
}


void Detector::_findSquares( const cv::Mat& image, std::vector<std::vector<cv::Point> >& squares,
                         int N, int thresh)
{

  using namespace std;
  using namespace cv;

  squares.clear();
  Mat pyr, timg, gray0(image.size(), CV_8U), gray;
  // down-scale and upscale the image to filter out the noise
  pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
  pyrUp(pyr, timg, image.size());
  vector<vector<Point> > contours;

  int channels = image.channels();
  // find squares in every color plane of the image
  for( int c = 0; c < channels; c++ )
  {
      int ch[] = {c, 0};
      mixChannels(&timg, 1, &gray0, 1, ch, 1);
      // try several threshold levels
      for( int l = 0; l < N; l++ )
      {
          // hack: use Canny instead of zero threshold level.
          // Canny helps to catch squares with gradient shading
          if( l == 0 )
          {
              // apply Canny. Take the upper threshold
              // and set the lower to 0 (which forces edges merging)
              Canny(gray0, gray, 0, thresh, 5);
              // dilate canny output to remove potential
              // holes between edge segments
              dilate(gray, gray, Mat(), Point(-1,-1));
          }
          else
          {
              // apply threshold if l!=0:
              //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
              gray = gray0 >= (l+1)*255/N;
          }
          // find contours and store them all as a list
          findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
          vector<Point> approx;
          // test each contour
          for( size_t i = 0; i < contours.size(); i++ )
          {
              // approximate contour with accuracy proportional
              // to the contour perimeter
              approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
              // square contours should have 4 vertices after approximation
              // relatively large area (to filter out noisy contours)
              // and be convex.
              // Note: absolute value of an area is used because
              // area may be positive or negative - in accordance with the
              // contour orientation
              if( approx.size() == 4 &&
                  fabs(contourArea(approx)) > 1000 &&
                  isContourConvex(approx) )
              {
                  double maxCosine = 0;
                  for( int j = 2; j < 5; j++ )
                  {
                      // find the maximum cosine of the angle between joint edges
                      double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                      maxCosine = MAX(maxCosine, cosine);
                  }
                  // if cosines of all angles are small
                  // (all angles are ~90 degree) then write quandrange
                  // vertices to resultant sequence
                  if( maxCosine < 0.3 )
                      squares.push_back(approx);
              }
          }
      }
  }
}




/**
 * @brief angle // helper function:
 * finds a cosine of angle between vectors
 * from pt0->pt1 and from pt0->pt2
 * @param pt1
 * @param pt2
 * @param pt0
 * @return
 */
double Detector::angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int Detector::orderAngles(std::vector<std::vector<cv::Point>> angles, std::vector<std::vector<cv::Point>> *orderedAngles){
  for (int i=0; i< angles.size(); i++){
    if (angles.at(i).size() != 4){
      std::cerr << "[DETECTOR] orderAngles Angles must be 4\n";
      return -1;
    }
    orderAngles(angles.at(i), &(orderedAngles->at(i)));
  }
  return 0;
}

int Detector::orderAngles(std::vector<cv::Point> angles, std::vector<cv::Point> *orderedAngles){

  if (angles.size() != 4){
    std::cerr << "[DETECTOR] orderAngles Angles must be 4\n";
    return -1;
  }


  cv::Point center = getCenter(angles);
  orderedAngles->resize(4);
  for (int i=0; i<4; i++){

    if (angles.at(i).x < center.x){
      if (angles.at(i).y < center.y){ //top left corner
        orderedAngles->at(0) = angles.at(i);
      } else { // bottom left corner
        orderedAngles->at(3) = angles.at(i);
      }

    } else {
      if (angles.at(i).y < center.y){ //top right corner
        orderedAngles->at(1) = angles.at(i);

      } else { // bottom rigth corner
        orderedAngles->at(2) = angles.at(i);
      }
    }
  }


}

cv::Point Detector::getCenter(std::vector<cv::Point> points){

    cv::Point A = points.at(0);
    cv::Point B = points.at(2);
    cv::Point C = points.at(3);
    cv::Point D = points.at(1);

    // Line AB represented as a1x + b1y = c1
    double a1 = A.x - A.y;
    double b1 = A.x - B.x;
    double c1 = a1*(A.x) + b1*(A.y);

    // Line CD represented as a2x + b2y = c2
    double a2 = D.y - C.y;
    double b2 = C.x - D.x;
    double c2 = a2*(C.x)+ b2*(C.y);

    double determinant = a1*b2 - a2*b1;

    if (determinant == 0)
    {
        /// TODO error, lines parallel
    }

    double x = (b2*c1 - b1*c2)/determinant;
    double y = (a1*c2 - a2*c1)/determinant;
    return cv::Point(x, y);

}


/** functions for METHOD TEMPLATE MATCHING ******************************************************************************/

/**
 * @brief Detector::templateMatching check function below, this one is used if multiple templates want be used
 * @param img image input where to find the match
 * @param templVector vector of templates
 * @param *found4CornersVector OUT each element will contain the 4 corners for each template
 * @param *bestValues OUT each element will contain the best value for each template (warning at MIN MAX based on method)
 * @param templ_method: OPTIONAL
 *        0 TM_SQDIFF : squared differences   (BEST is MIN)
 *        1 SQDIFF_NORMED: squared differences normed (BEST is MIN)
 *        2 CCORR : cross correlation? (BEST is MAX)
 *        3 CCORR_NORMED cross correlation normed?  (BEST is MAX)
 *        4 CCOEFF: ?  (BEST is MAX)
 *        5 CCOEFF_NORMED: ?  (BEST is MAX)
 * @param scaleFactors OPTIONAL the list of scaling factor default one is provided in the code
 * @param showDisplay OPTIONAL, choose if show template matching result in a window image
 * @return 0 correct execution
 */
int Detector::templateMatching(cv::Mat img, std::vector<cv::Mat> templVector,
                               std::vector<std::vector<cv::Point>> *found4CornersVector, std::vector<double> *bestValues,
                               int templ_method, std::vector<double> scaleFactors, bool showDisplay){

  for (int i=0; i< templVector.size(); i++){

    Detector::templateMatching(img, templVector.at(i), &(found4CornersVector->at(i)), &(bestValues->at(i)),
                                templ_method, scaleFactors, showDisplay);

  }



  return 0;
}



/**
 * @brief Detector::templateMatching template Matching method using opencv functions. Various method can be choosen
 * It detects the template and return a rectangle (represented as the 4 corners) around the best match. So if a precise
 * detection is needed, template and image should have same rotation, and be both square as possible. Size of template
 * is not important because various scaling of the original image are tried and the best match returned
 * (i.e. scale invariant template matching)
 *
 * @param img image input where to find the match
 * @param templ the template
 * @param *found4Corners OUT the founded4Corners passed by reference
 * @param *bestValue OUT OPTIONAL the value of the best match among all scalings.
 *     NOTE that depending on method can be a MAX or MIN value
 * @param templ_method: OPTIONAL
 *        0 TM_SQDIFF : squared differences   (BEST is MIN)
 *        1 SQDIFF_NORMED: squared differences normed (BEST is MIN)
 *        2 CCORR : cross correlation? (BEST is MAX)
 *        3 CCORR_NORMED cross correlation normed?  (BEST is MAX)
 *        4 CCOEFF: ?  (BEST is MAX)
 *        5 CCOEFF_NORMED: ?  (BEST is MAX)
 * @param scaleFactors OPTIONAL the list of scaling factor default one is provided in the code
 * @param showDisplay OPTIONAL, choose if show template matching result in a window image
 * @note the found4Corners are in CLOCKWISE order beginning from the TOP LEFT
 * @note template matching is good until no lighthing/transformations can happen, so not really suitable for continuosly
 * detect something in a video.
 * @note there is not a huge difference between the different summing methods,
 * formulas can be found in the opencv documentation :
 * @return 0 correct execution
 */
int Detector::templateMatching(cv::Mat img, cv::Mat templ,
                               std::vector<cv::Point> *found4Corners, double* bestValue,
                               int templ_method,
                               std::vector<double> scaleFactors, bool showDisplay){


  if (img.channels() == 1){ //GRAY image
    cv::cvtColor(img, img, cv::COLOR_GRAY2BGR); //actually not real conv in colors, but algos need 3 channels
  }

  const char* image_window;
  cv::Mat img_display;
  if(showDisplay){
    image_window = "Template Matching result";
    img_display = img.clone();
    cv::namedWindow( image_window, cv::WINDOW_AUTOSIZE );
  }

  if (scaleFactors.size() == 0){
    scaleFactors = {1, 0.9, 0.8, 0.7, 0.75, 0.6, 0.65, 0.5,
                    0.48, 0.45, 0.42, 0.4, 0.38, 0.38, 0.32, 0.3,
                    0.28, 0.25, 0.22, 0.2, 0.15, 0.1};
  }


  std::vector<double> minMaxValue(scaleFactors.size()),
                      scaleUpY(scaleFactors.size()),
                      scaleUpX(scaleFactors.size());

  std::vector<cv::Point> bestMatch(scaleFactors.size());
  cv::Mat imgScaled = img.clone();

  //last scaling is the last scaling, then if the for is "break" last scale is modified
  //it is used as index to consider only max/min values until the last scaling factor used
  int lastScale=scaleFactors.size();
  for (int i=0; i<scaleFactors.size(); i++){

    cv::resize(img, imgScaled, cv::Size(), scaleFactors[i], scaleFactors[i]);
    //std::cout << "[DETECTOR] img size y, x: " << img.rows << "  " << img.cols << "\n";
    //std::cout << "scaled img y, x: " << imgScaled.rows << "  " <<imgScaled.cols << "\n";
    // actual scale can be approximated so calculate scaleUp in this way
    // is better than 1/scaleFactors[i]
    scaleUpY[i] = ((double)img.rows) / ((double)imgScaled.rows);
    scaleUpX[i] = ((double)img.cols) / ((double)imgScaled.cols);

    //if scaled img little than template, break loop
    if (imgScaled.rows < templ.rows || imgScaled.cols < templ.cols){
      //std::cout << "[DETECTOR] template bigger\n";
      lastScale = i-1;
      break;
    }


    Detector::MatchingMethod(templ_method, imgScaled, templ, &(bestMatch[i]), &(minMaxValue[i]));


//      std::cout << "scale factor UPX and UPY  " << scaleUpX << " " << scaleUpY  << "\n";
//      std::cout << "bestMatrch.x: " << bestMatch[i].x << "\n";
//      std::cout << "bestMatrch.y: " << bestMatch[i].y << "\n";
//      std::cout << "topLeft.x: " << topLeft.x << "\n";
//      std::cout << "topLeft.y: " << topLeft.y << "\n";
//      std::cout << "bottomRi.x: " << bottomRight.x << "\n";
//      std::cout << "bottomRi.y: " << bottomRight.y << "\n";
//      std::cout << "temple row col " << templ.rows << "  " << templ.cols << "\n\n";


//      cv::rectangle( imgScaled, bestMatch[i],
//                    cv::Point( bestMatch[i].x + templ.cols , bestMatch[i].y + templ.rows ),
//                     cv::Scalar::all(0), 1, 8, 0 );
//      std::string boh = "asdasda" + std::to_string(i);
//      cv::imshow( boh, imgScaled );
//      cv::Mat otherMat;
//      cv::resize (imgScaled, otherMat, cv::Size(), scaleUpX[i], scaleUpY[i]);
//      cv::imshow( "boh", otherMat );
//      cv::waitKey();



  }

  int indexBest;
  if( templ_method  == cv::TM_SQDIFF || templ_method == cv::TM_SQDIFF_NORMED ){
    indexBest = std::distance(minMaxValue.begin(),
                                std::min_element(minMaxValue.begin(),
                                                 minMaxValue.begin() + lastScale));

  } else{
    indexBest = std::distance(minMaxValue.begin(),
                               std::max_element(minMaxValue.begin(),
                                                minMaxValue.begin() + lastScale));
  }

  std::cout << "[DETECTOR][TEMPLATE_MATCHING] BEST ITERATION: scaling factor " << scaleFactors[indexBest]
               <<"\n \t with value: " << minMaxValue.at(indexBest) << "\n";

  cv::Point topLeft, bottomRight;
  topLeft.x = (int)(bestMatch[indexBest].x * scaleUpX[indexBest]);
  topLeft.y = (int)(bestMatch[indexBest].y * scaleUpY[indexBest]);
  bottomRight.x = (int)( (bestMatch[indexBest].x + templ.cols) * scaleUpX[indexBest]);
  bottomRight.y = (int)( (bestMatch[indexBest].y +  templ.rows) * scaleUpY[indexBest]);

  if(showDisplay){

    cv::rectangle( img_display, topLeft, bottomRight,
                   cv::Scalar::all(0), 1, 8, 0 );
    cv::imshow( image_window, img_display);
    cv::waitKey(0);
  }

  found4Corners->resize(4);
  found4Corners->at(0) = topLeft;
  found4Corners->at(1) = cv::Point(bottomRight.x, topLeft.y);
  found4Corners->at(2) = bottomRight;
  found4Corners->at(3) = cv::Point(topLeft.x, bottomRight.y);

  if (bestValue != NULL){
    *bestValue = minMaxValue.at(indexBest);
  }

  return 0;

}


void Detector::MatchingMethod(int match_method, cv::Mat img, cv::Mat templ,
                    cv::Point *bestMatch, double *minMaxVal){
  using namespace std;
  using namespace cv;

  Mat result;

  int result_cols =  img.cols - templ.cols + 1;
  int result_rows = img.rows - templ.rows + 1;

  result.create( result_rows, result_cols, CV_32FC1 );

  matchTemplate( img, templ, result, match_method);

  double minVal; double maxVal; Point minLoc; Point maxLoc;
  minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc);
  if( match_method  == TM_SQDIFF || match_method == TM_SQDIFF_NORMED ) {
    *bestMatch = minLoc;
    *minMaxVal = minVal;
  } else {
    *bestMatch = maxLoc;
    *minMaxVal = maxVal;
  }

  return;
}
