// Include files for required libraries
#include <stdio.h>
#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"

using namespace cv;
using namespace std;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

// PID Controller

//#define Kp 1
//#define Ki 0.01
//#define Kd 0.03

int Kp = 0.5, Ki = 0.01, Kd = 0.01;
void onTrackbar(int, void*)
{

}

static double integral2 = 0;
static double lastInput = 0;
//int lowH = 0, highH = 255, lowS = 0, highS = 255, lowV = 0, highV = 235;

int angle;
static int lossCount = 0;
int Output;

struct HSVValues{
int lowH;
int highH;
int lowS;
int highS;
int lowV;
int highV;
};

HSVValues HSV;

//HSV Values for pink
       int lpH = 138, lpS = 15, lpV = 63;
       int hpH = 179, hpS = 133, hpV = 139;

float PID(float setpoint, float input) {

//  Kp = Kp/10;
  Ki = Ki/100;
  Kd = Kd/100;

  float error = setpoint - input;
  float Pout = (Kp) * error;
  float Iout = Ki * integral2;
  float derivative = (input - lastInput) / 0.1;
  float Dout = Kd * derivative;
  float output = Pout + Iout + Dout;

  lastInput = input;

  return output;
}

int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

// send data
void sendData(int16_t leftMotorspeed, int16_t rightMotorspeed, int16_t servoAngle) {

  char dataToSend[6];
  Pi2c arduino(4); // Open a connection to the slave device at address 4

  // Write the data to the slave device
  dataToSend[0] = (leftMotorspeed >> 8) & 0xFF;    // first byte of leftMotor_speed, containing bits 16 to 9
  dataToSend[1] = leftMotorspeed & 0xFF;            // second byte of leftMotor_speed, containing bits 8 to 1
  dataToSend[2] = (rightMotorspeed >> 8) & 0xFF;    // rest follows the same logic
  dataToSend[3] = rightMotorspeed & 0xFF;
  dataToSend[4] = (servoAngle >> 8) & 0xFF;
  dataToSend[5] = servoAngle & 0xFF;

  arduino.i2cWrite(dataToSend,6);
}

double angle_between_points(const cv::Point& p1, const cv::Point& p2)
{
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double angle = std::atan2(dy, dx) * 180.0 / (3.14);
    return angle;
}

int symbolDetect()
{

///Load reference images
    Mat umbrella = imread("Umbrella.png");
    Mat star = imread("Star.png");
    Mat triangle = imread("Triangle.png");
    Mat circle = imread("Circle.png");

    ///Convert reference Images to HSV
   //Umbrella
    cvtColor(umbrella, umbrella,COLOR_BGR2HSV);
    inRange(umbrella,Scalar(1,141,162),Scalar(150,255,255),umbrella);
    umbrella = umbrella(Range(30,320),Range(30,320));
    //Star
    cvtColor(star, star,COLOR_BGR2HSV);
    inRange(star,Scalar(1,141,162),Scalar(150,255,255),star);
    star = star(Range(30,320),Range(30,320));
    //Triangle
    cvtColor(triangle, triangle,COLOR_BGR2HSV);
    inRange(triangle,Scalar(1,141,162),Scalar(150,255,255),triangle);
    triangle = triangle(Range(30,320),Range(30,320));
    //Circle
    cvtColor(circle, circle,COLOR_BGR2HSV);
    inRange(circle,Scalar(1,141,162),Scalar(150,255,255),circle);
    circle = circle(Range(30,320),Range(30,320));


 Mat HSVPink;
 Mat image;
 image = captureFrame();
 rotate(image,image,ROTATE_180);
 resize(image,image,Size(350,350));

 cvtColor(image,image,COLOR_BGR2HSV);
 inRange(image,Scalar(lpH,lpS,lpV),Scalar(hpH,hpS,hpV),HSVPink);



 Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(2,2));  //Process image
 morphologyEx(HSVPink,HSVPink, MORPH_OPEN,kernel);



 Mat Lines;

 Canny(HSVPink,Lines,100,100*3,3);




    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(Lines, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    if (contours.size() == 0) return(0);

    /// Draw contours
    Mat contourImage = Mat::zeros(HSVPink.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(255, 255, 255);
        drawContours(contourImage, contours, i, color, 2, LINE_8, hierarchy, 0);
    }



    ///Find The Largest Contour (Bounding Rectangle)


    int largest_contour_index = 0;
    double largest_contour_area = 0;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if (area > largest_contour_area)
        {
            largest_contour_area = area;
            largest_contour_index = i;
        }
    }

    ///Draw Corners
    vector<Point> corners;

        approxPolyDP(contours[largest_contour_index], corners, 0.01 * arcLength(contours[largest_contour_index], true), true);

        if (corners.size() == 4) {
            // Draw corners
            for (int j = 0; j < corners.size(); j++) {
                cv::circle(contourImage, corners[j], 5, Scalar(0, 255, 0), 2);

                cout <<"Coords: " << corners[j] << endl;
            }
        }

    imshow("Corners", contourImage);


    ///Transform the Image

    if (corners.size()==4)
    {

    Mat transformed = transformPerspective(corners,HSVPink,350,350);
    kernel = getStructuringElement(MORPH_ELLIPSE,Size(15,15));
    morphologyEx(transformed,transformed, MORPH_OPEN,kernel);
    transformed = transformed(Range(30,320),Range(30,320));
   // resize(transformed,contourImage,Size(350,350));
    imshow("Transformed",transformed);  //Transformed Image */

    ///Compare Image to Reference
    float UMatch = compareImages(transformed,umbrella);
    float SMatch = compareImages(transformed,star);
    float TMatch = compareImages(transformed,triangle);
    float CMatch = compareImages(transformed,circle);

        ///Find Largest % Match and Output
    char output[20];
    float Max = UMatch;
    strcpy(output, "Umbrella");
    if (SMatch > Max)
            {
            Max = SMatch;
            strcpy(output, "Star");
            }
    if (TMatch > Max)
            {
            Max = TMatch;
            strcpy(output, "Triangle");
            }
    if (CMatch > Max)
            {
            Max = CMatch;
            strcpy(output, "Circle");
            }

    if (Max == UMatch) return(1);
    else if (Max == SMatch) return(2);
    else if (Max == TMatch) return(3);
    else if (Max == CMatch) return(4);


    cout << Max << endl;
    cout << output << endl;

    }

    return(0);

}

cv::Point findCenterPoint(Mat frame, int lowH, int highH, int lowS, int highS, int lowV, int highV)
{
    cv::Mat frameHSV, mask;
    cv::cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV);
    cv::inRange(frameHSV, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(5,5)); //Prepare kernel for morphology operation
    morphologyEx(mask,mask,MORPH_OPEN,kernel); //perform morphology

   // Canny(mask,mask,100,100*3,3); //canny edge detection

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    if (!contours.empty())
    {
        int largest_contour_index = 0;
        double largest_area = 0.0;

        for (int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i], false);
            if (area > largest_area)
            {
                largest_area = area;
                largest_contour_index = i;
            }
        }

        //cout <<"largest area" << largest_area<<endl;


        cv::Moments M = cv::moments(contours[largest_contour_index]);

        if (M.m00 != 0 && largest_area > 500 && largest_area < 20000)
        {
            int cx = static_cast<int>(M.m10 / M.m00);
            int cy = static_cast<int>(M.m01 / M.m00);

            //drawContours(frame,contours,largest_contour_index,Scalar(255,255,255),2,LINE_8,hierarchy,0);
            //imshow("Contours",frame);

            return cv::Point(cx, cy);

        }


    }
    return Point(-1,-1);
    // Return an invalid point if no center point is found
    //return cv::Point(-1, -1);
}

void findHSV(int Colour)
{
  switch (Colour)
  {
   case 0:  //Black Line
   HSV.lowH = 0;
   HSV.highH = 179;
   HSV.lowS = 0;
   HSV.highS = 255;
   HSV.lowV = 0;
   HSV.highV = 81;
   break;

   case 1: ///Umbrella (Yellow)
   //Insert HSV Values
   cout << "umbrella values" <<endl;
   break;

   case 2: ///Star (Green)
   //Insert HSV Values
   cout << "star values" <<endl;
   break;

   case 3: ///Triangle (Blue)
   //Insert HSV Values
   cout << "triangle values" <<endl;
   break;

   case 4: ///Circle (Red)
   //Insert HSV Values
   cout << "circle values" <<endl;
   break;

  }

}


void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV


}

int main( int argc, char** argv )
{
    setup();

    // PID GAIN SLIDER
    namedWindow("Threshold Trackbars");
    createTrackbar("Kp", "Threshold Trackbars", &Kp, 10, onTrackbar);
    createTrackbar("Ki", "Threshold Trackbars", &Ki, 100, onTrackbar);
    createTrackbar("Kd", "Threshold Trackbars", &Kd, 100, onTrackbar);

     //Call a setup function to prepare IO and devices

//    namedWindow("HSV Tester");   // Create a GUI window called photo
//
//    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;    // Initialise some variables for HSV limits
//
//    createTrackbar("Low Hue", "HSV Tester", &lowH, 179, NULL);      // Create trackbar controls for each HSV limit
//    createTrackbar("High Hue", "HSV Tester", &highH, 179, NULL);
//
//    createTrackbar("Low Sat", "HSV Tester", &lowS, 255, NULL);
//    createTrackbar("High Sat", "HSV Tester", &highS, 255, NULL);
//
//    createTrackbar("Low Value", "HSV Tester", &lowV, 255, NULL);
//    createTrackbar("High Value", "HSV Tester", &highV, 255, NULL);

    int colourToFollow = 0;

    while(1)    // Main loop to perform image processing
    {

   /*
    ///Isolate Pink on input image
    Mat HSVPink;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(1,1));
    cvtColor(image, HSVPink,COLOR_BGR2HSV);//convert to hsv
    inRange(HSVPink,Scalar(lowH,lowS,lowV),Scalar(highH,highS,highV),HSVPink);
    morphologyEx(HSVPink,HSVPink, MORPH_OPEN,kernel);

    imshow("Pink Isolated",HSVPink);*/

//        int lowH = getTrackbarPos("Low Hue", "HSV Tester");        // Update the variables with the trackbar setting
//        int highH = getTrackbarPos("High Hue", "HSV Tester");
//        int lowS = getTrackbarPos("Low Sat", "HSV Tester");
//        int highS = getTrackbarPos("High Sat", "HSV Tester");
//        int lowV = getTrackbarPos("Low Value", "HSV Tester");
//        int highV = getTrackbarPos("High Value", "HSV Tester");






       findHSV(colourToFollow);  //Find HSV values for colour to follow

       int lowH = HSV.lowH;  //set the returned values to current colour
       int highH = HSV.highH;
       int lowS = HSV.lowS;
       int highS = HSV.highS;
       int lowV = HSV.lowV;
       int highV = HSV.highV;

       //cout << "HSV values from function: " <<HSV.highH<<endl;



       int leftSpeed = 100;
       int rightSpeed = 100;

        Mat frame;

        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
            rotate(frame,frame,ROTATE_180);
            imshow("original",frame);
            frame = frame(Rect(0,frame.rows/4,frame.cols,frame.rows/3*2));
           // resize(frame,frame,Size(350,350));

        cv::Point center = findCenterPoint(frame, lowH, highH, lowS, highS, lowV, highV); //finds center point of largest contour of this colour
        cv::Point p1(center.x, center.y); //sets coordinates of contour to p1
        cv::Point p2(320/2, 240); //sets centre of image to p2

        ///Check for Pink Pixels
        Mat HSVPink;
        cvtColor(frame, HSVPink,COLOR_BGR2HSV);//convert to hsv
        inRange(HSVPink,Scalar(lpH,lpS,lpV),Scalar(hpH,hpS,hpV),HSVPink);
        int PinkPresent = countNonZero(HSVPink);
        if (PinkPresent > 1000){
        cout <<"Pink detected:" <<PinkPresent<<endl;

        int colourToFollow = symbolDetect();
        }

        if  (center.x == -1 || center.y == -1) lossCount += 1; //checks that centre of contour has been found

        cout << "losscount: " <<lossCount<<endl;

        if (lossCount >=5 && lossCount < 10)
        {
            colourToFollow = 0; //After 5 losses of line reset to follow black line
        }

        if (lossCount >= 10)
        {
          lossCount = 0;
          leftSpeed = -100;
          rightSpeed = -100;
          int reverseAngle;
          reverseAngle = (90-(angle-90));
          do {
          sendData(leftSpeed,rightSpeed,reverseAngle);
          usleep(500000);
          break;
          } while(0);

        }

        if (center.y > 100) center.y = 100;

        Mat HSVBlack;
        cvtColor(frame, HSVBlack,COLOR_BGR2HSV);//convert to hsv
        inRange(HSVBlack,Scalar(lowH,lowS,lowV),Scalar(highH,highS,highV),HSVBlack); //shows hsv image for visualisation purposes

        cv::circle(HSVBlack, center, 10, cv::Scalar(0, 0, 255), -1);
        cv::circle(HSVBlack, p2, 10, cv::Scalar(0, 0, 255), -1);  //draws circles on center of image and contour
        angle = angle_between_points(p1, p2);   //finds angle between points

        std::cout << "Angle between points: " << angle << " degrees" << std::endl; //when straight ( 90 degrees)
        //float error = 90-angle;


        Output = angle;  //for differential
        angle = PID(90, angle);//setpoint by Dan is 110 also produces negative values
        angle = map(angle, -50, 50,160,20);  //maps angle value to range of input expected by servo motor
        integral2 += (angle-90);  //add to integral value
        //cout << "angle before" <<angle<< endl;

        //if (angle < 50) angle = 50;
        //if (angle > 150) angle = 150;


        cout << "angle" <<Output<< endl;
        cout << "PIDAngle "<<angle<< endl;

        // 8. Send/set steering angle and motor speed
        do{
        sendData(100, 100, angle);
        waitKey(1);
        break;
        }while(0); //sends data to motors and servo

        //waitKey(1);

        imshow("frame", HSVBlack);

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)
        //sleep(0.3);
        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;

            sleep(0.1);
    }






    waitKey(1);
    //sendData(0, 0, 90);


	closeCV();  // Disable the camera and close any windows
    destroyAllWindows();
	return 0;
}


