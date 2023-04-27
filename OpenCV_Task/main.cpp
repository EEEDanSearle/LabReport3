#include <opencv2/opencv.hpp>
#include <iostream>
#include "opencv_aee.hpp"
#include "opencv_aee.cpp"
#include <stdlib.h>
#include <stdio.h>
#include <imgproc.hpp>

using namespace std;
using namespace cv;


int main()
{
    ///Load Images
    Mat image;
//
//            image = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
//            rotate(image,image,ROTATE_180);
            image = imread("pistar.png");
            resize(image,image,Size(350,350));

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


    ///Isolate Pink on input image

    Mat HSVPink;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(2,2));
    cvtColor(image, HSVPink,COLOR_BGR2HSV);//convert to hsv
    inRange(HSVPink,Scalar(138,15,63),Scalar(179,133,139),HSVPink);
    morphologyEx(HSVPink,HSVPink, MORPH_OPEN,kernel);

    imshow("Pink Isolated",HSVPink);
    imshow("Original",image);

    Mat Lines;

    Canny(HSVPink,Lines,100,100*3,3);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(Lines, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    /// Draw contours
    Mat contourImage = Mat::zeros(image.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(255, 255, 255);
        drawContours(contourImage, contours, i, color, 2, LINE_8, hierarchy, 0);
    }

    //imshow("contourImage",contourImage);

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

        approxPolyDP(contours[largest_contour_index], corners,
                     0.01 * arcLength(contours[largest_contour_index], true), true);

        if (corners.size() == 4) {
            // Draw corners
            for (int j = 0; j < corners.size(); j++) {
                cv::circle(contourImage, corners[j], 5, Scalar(0, 255, 0), 2);

                cout <<"Coords: " << corners[j] << endl;
            }
        }

    imshow("Corners", contourImage);


    ///Transform the Image

    Mat transformed = transformPerspective(corners,HSVPink,350,350);
    kernel = getStructuringElement(MORPH_ELLIPSE,Size(15,15));
    morphologyEx(transformed,transformed, MORPH_OPEN,kernel);
    transformed = transformed(Range(30,320),Range(30,320));
   // resize(transformed,contourImage,Size(350,350));
    imshow("Transformed",transformed);  //Transformed Image


   ///Compare Image to Reference
    float UMatch = compareImages(transformed,umbrella);
    float SMatch = compareImages(transformed,star);
    float TMatch = compareImages(transformed,triangle);
    float CMatch = compareImages(transformed,circle);


    ///Output percentage match
    cout << "UMatch% =  " << UMatch << endl;
    cout << "SMatch% =  " << SMatch << endl;
    cout << "TMatch% =  " << TMatch << endl;
    cout << "CMatch% =  " << CMatch << endl;

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


    cout << Max << endl;
    cout << output << endl;




    waitKey(0);

    return 0;
}

