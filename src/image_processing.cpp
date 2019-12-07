#include "headers/image_processing.h"
extern cv::Mat canny;
extern cv::Mat raw_img;
extern cv::Mat fliped_img;
extern cv::Mat HLines_img;

extern int estacao2robot[TAM_ESTACOES];
extern robot_consts *indiv[TAM_POPULATION];

//callback from getImage topic
//receives the image from the robot onboard camera
//detects the black line in the image, if there is any
void getImage_callback(const sensor_msgs::Image::ConstPtr& msg){
    raw_img =  cv_bridge::toCvShare(msg, "bgr8")->image; //get the image
    int estacao = msg->header.frame_id[0] - '0';
    int robot = estacao2robot[estacao];
    if(robot == -1)
        return;


    char robotName[] = "robot X";
    robotName[6] = robot + '0';

    char windowName[] = "estacaoX";
    windowName[7] = estacao + '0';

    robot_consts consts = *(indiv[robot]);
    robot_vel robotVel = {0,0};                //later used to send the robot motor's speed
    delta     robot_error;

    flip(raw_img,fliped_img, 0);               //since the raw image is fliped, the unflip it
    Canny(fliped_img, canny, 50, 200, 3);      //apply canny filter to after aply the HoughLines algorithm
    cvtColor(canny,HLines_img,COLOR_GRAY2BGR); //HLines_img is the image where is going to b draw the lines detected in the canny image

    vector<Vec4i> linesP;                      //stores all the detected lines
    Vec4i choosenLine;                         //store the image choosen to be followed by the robot
    HoughLinesP(canny, linesP, 1, CV_PI/180, HoughLineTH, 30, 10 ); //apply the HoughLines algorithm to detect the lines inside the image

    if(linesP.size() > 0){                     //if there is at least one line detect, then...
        choosenLine = chooseLine(linesP);        //choose the better line to be followed (gives preference to the line followed in the previously frame)
        line( HLines_img, Point(choosenLine[0], choosenLine[1]), Point(choosenLine[2], choosenLine[3]), Scalar(255,0,0), 3, LINE_AA); //print the choosen line
        robot_error = getError(choosenLine);     //given the choosen line, get the robot error

        if(robot_error.angle != ERROR){
            robotVel = getMotorsVelocity(robot_error, consts);
            sendSpeed(robotVel, estacao);                     //send the motor speed to the robot
        }
    }
    else{

    }

    char sv0[15];
    char sang[15];
    char slin[15];
    sprintf (sv0, "v0 : %d", consts.v0);
    sprintf (sang, "ang: %.3f", consts.angular_kp);
    sprintf (slin, "lin: %.3f", consts.linear_kp);
    putText(HLines_img, sv0, Point(0,27), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, sang, Point(0,40), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, slin, Point(0,55), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, robotName, Point(0,15), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    imshow(windowName, HLines_img);       //shows the image with the choosen line printed on it
    waitKey(1);
}


//given a line, find the robot error
//the erro is given by the line's angle in rads and by where the line touchs the botton of the image
delta getError(Vec4i line){
  delta result = {ERROR,ERROR}; //init the error variable
  int X1 = line[1];             //sets the point's values
  int X2 = line[3];
  int Y1 = line[0];
  int Y2 = line[2];
  int dx = (Y2 - Y1);
  int dy = (X2 - X1);
  float a = 0;
  float b = 0;

  if(fabs(dx) < 0.00001)       //if dx is to low, the following calculation wont work, thus return error
    return result;

  a = (float)dy / (float)dx;   //gets the angular coeficient
  b = (Y2*X1 - Y1*X2)/dx;      //gets the linear coeficient

  result.angle = atan2(dy , -dx); //calculates the angle of the line
  if(result.angle < 0)            
    result.angle += M_PI;         //if the angle is negative, make it positive

  result.angle -= M_PI/2;
  result.shift = (128-b)/a - 128; //calculates where the line touch the botton of the image (y = a*x + b, where y = 128)

  //for debugging, print the error
  //cout << "a: " << a  << ",  b: " << b << ",  angle: " << result.angle << ", shift: " << result.shift << endl;
  return result;
}


//giving a vector of lines, choose the better one to be followed
//the best line is the where the first coordiante is the futher away from the image
Vec4i chooseLine(vector<Vec4i> linesP){
  Vec4i result;
  result[0] = linesP[0][0];
  result[1] = linesP[0][1];
  result[2] = linesP[0][2];
  result[3] = linesP[0][3];

  Vec4i l;
  for( size_t i = 0; i < linesP.size(); i++ )
  {
      l = linesP[i];
      if(l[0] < result[0]){
        result[0] = l[0];
        result[1] = l[1];
        result[2] = l[2];
        result[3] = l[3];
      }
  }

  return result;
}