#include "headers/image_processing.h"
extern cv::Mat canny;
extern cv::Mat raw_img;
extern cv::Mat fliped_img;
extern cv::Mat HLines_img;

extern estacao estacao2robot[TAM_ESTACOES];
extern vector<robot_consts*> indiv;


extern double sumFitness;
extern double maxFitnessGen;
extern double maxFitnessTotal;
extern vector<double> maxFitnessVec;
extern vector<double> medFitnessVec;

//callback from getImage topic
//receives the image from the robot onboard camera
//detects the black line in the image, if there is any
void getImage_callback(const sensor_msgs::Image::ConstPtr& msg){
    raw_img =  cv_bridge::toCvShare(msg, "bgr8")->image; //get the image
    int estacao = msg->header.frame_id[0] - '0';
    int robot = estacao2robot[estacao].robot_station;
    robot_vel robotVel = {0,0};                //later used to send the robot motor's speed

    if(robot == -1){
        sendSpeed(robotVel, estacao);
        return;
    }

    robot_consts consts = *(indiv[robot]);
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
            // robot_vel vel = {70,70};
            //cout << "Robot[" << robot << "] VE:" << robotVel.Ve << " VD:" << robotVel.Vd << endl;
            sendSpeed(robotVel, estacao);                     //send the motor speed to the robot
        }
        indiv[robot]->framesPerdidos = 0;
    }
    else{
      (indiv[robot]->framesPerdidos)++;
    }
    (indiv[robot]->framesTotal)++;


    char windowName[15] = "estacaoX";
    windowName[7] = estacao + '0';

    char sv0[15];
    char sang[15];
    char slin[15];
    char ve[15];
    char vd[15];
    char robotName[15];
    char timeQuad[25];
    char LostFrames[25];
    char distPercorrida[25];
    sprintf (sv0, "v0 : %d", consts.v0);

    sprintf (sang, "ang: %.3f", robot_error.angle);
    // sprintf (sang, "ang: %.3f", consts.angular_kp);

    sprintf (slin, "lin: %.3f", consts.linear_kp);
    sprintf (ve, "ve: %d", robotVel.Ve > 127 ? robotVel.Ve - 256 : robotVel.Ve);
    sprintf (vd, "vd: %d", robotVel.Vd > 127 ? robotVel.Vd - 256 : robotVel.Vd);
    sprintf (robotName, "robot: %d", robot);
    sprintf (timeQuad, "QuadT: %ld", indiv[robot]->tempoNoQuadrante);
    sprintf (LostFrames, "Lost: %ld", indiv[robot]->framesPerdidos);
    sprintf (distPercorrida, "dist: %.3f", indiv[robot]->distanciaPercorrida);
    putText(HLines_img, sv0, Point(0,27), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, sang, Point(0,40), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, slin, Point(0,55), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, ve, Point(0,70), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, vd, Point(0,85), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, timeQuad, Point(0,97), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, LostFrames, Point(0,110), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
    putText(HLines_img, distPercorrida, Point(0,123), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
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


void ini_quadrantes(int estacao){
  
  estacao2robot[estacao].quadrante = (quadrante*) malloc(4 * sizeof(quadrante));
  quadrante quad_1;
  quadrante quad_2;
  quadrante quad_3;
  quadrante quad_4;

  switch (estacao){
    case 0:
      quad_1.posX = (-0.975);
      quad_1.posY = 1.675;
      
      quad_2.posX = (1.875);
      quad_2.posY = 1.125;
      
      quad_3.posX = (-1.175);
      quad_3.posY = (-1.9);

      quad_4.posX = (-1.95);
      quad_4.posY = 0.975;
      break;

    case 1:
      quad_1.posX = (5.025);
      quad_1.posY = 1.675;
      
      quad_2.posX = (7.875);
      quad_2.posY = 1.125;
      
      quad_3.posX = (4.825);
      quad_3.posY = (-1.9);
      
      quad_4.posX = (4.05);
      quad_4.posY = 0.975;
      break;

    case 2:
      quad_1.posX = (-0.975);
      quad_1.posY = 7.675;
      
      quad_2.posX = (1.875);
      quad_2.posY = 7.125;
      
      quad_3.posX = (-1.175);
      quad_3.posY = (4.1);
      
      quad_4.posX = (-1.95);
      quad_4.posY = 7.025;
      break;

    case 3:
      quad_1.posX = (5.025);
      quad_1.posY = 7.675;
      
      quad_2.posX = (7.875);
      quad_2.posY = 7.125;
      
      quad_3.posX = (4.825);
      quad_3.posY = (4.1);
      
      quad_4.posX = (4.05);
      quad_4.posY = 7.025;
      break;

    case 4:
      quad_1.posX = (11.025);
      quad_1.posY = 1.675;
      
      quad_2.posX = (13.875);
      quad_2.posY = 1.125;
      
      quad_3.posX = (10.825);
      quad_3.posY = (-1.9);
      
      quad_4.posX = (10.05);
      quad_4.posY = 1.025;
      break;

    case 5:
      quad_1.posX = (11.025);
      quad_1.posY = 7.675;
      
      quad_2.posX = (13.875);
      quad_2.posY = 7.125;
      
      quad_3.posX = (10.825);
      quad_3.posY = (4.1);
      
      quad_4.posX = (10.05);
      quad_4.posY = 7.025;
      break;
  }

  estacao2robot[estacao].quadrante[0] = quad_1;
  estacao2robot[estacao].quadrante[1] = quad_2;
  estacao2robot[estacao].quadrante[2] = quad_3;
  estacao2robot[estacao].quadrante[3] = quad_4;
}

void updateFitnessGraph(){
  int imgHeight = 480;
  int imgWidth  = 1280;
  int Qtd = maxFitnessVec.size();
  Mat graph(imgHeight, imgWidth, CV_8UC3, Scalar(0,0,0));
  
  int i = 0;
  char label[15];
  Point tempPt(0, 0);
  double FitnessTemp = maxFitnessTotal*10/8;
  double verticalStep = imgHeight/FitnessTemp;
  for(i = 0; i < 4; i++){
    tempPt.y = (imgHeight*(1 + i)/5);
    sprintf(label, "%.1lf", FitnessTemp*(4 - i)/5);
    putText(graph, label, tempPt, FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);
  }

  int HorizontalStep = imgWidth/(Qtd+1);
  Point firstPt(0,imgHeight);
  Point secondPt(0,0);

  Point medFirstPt(0,imgHeight);
  Point medSecondPt(0,0);
  
  int generation = 0;
  sprintf(label, "G%d", generation);
  putText(graph, label, Point(0, imgHeight - 10), FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);

  for(i = 0; i < Qtd; i++){
    secondPt.x += HorizontalStep;
    secondPt.y = imgHeight - maxFitnessVec[i] * verticalStep;

    if(maxFitnessVec[i] == PLOT_NEW_GENERATION){
      generation++;
      
      tempPt.x = secondPt.x - 10;
      if(generation % 2)
        tempPt.y = imgHeight - 15;
      else
        tempPt.y = imgHeight;

      secondPt.y = firstPt.y;
      sprintf(label, "G%d", generation);
      line(graph, Point(secondPt.x, 0), Point(secondPt.x, tempPt.y-15), Scalar::all(255), 1, 8, 0);
      putText(graph, label, tempPt, FONT_HERSHEY_PLAIN, 1, Scalar::all(255), 1, 8);

      medSecondPt.x = secondPt.x;
      medSecondPt.y = imgHeight - medFitnessVec[generation - 1] * verticalStep;
      line(graph, medFirstPt, medSecondPt, Scalar(0,255,0), 1, 8, 0);
      medFirstPt.x = medSecondPt.x;
      medFirstPt.y = medSecondPt.y;
    }

    line(graph, firstPt, secondPt, Scalar(255,0,0), 1, 8, 0);
    // ROS_INFO("Vec[i]: %0.2lf, MaxTotal: %0.2lf, FitnessTemp: %lf, verticalStep: %f, Points: %d %d (%lf) to %d %d (%lf)", maxFitnessVec[i], maxFitnessTotal, FitnessTemp, verticalStep, firstPt.x, firstPt.y, (imgHeight - firstPt.y)/verticalStep, secondPt.x, secondPt.y,  (imgHeight - secondPt.y)/verticalStep);
    firstPt.x = secondPt.x;
    firstPt.y = secondPt.y;
  }

  imshow("graph", graph);
  imwrite( "BestCross.jpg", graph);
}