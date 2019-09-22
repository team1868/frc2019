// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// // #include <iostream>
// // #include <fstream> 
// // #include <string>
// // #include <direct.h>
// //#include <../experimental/filesystem>
// #include <pathfinder.h>

#include "Targets.h"
#include "../../../RobotModel.h"
#include "../../AutoCommand.h"
#include <frc/WPILib.h>
#include <math.h>

// #include <iostream>
// #include <cstdio>
extern "C"{
#include "../../../../pathfinderIncludes/pathfinder.h"
}

#pragma once

static const double METERS_IN_FOOT = 0.3048;

class MotionProfileCommand : public AutoCommand{
  public:
    MotionProfileCommand(RobotModel* robot);
    void Init();
    void Update(double currTimeSec, double deltaTimeSec);
    bool IsDone();
    void Reset();
    ~MotionProfileCommand();
  private:
  
    void LoadTrajectory();
  
    RobotModel *robot_;
    int trajectoryLen;
    Segment *leftTrajectory_, *rightTrajectory_;
    EncoderFollower *leftFollower_, *rightFollower_;
    EncoderConfig *leftConfig_, *rightConfig_;    
};

// class MotionProfile {
//  public:
// //   MotionProfile(RobotModel* robot, NavXPIDSource* navXSource, VelocPIDSource* velocSource,
// // 		AnglePIDOutput* anglePIDOutput, VelocPIDOutput* velocPIDOutput);
// //   double GetRelativeAngle();
// //   static double GetCurVeloc();
// //   void Update();
// //  private:
// //   RobotModel *robot_;
// //   NavXPIDSource *navXSource_;
// // 	VelocPIDSource *velocSource_;
// // 	AnglePIDOutput *anglePIDOutput_;
// // 	VelocPIDOutput *velocPIDOutput_;
// // 	PIDController *anglePID_;
// // 	PIDController *velocPID_;
// //   int dPFac_, dIFac_, dDFac_, rPFac_, rIFac_, rDFac_;
// //   double rTolerance_, dTolerance_;
// //   double rMaxOutput_, dMaxOutput_;
  
// //   //where the robot is
// //   int nextPoint;
// //   double lastAngle_, lastEncoderVal_, lastTime_; //time is in seconds
// //   double startAngle_, curX_, curY_, curAngle_;
// //   static double curVeloc_;








//   //below is experimental for reading from file rather than massive point
// //   void CalcSeq();
// //   void AddSeqToFile();
// //   void LoadSeqFromFile(int seqNum);

// //  private:
// //   RobotModel *robot_;
  
// //   std::ofstream sequencesFileO;
// //   std::ifstream sequencesFileI;
// //   std::string currSeq;

// //   std::string outputFolder;
// //   int numSeqsInFile; //keeps track of num files in current run
// };
