#include <BasicLinearAlgebra.h>
#define _USE_MATH_DEFINES
#include <cmath>

using namespace BLA;

double accX {0}, accY {0}, accZ {1};
double accN {}, accE {}, accD {};
double qW {1}, qX {0}, qY {0}, qZ {0}, qN {};
double heading {}, pitch {}, roll {};

BLA::Matrix<3> accBodyVector;
BLA::Matrix <3> accNEDVector;
BLA::Matrix <3,3> rotMatrix;
BLA::Matrix <3,3> rotMatrixInv;

double rad_to_deg(double);

void setup() {
  Serial1.begin(9600);
}

void loop() {
  accBodyVector(0) = accX;
  accBodyVector(1) = accY;
  accBodyVector(2) = accZ;

  Serial1.println("print Body Acceleration Vector");
  for (int i {}; i < accBodyVector.GetRowCount(); i++)  {
    Serial1.println(accBodyVector(i));
  }
  Serial1.println("=================================================");

  qN = sqrt((qW * qW) + (qX * qX) + (qY * qY)+ (qZ * qZ));
  qW /= qN;
  qX /= qN;
  qY /= qN;
  qZ /= qN;

  Serial1.println("Print Normalised Quaternion");
  Serial1.print(qW, 6);
  Serial1.print("\t");
  Serial1.print(qX, 6);
  Serial1.print("\t");
  Serial1.print(qY, 6);
  Serial1.print("\t");
  Serial1.println(qZ, 6);
  Serial1.println("=================================================");

  rotMatrix(0,0) = (1 - (2 * qY * qY) - (2 * qZ * qZ)); rotMatrix(0,1) = ((2 * qX * qY) - (2 * qZ * qW)); rotMatrix(0,2) = ((2* qX * qZ) + (2 * qY * qW));
  rotMatrix(1,0) = ((2 * qX * qY) + (2 * qZ * qW)); rotMatrix(1,1) = (1 - (2 * qX * qX) - (2 * qZ *qZ));  rotMatrix(1,2) = ((2 * qY * qZ) - (2 * qX * qW));
  rotMatrix(2,0) = ((2 * qX * qZ) - (2 * qY * qW)); rotMatrix(2,1) = ((2 * qY * qZ) + (2 * qX * qW)); rotMatrix(2,2) = (1 - (2 * qX * qX) - (2 * qY * qY));
  
  Serial1.println("Print Rotation Matrix");
  for (int i {}; i < rotMatrix.GetColCount(); i++)  {
    for (int j {}; j < rotMatrix.GetRowCount(); j++)  {
      Serial1.print(rotMatrix(i,j), 6);
      Serial1.print("\t");
    }
    Serial1.println();
  }
  Serial1.println("=================================================");

  rotMatrixInv = rotMatrix.Inverse();

  Serial1.println("Print Inverse Rotation Matrix");
  for (int i {}; i < rotMatrixInv.GetColCount(); i++) {
    for (int j {}; j < rotMatrixInv.GetRowCount(); j++) {
      Serial1.print(rotMatrixInv(i,j), 6);
      Serial1.print("\t");
    }
    Serial1.println();
  }
  Serial1.println("=================================================");

  accNEDVector = rotMatrixInv * accBodyVector;
  
  Serial1.println("print NED Acceleration Vector");
  for (int i {}; i < accNEDVector.GetRowCount(); i++)  {
    Serial1.println(accNEDVector(i));
  }
  Serial1.println("=================================================");

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (qW * qX + qY * qZ);
    double cosr_cosp = 1 - 2 * (qX * qX + qY * qY);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (qW * qY - qZ * qX);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (qW * qZ + qX * qY);
    double cosy_cosp = 1 - 2 * (qY * qY + qZ * qZ);
    heading = std::atan2(siny_cosp, cosy_cosp);

  Serial1.println("Print Orientation");
  Serial1.print(roll, 6);
  Serial1.print("\t");
  Serial1.print(pitch, 6);
  Serial1.print("\t");
  Serial1.println(heading, 6);

  Serial1.println("=================================================");

  while (1) {
  }
}

double rad_to_deg(double val) {
  return val * 180/PI;
}
