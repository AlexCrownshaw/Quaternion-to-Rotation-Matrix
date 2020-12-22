#include <BasicLinearAlgebra.h>

using namespace BLA;

double accX {1}, accY {2}, accZ {3};
double accN {}, accE {}, accD {};
double qW {1}, qX {0.5}, qY {4.3}, qZ {0.68};
double qN {};

BLA::Matrix<3> accBodyVector;
BLA::Matrix <3> accNEDVector;
BLA::Matrix <3,3> rotMatrix;
BLA::Matrix <3,3> rotMatrixInv;

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

  while (1) {
  }
}
