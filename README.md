# Quaternion-to-Rotation-Matrix
This code is design to convert acceleration measured by an IMU from the body frame to the NED frame. To do this, a quaternion rotation produced by an AHRS is converted into a rotation matrix. The inverse of that matrix is then multiplied by the acceleraion vector in the body frame. The code is based on the Arduino framework and utilises the Basic Linear Algebra library from Tom Stewart. https://github.com/tomstewart89/BasicLinearAlgebra   