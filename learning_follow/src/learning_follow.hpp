#pragma once
#include "Etats.hpp"

inline Thetas  operator+(Thetas& theta1,Thetas& theta2) {
  Thetas out;
  out[0]=theta1[0]+theta2[0];
  out[1]=theta1[1]+theta2[1];
  out[2]=theta1[2]+theta2[2];
  out[3]=theta1[3]+theta2[3];
  out[4]=theta1[4]+theta2[4];
  return out;
}

inline std::ostream& operator<<(std::ostream& os, const Thetas& theta) {
  os << theta[0] << ' ' << theta[1] << ' ' <<  theta[2] << ' ' << theta[3]<< ' ' << theta[4];
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const Rayon& r) {
  os << r[0] << ' ' << r[1] << ' ' <<  r[2];
  return os;
}

inline double sqr(double x) {return  x*x;}

inline double norme(Rayon r) {
  double norme = sqrt(sqr(r[0])+sqr(r[1])+sqr(r[2]));
  return norme;
}
