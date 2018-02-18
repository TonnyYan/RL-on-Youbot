#pragma once


typedef std::array<double, 4> Joints;
typedef std::vector<Joints>   JointsSet;

//Redéfinition de l'opérateur "<<"
inline std::ostream& operator<<(std::ostream& os, const Joints& coord_recues) {
  os << coord_recues[0] << ' ' << coord_recues[1] << ' ' <<  coord_recues[2] << ' ' << coord_recues[3];
  return os;
}

inline std::istream& operator>>(std::istream& is, Joints& coord_recues) {
  is >> coord_recues[0] >> coord_recues[1] >> coord_recues[2] >> coord_recues[3];
  return is;
}

inline double sqr(double x) {return  x*x;}

inline double distance2(const Joints& coord1, const Joints coord2) {
  return sqr(coord1[0]-coord2[0])+sqr(coord1[1]-coord2[1])+sqr(coord1[2]-coord2[2])+sqr(coord1[3]-coord2[3]);
}
