#pragma once


typedef std::array<double, 3> Rayon; //vecteur à 3 dim r
typedef std::array<double, 5> Thetas;//vecteur à 5 dim Theta (seul les 4 premiers sont utilisés)

class Etat{
  Rayon r;
  Thetas theta;
  Thetas dthetamin;

public :

  Etat(const Rayon& r, const Thetas& theta, const Thetas& dthetamin)
  : r(r), theta(theta), dthetamin(dthetamin)  {
  }
  Etat()                       = default;
  Etat(const Etat&)            = default;
  Etat& operator=(const Etat&) = default;
};

typedef std::vector<Etat> BaseEtats;
