// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include <list>
#include <string>
#include <map>

namespace srrg2_core {
  struct Chrono{
    using ChronoMap=std::map<std::string, std::list<std::pair<double, double> > >;
    Chrono(const std::string& tag_,
           ChronoMap* time_map_,
           bool verbose_);
    
    Chrono(const std::string& tag_,
           double& accumulator_,
           bool verbose_);
    
    ~Chrono();
    double duration() const;
    static void printReport(const ChronoMap& chrono_map);
  protected:
    bool _verbose;
    std::string _tag;
    double _time;
    double* _accumulator=nullptr;
    ChronoMap* _time_map=nullptr;
  };

}
