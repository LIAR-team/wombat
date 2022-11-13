// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include<iostream>
#include<string>

namespace srrg2_core {

class ObjectData;

class ObjectParser {
public:
  /*!
   * \brief Read a single message data.
   * \details Read and parse a single object, returning a pointer to the object data, if any; a null pointer
   * is returned in case of parsing error or EOF. <br/>
   * The caller takes ownership of the returned pointer.
   * \param is input stream to read the message from
   * \return the read object's data
   */
  virtual ObjectData* readObject(std::istream& is, std::string& type)=0;

  /*!
   * \brief Read a single message data.
   * \details Read and parse a single object, returning a pointer to the object data, if any; a null pointer
   * is returned in case of parsing error or EOF. <br/>
   * The caller takes ownership of the returned pointer.
   * \param s the log line with the object data
   * \return the read object's data 
   */
  virtual ObjectData* readObject(const std::string& s, std::string& type)=0;

  virtual ~ObjectParser() {};
};

}
