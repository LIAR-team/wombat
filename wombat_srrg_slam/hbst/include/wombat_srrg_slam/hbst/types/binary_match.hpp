#pragma once

#include "wombat_srrg_slam/hbst/types/binary_matchable.hpp"

namespace srrg_hbst {

//! @class elementary match object: inspired by opencv cv::DMatch
//! (docs.opencv.org/trunk/d4/de0/classcv_1_1DMatch.html)
//! @param MatchableType_ matchable type (class) for the match
//! @param real_precision_ matching distance precision
template <typename BinaryMatchableType_, typename real_type_ = double>
struct BinaryMatch
{
  using Matchable  = BinaryMatchableType_;
  using ObjectType = typename Matchable::ObjectType;
  using real_type  = real_type_;

  //! @brief default constructor for an uninitialized match
  //! @returns an uninitialized match
  BinaryMatch()
  : matchable_query(nullptr), distance(0)
  {}

  //! @brief default constructor for an uninitialized match
  //! @returns a fully initialized match
  BinaryMatch(
    const Matchable* matchable_query_,
    const Matchable* matchable_reference_,
    ObjectType pointer_query_,
    ObjectType pointer_reference_,
    const real_type_& distance_)
  : matchable_query(matchable_query_),
    object_query(pointer_query_),
    distance(distance_)
  {
    matchable_references.push_back(matchable_reference_);
    object_references.push_back(std::move(pointer_reference_));
  }

  //! @brief copy constructor
  //! @param[in] match_ binary match object to be copied from
  //! @returns a binary match copy of the match_
  BinaryMatch(const BinaryMatch& match_)
  : matchable_query(match_.matchable_query),
    matchable_references(std::move(match_.matchable_references)),
    object_query(std::move(match_.object_query)),
    object_references(std::move(match_.object_references)),
    distance(match_.distance)
  {}

  //! @brief default destructor: nothing to do
  ~BinaryMatch() = default;

  //! @brief attributes
  const Matchable* matchable_query;
  // ds multiple references are possible for identical matching distance
  std::vector<const Matchable*> matchable_references;
  ObjectType object_query;
  // ds multiple references are possible for identical matching distance
  std::vector<ObjectType> object_references;
  real_type distance;
};

} // namespace srrg_hbst
