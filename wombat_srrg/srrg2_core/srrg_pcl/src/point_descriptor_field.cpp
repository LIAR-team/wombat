// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_pcl/point_descriptor_field.h"

namespace srrg2_core {

  int PointDescriptorFieldCvMatBinary::distance(
    const PointDescriptorFieldCvMatBinary& other_) const
  {
    return cv::norm(value, other_.value, cv::NORM_HAMMING);
  }

  int PointDescriptorFieldCvMatFloat::distance(
    const PointDescriptorFieldCvMatFloat& other_) const
  {
    return cv::norm(value, other_.value, cv::NORM_L2);
  }

  int PointDescriptorFieldBit256::distance(
    const PointDescriptorFieldBit256& other_) const
  {
    // TODO implement architecture specific Hamming distance computation
    return (value ^ other_.value).count();
  }

  void PointDescriptorFieldBit256::from(
    const PointDescriptorFieldCvMatBinary& other_) 
  {
    from(other_.value);
  }

  void PointDescriptorFieldBit256::from(const cv::Mat& other_value_)
  {
    assert(other_value_.type() == CV_8UC1);
    assert(other_value_.cols == 32);

    // assign exactly 32 bytes to 256 bits
    for (uint32_t byte_index = 0;
         byte_index < static_cast<uint32_t>(other_value_.cols);
         ++byte_index) {
      const uint32_t bit_index_start = byte_index * 8;

      // grab a byte and convert it to a bitset so we can access the single
      // bits
      const std::bitset<8> descriptor_byte(other_value_.at<uchar>(byte_index));

      // set bitstring
      for (uint8_t v = 0; v < 8; ++v) {
        value[bit_index_start + v] = descriptor_byte[v];
      }
    }
  }
  int PointDescriptorFieldFloat32::distance(
    const PointDescriptorFieldFloat32& other_) const
  {
    const ValueType& other_value = other_.value;

    // TODO move to reduce in C++17
    // TODO optimize for architecture
    std::array<float, 32> difference;
    std::set_difference(value.begin(),
                        value.end(),
                        other_value.begin(),
                        other_value.end(),
                        difference.begin());
    return std::accumulate(
      difference.begin(), difference.end(), 0 /*initial distance*/);
  }
} // namespace srrg2_core
