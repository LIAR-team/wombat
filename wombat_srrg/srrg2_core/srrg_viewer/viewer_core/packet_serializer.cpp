// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "packet_serializer.h"
#include "packets.h"

namespace srrg2_core {
  PacketSerializer::PacketSerializer() {
  }

  PacketSerializer::~PacketSerializer() {
  }

  void PacketSerializer::putPacket(const PacketBase& packet_) {
    if (!_buffer || !_buffer->size)
      throw std::runtime_error("[PacketSerializer] invalid buffer, please set the buffer");

    if (packet_.type == PACKET_TYPE_INVALID)
      throw std::runtime_error("[PacketSerializer] invalid packet type");

    // we put first the packet type
    _buffer->data = putInBuffer(_buffer->data, packet_.type);

    // finally the payload
    _buffer->data = packet_.serialize(_buffer->data);

    // increment the packet counter
    ++_buffer->num_packets;
  }

} // namespace srrg2_core
