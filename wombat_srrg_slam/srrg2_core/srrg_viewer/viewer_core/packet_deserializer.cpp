// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "packet_deserializer.h"

namespace srrg2_core {
  PacketDeserializer::PacketDeserializer() {
    // creating a new factory and registering all the packet types
    _factory = new PacketFactory();
  }

  PacketDeserializer::~PacketDeserializer() {
    delete _factory;
  }

  PacketBase* PacketDeserializer::getPacket() {
    if (!_buffer || !_buffer->size)
      throw std::runtime_error(
        "PacketDeserializer::getPacket|invalid buffer, please set the buffer");

    uint8_t packet_type = PACKET_TYPE_INVALID;

    // deserialize
    const char* b = _buffer->data;

    b = getFromBuffer(packet_type, b);

    PacketBase* packet = _factory->createPacket(packet_type);

    if (!packet) {
      throw std::runtime_error("PacketDeserializer::getPacket|invalid packet type");
    }

    b = packet->deserialize(b);

    // move the data ptr forward 
    size_t offset = (size_t)(b - _buffer->data);
    _buffer->data += offset;

    return packet;
  }

} // namespace srrg2_core
