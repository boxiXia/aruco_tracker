//#include <unistd.h>
#include<io.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <zmq.hpp>
#include <msgpack.hpp>
#include "message.hpp"

template <typename T>
void subscribe(zmq::socket_t& socket, T& data)
{
  zmq::message_t tag_msg, body_msg;
  socket.recv(&tag_msg);
  socket.recv(&body_msg);

  const std::string tag(static_cast<const char*>(tag_msg.data()), tag_msg.size());

  msgpack::unpacked unpacked_body;
  msgpack::unpack(&unpacked_body, static_cast<const char*>(body_msg.data()), body_msg.size());
  unpacked_body.get().convert(&data);
}

int main()
{
  zmq::context_t context(1);
  zmq::socket_t socket(context, ZMQ_SUB);
  socket.connect("tcp://127.0.0.1:12900");

  const std::string tag = "msg";
  socket.setsockopt(ZMQ_SUBSCRIBE, tag.c_str(), tag.size());

  while (true)
  {
    message msg;
    subscribe(socket, msg);
    std::cout << msg.time << " [" << msg.tag << "] " << msg.text << std::endl;
  }
}