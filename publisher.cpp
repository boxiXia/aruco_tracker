//#include <unistd.h>
#include<io.h>
#include <iostream>
#include <sstream>
#include <string>
#include <zmq.hpp>
#include <msgpack.hpp>
#include "message.hpp"
#include <thread>
#include <chrono>

template <typename T>
void publish(zmq::socket_t& socket, const std::string& tag, const T& data)
{
  msgpack::sbuffer packed;
  msgpack::pack(&packed, data);

  zmq::message_t tag_msg(tag.size());
  std::memcpy(tag_msg.data(), tag.data(), tag.size());

  zmq::message_t body_msg(packed.size());
  std::memcpy(body_msg.data(), packed.data(), packed.size());

  socket.send(tag_msg, ZMQ_SNDMORE);
  socket.send(body_msg);
}

int main()
{
  zmq::context_t context(1);
  zmq::socket_t socket(context, ZMQ_PUB);
  socket.bind("tcp://127.0.0.1:12900");

  const std::string tag = "msg";
  while (true)
  {
    const std::time_t now = std::time(0);

    message msg;
    msg.tag = tag;
    msg.time = now;
    msg.text = "hello";

    publish(socket, tag, msg);

    std::cout << now << std::endl;

    
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}