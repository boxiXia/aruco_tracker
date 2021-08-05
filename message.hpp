#include <string>
#include <ctime>
#include <msgpack.hpp>

struct message
{
  std::string tag;
  std::time_t time;
  std::string text;
  MSGPACK_DEFINE(tag, time, text);
};