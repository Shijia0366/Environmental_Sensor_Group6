#pragma once
#include <Arduino.h>

struct Status {
  bool ok;
  String err;

  static Status Ok()                  { return { true,  ""    }; }
  static Status Err(const String& e)  { return { false, e     }; }
};

template<typename T>
struct Result {
  bool ok;
  T value;
  String err;

  static Result<T> Ok(const T& v)         { return { true,  v, "" }; }
  static Result<T> Err(const String& e)   { return { false, T{}, e }; }
};
