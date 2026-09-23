#pragma once

namespace remus::hal {

class IStorage {
public:
  virtual ~IStorage() = default;
  virtual bool begin() = 0;
  virtual bool healthy() const = 0;
  virtual const char* name() const = 0;
};

}  // namespace remus::hal
