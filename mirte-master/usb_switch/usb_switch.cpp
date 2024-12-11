
#include <signal.h>
#include <stdlib.h>
#include <vector>

#include <gpiod.hpp>
#include <iostream>
#include <string.h>
#include <unistd.h>

class gpio_pin {
public:
  gpio_pin(std::string name);
  std::string name;
  char block = 'A';
  std::string chip_name = "gpiochip0";
  int block_line = 0;
  int line = 0;
  gpiod::chip chip;
  gpiod::line gpio_line;
};

std::vector<gpio_pin> pins;
void stop_handler(int s) {
  for (const auto &pin : pins) {
    std::cout << "Turn off " << pin.name << std::endl;
    pin.gpio_line.set_value(0);
    pin.gpio_line.release();
  }
  exit(0);
}

int main(void) {
  struct sigaction sigIntHandler;

  sigIntHandler.sa_handler = stop_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;

  sigaction(SIGINT, &sigIntHandler, NULL);
  sigaction(SIGTERM, &sigIntHandler, NULL);

  gpio_pin pin("GPIO4_C3");

  pins.push_back(pin);
  pin.gpio_line.request(
      {"usb_switch", gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
  pin.gpio_line.set_value(0); // Force off before turning on.
  sleep(1);
  pin.gpio_line.set_value(1);
  pause();
}

gpio_pin::gpio_pin(std::string pin_name) {
  this->name = pin_name;
  // name should be GPIOx_yz, with 0<=x<=5, A<=y<=D, 0<=z<=7
  this->chip_name = (std::string)("gpiochip") + pin_name[4];
  this->block = pin_name[6];
  this->block_line = (int)(pin_name[7] - '0');
  this->line = 8 * (this->block - 'A') + this->block_line;
  this->chip = ::gpiod::chip(this->chip_name);
  this->gpio_line = chip.get_line(this->line);
  std::cout << "new pin" << pin_name << std::endl;
}