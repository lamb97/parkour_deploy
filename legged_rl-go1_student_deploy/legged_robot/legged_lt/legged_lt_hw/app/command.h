//
// Created by bismarck on 11/19/22.
//

#ifndef MASTERSTACK_COMMAND_H
#define MASTERSTACK_COMMAND_H

#include <iostream>
#include <vector>
#include <unistd.h>
#include <chrono>
#include "queue.h"

extern "C" {
#include "config.h"
#include "motor_control.h"
#include "transmit.h"
}

unsigned help(const std::vector<std::string> &);
unsigned motorIdGet(const std::vector<std::string> & input);
unsigned motorIdSet(const std::vector<std::string> & input);
unsigned motorSpeedSet(const std::vector<std::string> & input);
unsigned motoPositionSet(const std::vector<std::string> & input);

#endif //MASTERSTACK_COMMAND_H
