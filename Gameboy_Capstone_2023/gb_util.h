#pragma once
#ifndef GB_UTIL_H_
#define GB_UTIL_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

bool util_check_bit(const uint8_t value, const uint8_t n);

void util_set_bit(uint8_t* byte, const uint8_t n);
void util_unset_bit(uint8_t* byte, const uint8_t n);

#endif
