// ItemSprites.h
// Item sprite arrays (mushroom, coins, etc.)
#pragma once
#include <Arduino.h>

// Item sprite arrays
extern const uint8_t Red_Mushroom_Sprite[7][8];
extern const uint8_t worldCoin[6][6];
extern const uint8_t Green_Mushroom_Sprite[7][8];

// Red Mushroom sprite (7x8)
const uint8_t Red_Mushroom_Sprite[7][8] PROGMEM = {
  { 0, 2, 2, 2, 2, 2, 2, 0 },
  { 2, 2, 6, 2, 2, 6, 2, 2 },
  { 2, 6, 2, 2, 2, 2, 6, 2 },
  { 2, 2, 2, 2, 2, 2, 2, 2 },
  { 0, 1, 1, 1, 1, 1, 1, 0 },
  { 0, 1, 1, 1, 1, 1, 1, 0 },
  { 0, 1, 1, 0, 0, 1, 1, 0 }
};

// Green Mushroom sprite (7x8)
const uint8_t Green_Mushroom_Sprite[7][8] PROGMEM = {
  { 0, 23, 23, 23, 23, 23, 23, 0 },
  { 23, 23, 6, 23, 23, 6, 23, 23 },
  { 23, 6, 23, 23, 23, 23, 6, 23 },
  { 23, 23, 23, 23, 23, 23, 23, 23 },
  { 0, 1, 1, 1, 1, 1, 1, 0 },
  { 0, 1, 1, 1, 1, 1, 1, 0 },
  { 0, 1, 1, 0, 0, 1, 1, 0 }
};

const uint8_t worldCoin[6][6] PROGMEM = {
  { 0, 5, 5, 5, 5, 0 },
  { 5, 5, 13, 13, 5, 5 },
  { 5, 5, 13, 13, 5, 5 },
  { 5, 5, 13, 13, 5, 5 },
  { 5, 5, 13, 13, 5, 5 },
  { 0, 5, 5, 5, 5, 0 }
};
