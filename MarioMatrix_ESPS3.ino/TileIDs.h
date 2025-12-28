// TileIDs.h
// ----------------------------------------------------
// Single source of truth for all tile ID numbers.
//
// Rules:
// - This file defines ONLY tile IDs (numbers) and related comments.
// - No map grids.
// - No sprite pixel arrays.
//
// IMPORTANT:
// These IDs must remain stable because:
// - The web editors export numeric tile IDs.
// - Saved maps rely on numeric IDs.
// - Gameplay logic checks specific tile IDs (pipes, hazards, etc.).

#ifndef TILE_IDS_H
#define TILE_IDS_H

#include <Arduino.h>

// Tile IDs used across maps, gameplay logic, and rendering.
// Values must match the editor palette/export.
enum TileID : uint8_t {
  // 0 = EMPTY
  TILE_ID_EMPTY = 0,

  // 1 = Ground_Tile_Array (main grass + dirt)
  TILE_ID_GROUND = 1,

  // 2 = Ground_Tile_Top_Left (cliff corner)
  TILE_ID_GROUND_TOP_LEFT = 2,

  // 3 = Ground_Tile_Top_Right (cliff corner)
  TILE_ID_GROUND_TOP_RIGHT = 3,

  // 4 = Ground_Tile_Wall_Left (left vertical wall)
  TILE_ID_GROUND_WALL_LEFT = 4,

  // 5 = Ground_Tile_Wall_Right (right vertical wall)
  TILE_ID_GROUND_WALL_RIGHT = 5,

  // 6 = Ground_Tile_Center_Dirt (underground dirt fill)
  TILE_ID_GROUND_CENTER_DIRT = 6,

  // 7 = Ground_Tile_Slope_Up_Right (slope ascending right)
  TILE_ID_SLOPE_UP_RIGHT = 7,

  // 8 = Ground_Tile_Slope_Up_Left (slope ascending left)
  TILE_ID_SLOPE_UP_LEFT = 8,

  // 9 = Ground_Tile_Grass_Side_Left
  TILE_ID_GRASS_SIDE_LEFT = 9,

  // 10 = Ground_Tile_Grass_Side_Right
  TILE_ID_GRASS_SIDE_RIGHT = 10,

  // 11 = Ground_Tile_Fill_Left
  TILE_ID_FILL_LEFT = 11,

  // 12 = Ground_Tile_Fill_Right
  TILE_ID_FILL_RIGHT = 12,

  // 13 = Ground_Tile_Grass_Dirt_Fill_Left
  TILE_ID_GRASS_DIRT_FILL_LEFT = 13,

  // 14 = Ground_Tile_Grass_Dirt_Fill_Right
  TILE_ID_GRASS_DIRT_FILL_RIGHT = 14,

  // 15 = Ground_Tile_Grass_Straight_Side_Right
  TILE_ID_GRASS_STRAIGHT_SIDE_RIGHT = 15,

  // 16 = Ground_Tile_Grass_Straight_Side_Left
  TILE_ID_GRASS_STRAIGHT_SIDE_LEFT = 16,

  // 17 = Ground_Tile_Vert_Cliff_Left
  TILE_ID_VERT_CLIFF_LEFT = 17,

  // 18 = Ground_Tile_Vert_Cliff_Right
  TILE_ID_VERT_CLIFF_RIGHT = 18,

  // 19 = Ground_Tile_Green_Tunnel (18×17 pixels)
  TILE_ID_GREEN_TUNNEL = 19,

  // 20 = Ground_Tile_Bush (25×8 pixels)
  TILE_ID_BUSH = 20,

  // 21 = SemiGround_Tile_Array
  TILE_ID_SEMI_GROUND = 21,

  // 22 = Ground_Tile_Green_Tunnel_Down (18×17 pixels)
  TILE_ID_GREEN_TUNNEL_DOWN = 22,

  // 23 = Tile_Brick_Question_Array (Question block with mushroom)
  TILE_ID_Q_BLOCK = 23,

  // 24 = Tile_Brick_Solid_Array (Non-breaking solid block)
  TILE_ID_SOLID_BLOCK = 24,

  // 25 = Tile_Brick_Breakable_Array (Breakable brick)
  TILE_ID_BRICK = 25,

  // 26 = Tile_Brick_Used_Brick_Array (Used/empty block)
  TILE_ID_USED_BLOCK = 26,

  // 27 = worldCoin (Collectible coin, 6×6 pixels)
  TILE_ID_COIN = 27,

  // 28 = Item :: Green_Mushroom_Sprite (reserved)
  TILE_ID_GREEN_MUSHROOM = 28,

  // 29 = Tile_Brick_Coin_Array (render-only)
  TILE_ID_BRICK_COIN = 29,

  // 30 = Tile_Brick_GreenMushroom_Array (render-only green 1UP-brick)
  TILE_ID_GREEN_MUSHROOM_BRICK = 30,

  // 31 = Ground_Tile_Stone_Ledge (terrain)
  TILE_ID_STONE_LEDGE = 31,

  // 32 = Ground_Tile_Green_Tunnel_Left (left entrance, 17×18 pixels)
  TILE_ID_GREEN_TUNNEL_LEFT = 32,

  // 33 = Under_Ground_Tile_Roof_Array (underground terrain)
  TILE_ID_UNDERGROUND_ROOF = 33,

  // 34 = Under_Ground_Tile_Wall_Right (underground terrain)
  TILE_ID_UNDERGROUND_WALL_RIGHT = 34,

  // 35 = Under_Ground_Tile_Ground (underground terrain)
  TILE_ID_UNDERGROUND_GROUND = 35,

  // 36 = Under_Ground_Tile_Wall_Left (underground terrain)
  TILE_ID_UNDERGROUND_WALL_LEFT = 36,

  // 37 = Under_Ground_Tile_Roof_Fill_Left (underground terrain)
  TILE_ID_UNDERGROUND_ROOF_FILL_LEFT = 37,

  // 38 = Under_Ground_Tile_Roof_Fill_Right (underground terrain)
  TILE_ID_UNDERGROUND_ROOF_FILL_RIGHT = 38,

  // 39 = Under_Ground_Tile_Ground_Fill_Right (underground terrain)
  TILE_ID_UNDERGROUND_GROUND_FILL_RIGHT = 39,

  // 40 = Under_Ground_Tile_Ground_Fill_Left (underground terrain)
  TILE_ID_UNDERGROUND_GROUND_FILL_LEFT = 40,

  // 41 = Ground_Tile_Green_Tunnel_1_2 (named upright entry for pair 1)
  TILE_ID_GREEN_TUNNEL_1_2 = 41,

  // 42 = Mob :: Goomba (paintable/renderable mob tile)
  // NOTE: ID 43 is reserved for legacy maps (old Goomba option); do not reuse.
  TILE_ID_MOB_GOOMBA = 42,

  // 44 = Ground_Tile_Green_NoTel_Tunnel (18×17 pixels) - solid, never teleports
  TILE_ID_GREEN_TUNNEL_NOTEL = 44,

  // 45-46 = Mob :: Piranha Plant (render-only overlay mob; does not affect tunnels)
  // Placement uses a 2-tile-wide footprint:
  // - _L is the anchor tile that draws the sprite.
  // - _R is an invisible helper tile.
  TILE_ID_MOB_PIRANHA_PLANT_L = 45,
  TILE_ID_MOB_PIRANHA_PLANT_R = 46,
  // Legacy alias (older exports used a single ID)
  TILE_ID_MOB_PIRANHA_PLANT = TILE_ID_MOB_PIRANHA_PLANT_L,

  // 47 = Mob :: Mario Spawn marker (overlay-only; controls hero spawn on reset)
  TILE_ID_MOB_MARIO_SPAWN = 47,

  // 48-49 = Checkpoint flags (visual only for now)
  TILE_ID_CHECK_POINT = 48,
  TILE_ID_CHECKED_POINT = 49,

  // 50-52 = Finish flag (decoration)
  TILE_ID_FLAG_TOP = 50,
  TILE_ID_FLAG_POLE = 51,
  TILE_ID_FLAG_BASE = 52,

  // 53-54 = Background clouds (decoration)
  TILE_ID_BG_CLOUD_PUFF_A = 53,
  TILE_ID_BG_CLOUD_LONG_A = 54,

  // 55 = Brick cloud platform (acts as semi-solid)
  TILE_ID_BRICK_CLOUD = 55,

  // 56 = Mario Castle (finish line decoration, 40×40)
  TILE_ID_MARIO_CASTLE = 56,

  // 57 = Mob :: Koopa Troopa (terrain-spawn mob tile)
  TILE_ID_MOB_KOOPA_TROOPA = 57,

  // 58 = Ground_Tile_Lava_1 (animated by swapping frames)
  TILE_ID_LAVA = 58,

  // 59 = Ground_Tile_Green_Tunnel_Down_2_2 (named down/exit for pair 1)
  TILE_ID_GREEN_TUNNEL_DOWN_2_2 = 59,

  // 60 = Ground_Tile_Green_Tunnel_Left_2_4 (named left entrance for pair 2)
  TILE_ID_GREEN_TUNNEL_LEFT_2_4 = 60,

  // 61 = Ground_Tile_Green_Tunnel_4_4 (named upright exit for pair 2)
  TILE_ID_GREEN_TUNNEL_4_4 = 61
};

// Back-compat alias for prompts/docs that use TILEID_* naming
#define TILEID_MARIO_CASTLE   TILE_ID_MARIO_CASTLE

#endif // TILE_IDS_H
