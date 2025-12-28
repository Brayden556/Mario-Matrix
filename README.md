# 🍄 Mario Matrix

**A custom Super Mario–inspired platformer running on real LED matrix hardware**

Mario Matrix is a fully custom-built game engine designed to run on a **192×64 HUB75 RGB LED matrix**, powered by **ESP32 microcontrollers**.  
This project combines embedded systems, game engine design, custom tooling, and hardware integration into a single playable physical game.

The v1 build was completed and gifted as a one-of-a-kind system — and this repository captures that shipped version.

---

## 🎮 Project Overview

Mario Matrix is:
- A **tile-based platformer engine**
- Running on **real hardware**, not emulation
- Built with **ESP32 + HUB75 LED panels**
- Controlled using a **Bluetooth game controller**
- Supported by **custom-built editors** for maps and sprites

Inspired by **Super Mario World**, but re-engineered for an LED matrix environment.

---

## ✨ Key Features

- Tile-based world and rendering engine  
- Multiple overworld maps  
- Slopes and collision physics  
- Animated lava with damage handling  
- Flagpole finish and checkpoints  
- Attract mode with demo playback  
- HUD and weather systems  
- Enemy movement and interaction logic  
- Bluetooth controller input (Bluepad32)  
- Optimized for ESP32 memory and timing constraints  

---

## 🛠️ Hardware Used (v1)

This project was designed so others can **rebuild or extend it** without exotic components.

### 🧠 Microcontrollers

**ESP32**
- Dedicated to Bluetooth controller input
- Runs Bluepad32
- Keeps input handling isolated from rendering

**ESP32-S3**
- Runs the full game engine
- Handles rendering, logic, physics, enemies, and HUD
- Drives the HUB75 LED matrix
- Uses PSRAM for large buffers and sprite data

Using two microcontrollers improves timing stability and prevents Bluetooth tasks from interfering with rendering.

---

### 🎮 Controller

**8BitDo SN30 Pro (Bluetooth)**
- SNES-style layout
- Low latency
- Widely supported by Bluepad32

Other compatible Bluetooth controllers should also work.

---

### 🟥 Display

**RGB LED Matrix Panels**
- Quantity: **3**
- Resolution: **64×64 each**
- Pitch: **2.5mm**
- Interface: **HUB75**
- Total resolution: **192×64 (12,288 pixels)**

Panels are chained and treated as a single continuous display.

---

### ⚡ Power

**MEAN WELL LRS-150F-5**
- Output: **5V**
- Current: **22A**
- Power: **110W**

Chosen for reliability, clean power delivery, and sufficient headroom for brightness spikes.

> LED matrices draw significant current — undersizing the PSU is not recommended.

---

## 🔌 Power & Wiring (High Level)

- LED panels are powered directly from the PSU
- ESP32 boards share a common ground with the panels
- Brightness is software-limited to manage current draw
- Proper wire gauge and fusing are strongly recommended

(Detailed wiring diagrams may be added later.)

---

## 🧰 Tooling & Editors

Custom tools were built to make this project manageable.

### 🗺️ Matrix Map Editor
- Web-based visual editor
- Build levels by placing tiles and large sprite arrays
- Exports map data directly into firmware-readable formats

### 🎨 Sprite & Tile Editors
- Custom workflows for:
  - Tile arrays
  - Large sprites
  - Palette-based pixel art
- Designed specifically for LED matrix constraints

**Shoutout to the editors** — without them, this project would not have completed.

---

## 📂 Repository Layout (High Level)

Mario-Matrix/
├─ MarioMatrix_ESPS3.ino/ # Game engine + display firmware
├─ MarioController_ESP32.ino/ # Bluetooth controller firmware
├─ Matrix_Editor.html # Sprite editor
├─ Matrix_Map_Editor.html # Map editor
├─ Docs and references

---

## 🚀 Project Status

- ✅ v1 complete
- 🎁 Delivered as a physical gift
- 🚧 v2 planned (engine polish, new content, hardware refinements)

This repository represents the **v1 gifted build**.

---

## 💡 Why This Project Exists

Mario Matrix began as a curiosity:
> “Can a Mario-style platformer run on an LED matrix with a microcontroller?”

It became:
- An embedded systems challenge
- A custom game engine
- A full hardware/software build
- And a meaningful personal project

---

## ❤️ Final Notes

This project was built with:
- A lot of late nights
- Careful optimization
- Custom tooling
- And a strong desire to make something tangible and fun

If this repository helps you build something similar — or inspires you to try — that’s a huge win.

🍄🎮

