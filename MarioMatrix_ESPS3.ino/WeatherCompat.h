#pragma once

#include <WiFiClientSecure.h>
#include <utility>

namespace WeatherCompat {

// Some ESP32 Arduino cores require explicitly setting the SSL hostname (SNI) for
// Google endpoints. Different core versions expose different APIs.
//
// IMPORTANT: This lives in a header so Arduino's .ino auto-prototype generator
// doesn't break template signatures.

template <typename T>
static inline auto trySetSSLHostname(T &client, const char *host, int)
  -> decltype(client.setSSLHostname(host), void()) {
  client.setSSLHostname(host);
}

template <typename T>
static inline void trySetSSLHostname(T &, const char *, long) {
}

template <typename T>
static inline auto trySetHostname(T &client, const char *host, int)
  -> decltype(client.setHostname(host), void()) {
  client.setHostname(host);
}

template <typename T>
static inline void trySetHostname(T &, const char *, long) {
}

static inline void setSNIIfSupported(WiFiClientSecure &client, const char *host) {
  trySetSSLHostname(client, host, 0);
  trySetHostname(client, host, 0);
}

// Some cores expose setBufferSizes(rx, tx). Reducing these can significantly lower
// heap use during TLS handshakes.
template <typename T>
static inline auto trySetBufferSizes(T &client, int rx, int tx, int)
  -> decltype(client.setBufferSizes(rx, tx), void()) {
  client.setBufferSizes(rx, tx);
}

template <typename T>
static inline void trySetBufferSizes(T &, int, int, long) {
}

static inline void setBufferSizesIfSupported(WiFiClientSecure &client, int rx, int tx) {
  trySetBufferSizes(client, rx, tx, 0);
}

} // namespace WeatherCompat
