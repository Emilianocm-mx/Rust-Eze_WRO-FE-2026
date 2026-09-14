#pragma once
#include <stdint.h>
#include <stddef.h>

// Zona activa: conservar coordenadas y resolucion originales.
static const int IGNORE_TOP_PERCENT = 25;
static const int IGNORE_SIDE_PERCENT = 0; // Por CADA lado; 0 desactiva el recorte lateral.
static_assert(IGNORE_SIDE_PERCENT >= 0 && IGNORE_SIDE_PERCENT < 50,
              "IGNORE_SIDE_PERCENT debe estar entre 0 y 49");
static_assert(IGNORE_TOP_PERCENT >= 0 && IGNORE_TOP_PERCENT < 100,
              "IGNORE_TOP_PERCENT debe estar entre 0 y 99");
inline int detectionTop(int height) {
  return height * IGNORE_TOP_PERCENT / 100;
}

inline int detectionLeft(int width) { return width * IGNORE_SIDE_PERCENT / 100; }
inline int detectionRight(int width) { return width - detectionLeft(width); } // Exclusivo.

// AJUSTES: H en grados (0..359); S y V en escala 0..255.
static int RED_H_MAX = 10;
static int RED_H_MIN = 345;
static int GREEN_H_MIN = 95;
static int GREEN_H_MAX = 130; // Muestras H=104..118, con margen.
static int RED_MIN_S = 75; // Muestras S=90..98; tolerar color algo mas apagado.
static int GREEN_MIN_S = 60; // Muestras S=72..83, con margen.
static int MIN_V = 60;
static int MIN_AREA = 35;       // Pixeles de color conectados.
static int MIN_HEIGHT = 7;
static const int MIN_WIDTH = 5;
static const int MIN_FILL_PERCENT = 45;
static const int MIN_ASPECT_PERCENT = 95;  // Alto/ancho >= 0.95.
static const int MAX_ASPECT_PERCENT = 600;
static const int MAX_OBJECTS = 16;

struct Pillar {
  int color, x, y, w, h, area;
  bool clipped;
};

inline uint8_t classifyRGB(int r, int g, int b) {
  int hi = r > g ? r : g;
  if (b > hi) hi = b;
  int lo = r < g ? r : g;
  if (b < lo) lo = b;
  int delta = hi - lo;
  if (hi < MIN_V || delta == 0) return 0;
  int hue;
  if (hi == r) hue = 60 * (g - b) / delta;
  else if (hi == g) hue = 120 + 60 * (b - r) / delta;
  else hue = 240 + 60 * (r - g) / delta;
  if (hue < 0) hue += 360;
  if ((hue <= RED_H_MAX || hue >= RED_H_MIN) && delta * 255 >= RED_MIN_S * hi) return 1;
  if (hue >= GREEN_H_MIN && hue <= GREEN_H_MAX && delta * 255 >= GREEN_MIN_S * hi) return 2;
  return 0;
}

// mask y queue deben tener width*height elementos. Maximo 65535 pixeles.
// Conserva el color en bits bajos; bit 7 marca visitados, sin otra matriz.
inline int findPillars(uint8_t *mask, uint16_t *queue, int width, int height,
                      Pillar *out) {
  // Enmascarar antes de agrupar: nada puede conectarse por fuera de la zona activa.
  const int left=detectionLeft(width), right=detectionRight(width), top=detectionTop(height);
  for(int y=0;y<height;++y)
    for(int x=0;x<width;++x)
      if(y<top || x<left || x>=right)mask[y*width+x]=0;
  int found = 0;
  for (int start = 0; start < width * height; ++start) {
    uint8_t color = mask[start];
    if (color != 1 && color != 2) continue;
    int head = 0, tail = 0;
    queue[tail++] = (uint16_t)start;
    mask[start] |= 128;
    int minX = start % width, maxX = minX;
    int minY = start / width, maxY = minY;
    while (head < tail) {
      int index = queue[head++];
      int x = index % width, y = index / width;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          int nx = x + dx, ny = y + dy;
          if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
          int next = ny * width + nx;
          if (mask[next] == color) {
            mask[next] |= 128;
            queue[tail++] = (uint16_t)next;
          }
        }
      }
    }
    int w = maxX - minX + 1, h = maxY - minY + 1;
    if (tail < MIN_AREA || h < MIN_HEIGHT || w < MIN_WIDTH ||
        tail * 100 < w * h * MIN_FILL_PERCENT ||
        h * 100 < w * MIN_ASPECT_PERCENT ||
        h * 100 > w * MAX_ASPECT_PERCENT) continue;
    if (found == MAX_OBJECTS) return -1;
    out[found++] = {color, minX, minY, w, h, tail,
                   minX <= left || minY <= top || maxX >= right-1 || maxY == height-1};
  }
  // Mayor altura primero. Empate: izquierda primero (solo convencion).
  for (int i = 1; i < found; ++i) {
    Pillar p = out[i];
    int j = i - 1;
    while (j >= 0 && (out[j].h < p.h || (out[j].h == p.h && out[j].x > p.x))) {
      out[j+1] = out[j];
      --j;
    }
    out[j+1] = p;
  }
  return found;
}
