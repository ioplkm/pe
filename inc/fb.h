#include <stdint.h>

#include <linux/fb.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define    red 0x00FF0000
#define yellow 0x00FFFF00
#define  green 0x0000FF00
#define   blue 0x000000FF
#define  white 0x00FFFFFF
#define  black 0x00000000

uint32_t *buffer;
struct fb_var_screeninfo info;

#include "vector.h"

void drawPoint(uint32_t x, uint32_t y, uint32_t color) {
  for (uint32_t i = x - 3; i < x + 3; i++)
    for (uint32_t j = y - 3; j < y + 3; j++)
      buffer[j * info.xres + i] = color;
}

void drawV(Vector v, uint32_t color) {
   drawPoint((int)(v.x * 10) + 960, (int)(-v.y * 10) + 540, color);
}

void fbInit() {
  int fbfd = open("/dev/fb0", O_RDWR);
  ioctl(fbfd, FBIOGET_VSCREENINFO, &info);
  buffer = mmap(NULL, 4 * info.xres * info.yres, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
  for (uint32_t i = 0; i < info.xres; i++)
    for (uint32_t j = 0; j < info.yres; j++)
      buffer[j * info.xres + i] = black;
}
