#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "image.h"

namespace prt
{

Image::Image(uint32_t width, uint32_t height) : m_width(width), m_height(height)
{
    size_t size = 3*width*height;
    m_pixels = new uint8_t[size];
    memset(m_pixels, 0, size);
}

Image::~Image()
{
    delete[] m_pixels;
    m_pixels = nullptr;
}

void Image::SaveToPpm(const char* path) const
{
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        printf("Error saving %s\n", path);
        return;
    }

    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", m_width, m_height);
    fprintf(fp, "255\n");
    fwrite(m_pixels, m_width*m_height*kChannelsPerPixel, 1, fp);
    fclose(fp);
    printf("Save %s\n", path);
}

} // namespace prt
