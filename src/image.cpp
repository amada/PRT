#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "image.h"

namespace prt
{

Image::Image(uint32_t width, uint32_t height, bool tonemap, float exposure)
    : m_width(width), m_height(height), m_tonemap(tonemap), m_exposure(exposure)
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

void Image::writePixel(uint32_t x, uint32_t y, const Vector3f& color) {
    auto c = m_exposure*color;
    c = m_tonemap ? c/(c + 1) : c;
    const uint32_t indexBase = (x + y*m_width)*kChannelsPerPixel;
    m_pixels[indexBase + 0] = gamma(clamp(c.x, 0.0f, 1.0f))*0xff;
    m_pixels[indexBase + 1] = gamma(clamp(c.y, 0.0f, 1.0f))*0xff;
    m_pixels[indexBase + 2] = gamma(clamp(c.z, 0.0f, 1.0f))*0xff;
}

void Image::saveToPpm(const char* path) const
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
