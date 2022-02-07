#pragma once

#include "vecmath.h"

namespace prt
{

class Image
{
public:
    Image(uint32_t width, uint32_t height);
    virtual ~Image();

    void WritePixel(uint32_t x, uint32_t y, const Vector3f& color) {
        const uint32_t indexBase = (x + y*m_width)*kChannelsPerPixel;
        m_pixels[indexBase + 0] = gamma(clamp(color.x, 0.0f, 1.0f))*0xff;
        m_pixels[indexBase + 1] = gamma(clamp(color.y, 0.0f, 1.0f))*0xff;
        m_pixels[indexBase + 2] = gamma(clamp(color.z, 0.0f, 1.0f))*0xff;
    }

    void SaveToPpm(const char* path) const;

    uint32_t GetWidth() const { return m_width; }
    uint32_t GetHeigit() const { return m_height; }
private:
    const static uint32_t kChannelsPerPixel = 3;
    float gamma(float f)
    {
        return pow(f, 1/2.2f);
    }

    uint8_t* m_pixels;
    uint32_t m_width;
    uint32_t m_height;
};

} // namespace prt
