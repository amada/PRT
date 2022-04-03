#pragma once

#include "vecmath.h"

namespace prt
{

class Image
{
public:
    Image(uint32_t width, uint32_t height, bool tonemap = true);
    virtual ~Image();

    void WritePixel(uint32_t x, uint32_t y, const Vector3f& color) {
        auto c = m_tonemap ? color/(color + 1) : color;
        const uint32_t indexBase = (x + y*m_width)*kChannelsPerPixel;
        m_pixels[indexBase + 0] = gamma(clamp(c.x, 0.0f, 1.0f))*0xff;
        m_pixels[indexBase + 1] = gamma(clamp(c.y, 0.0f, 1.0f))*0xff;
        m_pixels[indexBase + 2] = gamma(clamp(c.z, 0.0f, 1.0f))*0xff;
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
    bool m_tonemap;
};

} // namespace prt
