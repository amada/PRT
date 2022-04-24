#include "log.h"
#include "vecmath.h"
#include "triangle.h"

#include "light.h"


namespace prt
{

void InfiniteAreaLight::init()
{
    m_image.init();
    m_width = 0;
    m_height = 0;
    m_verticalP = nullptr;
    m_horizontalP = nullptr;
}

void InfiniteAreaLight::create(const char* path)
{
    m_image.loadExr(path);

// TODO support for downsampling
    auto width = m_image.width;
    auto height = m_image.height;

    m_width = width;
    m_height = height;

    m_verticalP = new float[height];
    m_horizontalP = new float[width*height];

    auto vert = m_verticalP;
    auto hori = m_horizontalP;
    auto p = (float*)m_image.texels;
    auto vsum = 0.0f;
    for (uint32_t y = 0; y < height; y++) {
        auto hsum = 0.0f;
        for (uint32_t x = 0; x < width; x++) {
            auto indexBase = (x + y*width);

            Vector3f c(p[4*indexBase + 0], p[4*indexBase + 1], p[4*indexBase + 2]);
            float l = length(c); // Should use better method to get intensity
            hori[indexBase] = l;
            hsum += l;
        }

        vert[y] = hsum;
        vsum += hsum;

        auto invH = 1.0f/hsum;
        auto accumH = 0.0f;
        for (uint32_t x = 0; x < width; x++) {
            auto indexBase = (x + y*width);

            auto ph = accumH + invH*hori[indexBase];
            hori[indexBase] = ph;
            accumH = ph;
        }
    }

    auto invV = 1.0f/vsum;
    auto accumV = 0.0f;
    for (uint32_t y = 0; y < height; y++) {
        auto pv = accumV + invV*vert[y];
        vert[y] = pv;
        accumV = pv;
    }
}

void InfiniteAreaLight::sample(Vector3f& dir, Vector3f& color, const Vector2f& u) const
{
    // Pick up sampling point where higher probability is more likely to be selected
    // TODO bisect
    int32_t y;
    float yf = 0.0f;
    for (y = 1; y < m_height; y++) {
        if (m_verticalP[y] > u.y) {
            float prev = m_verticalP[y - 1];
            yf = y + (u.y - prev)/(m_verticalP[y] - prev) - 1.0f;
            break;
        }
    }

    float xf = 0.0f;
    for (int32_t x = 1; x < m_width; x++) {
        if (m_horizontalP[x + y*m_width] > u.x) {
            float prev = m_horizontalP[x - 1 + y*m_width];
            xf = x + (u.x - prev)/(m_horizontalP[x + y*m_width] - prev) - 1.0f;
            break;
        }
    }

    const Vector2f uv(xf/m_width, yf/m_height);
//    const Vector2f uv(0.92f, 0.15f);

    color = m_image.sample<Vector3f>(uv);

//    float theta = 2.0f*kPi*uv.x;
//    float theta = 2.0f*kPi*(uv.x + 0.5f); // For Sponza
    float theta = 2.0f*kPi*(uv.x + 0.5f);
    float phi = kPi*uv.y;
    float sinPhi = std::sin(phi);

    dir = Vector3f(std::cos(theta)*sinPhi, std::cos(phi), std::sin(theta)*sinPhi);
}


} // namespace prt