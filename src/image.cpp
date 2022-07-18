#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-but-set-variable"
#pragma clang diagnostic ignored "-Wunused-function"
#include "../ext/tinyexr/tinyexr.h"
#pragma clang diagnostic pop

#include "image.h"

namespace
{
    float gamma(float f)
    {
        return pow(f, 1/2.2f);
    }

    prt::Vector3f gamma(const prt::Vector3f& v)
    {
        return prt::Vector3f(gamma(v.x), gamma(v.y), gamma(v.z));
    }
}

namespace prt
{

Image::Image(uint32_t width, uint32_t height, bool tonemap, float exposure)
    : m_width(width), m_height(height), m_tonemap(tonemap), m_exposure(exposure)
{
    size_t size = 3*width*height;
    m_pixels = new float[size];
    memset(m_pixels, 0, size);
}

Image::~Image()
{
    delete[] m_pixels;
    m_pixels = nullptr;
}

void Image::writePixel(uint32_t x, uint32_t y, const Vector3f& color) {
    auto c = m_exposure*color;
    const uint32_t indexBase = (x + y*m_width)*kChannelsPerPixel;
    m_pixels[indexBase + 0] = c.x;
    m_pixels[indexBase + 1] = c.y;
    m_pixels[indexBase + 2] = c.z;
}

void Image::savePpm(const char* path) const
{
    FILE *fp = fopen(path, "wb");
    if (!fp) {
        printf("Error saving %s\n", path);
        return;
    }

    auto pixels = new uint8_t[kChannelsPerPixel*m_width*m_height];
    for (uint32_t i = 0; i < m_width*m_height; i++) {
        const uint32_t indexBase = i*kChannelsPerPixel;
        Vector3f c(m_pixels[indexBase + 0], m_pixels[indexBase +1], m_pixels[indexBase + 2]);
        c = m_tonemap ? c/(c + 1) : c;
        c = gamma(clamp(c, 0.0f, 1.0f))*0xff;
        pixels[indexBase + 0] = c.x;
        pixels[indexBase + 1] = c.y;
        pixels[indexBase + 2] = c.z;
    }

    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", m_width, m_height);
    fprintf(fp, "255\n");
    fwrite(pixels, m_width*m_height*kChannelsPerPixel, 1, fp);
    fclose(fp);

    delete[] pixels;

    printf("Save %s\n", path);
}

void Image::saveExr(const char* path) const
{
    EXRHeader header;
    InitEXRHeader(&header);

    EXRImage image;
    InitEXRImage(&image);

    image.num_channels = 3;

    std::vector<float> images[3];
    images[0].resize(m_width*m_height);
    images[1].resize(m_width*m_height);
    images[2].resize(m_width*m_height);

    // Split RGBRGBRGB... into R, G and B layer
    for (int32_t i = 0; i < m_width*m_height; i++) {
        images[0][i] = m_pixels[3*i+0];
        images[1][i] = m_pixels[3*i+1];
        images[2][i] = m_pixels[3*i+2];
    }

    float* image_ptr[3];
    image_ptr[0] = &(images[2].at(0)); // B
    image_ptr[1] = &(images[1].at(0)); // G
    image_ptr[2] = &(images[0].at(0)); // R

    image.images = (unsigned char**)image_ptr;
    image.width = m_width;
    image.height = m_height;

    header.num_channels = 3;
    header.channels = (EXRChannelInfo*)malloc(sizeof(EXRChannelInfo)*header.num_channels);
    // Must be (A)BGR order, since most of EXR viewers expect this channel order.
    strncpy(header.channels[0].name, "B", 255); header.channels[0].name[strlen("B")] = '\0';
    strncpy(header.channels[1].name, "G", 255); header.channels[1].name[strlen("G")] = '\0';
    strncpy(header.channels[2].name, "R", 255); header.channels[2].name[strlen("R")] = '\0';

    header.pixel_types = (int *)malloc(sizeof(int)*header.num_channels);
    header.requested_pixel_types = (int *)malloc(sizeof(int)*header.num_channels);
    for (int32_t i = 0; i < header.num_channels; i++) {
        header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
        header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_HALF;
    }

    const char* err = nullptr;
    int32_t ret = SaveEXRImageToFile(&image, &header, path, &err);
    if (ret != TINYEXR_SUCCESS) {
        fprintf(stderr, "Save EXR err: %s\n", err);
        FreeEXRErrorMessage(err); // free's buffer for an error message
        return;
    }
    printf("Saved %s\n", path);

    free(header.channels);
    free(header.pixel_types);
    free(header.requested_pixel_types);
}

} // namespace prt
