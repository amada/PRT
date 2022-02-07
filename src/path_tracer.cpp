#include <stdio.h>
#include <cmath>

#include "vecmath.h"
#include "triangle.h"
#include "camera.h"
#include "ray.h"
#include "bvh.h"
#include "image.h"
#include "path_tracer.h"


namespace prt
{

void PathTracer::TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera, uint32_t samples)
{
    for (uint32_t y = y0; y <= y1; y++) {
        for (uint32_t x = x0; x <= x1; x++) {
            Vector3f color = Trace(camera, scene, x, y, samples);

            color = 1.0f*2/samples*color;

            image.WritePixel(x, y, color);
        }
    }    
}

Vector3f PathTracer::Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples)
{
    return SoaTrace(camera, scene, x, y, samples);
    Vector3f color(0);

    for (uint32_t i = 0; i < samples; i++) {
        auto ray = camera.GenerateJitteredRay(m_rand, x, y);
        RayIntersection intr;
        scene.intersect(intr, ray);

        if (intr.isHit()) {
            auto pos = intr.t*ray.dir + ray.org;
            SurfaceProperties prop;
            scene.getSurfaceProperties(prop, intr);
            color = color + ComputeRadiance(scene, ray.dir, pos, prop.normal, prop.material, 0);
        } 
    }

    return color;
}

Vector3f PathTracer::SoaTrace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples)
{
    Vector3f color(0);

    for (uint32_t i = 0; i < samples/SoaConstants::kLaneCount; i++) {
        auto ray = camera.GenerateJitteredSoaRay(m_rand, x, y);

        SoaRayIntersection intr;
        scene.intersect(intr, ray);

        RayIntersection scalarRes[SoaConstants::kLaneCount];

        auto pos = intr.t*ray.dir + ray.org;

        for (uint32_t lane = 0; lane < SoaConstants::kLaneCount; lane++) {
            auto& sr = scalarRes[lane];
            sr.t = intr.t.getLane(lane);
            sr.i = intr.i.getLane(lane);
            sr.j = intr.j.getLane(lane);
            sr.k = intr.k.getLane(lane);
            sr.primId = intr.primId.getLane(lane);
            sr.meshId = intr.meshId.getLane(lane);
 
            if (sr.isHit()) {
                SurfaceProperties prop;
                scene.getSurfaceProperties(prop, sr);
                
                color = color + ComputeRadiance(scene, ray.dir.getLane(lane), pos.getLane(lane), prop.normal, prop.material, 0);
            } 
        }
    }

    return color;
}

Vector3f PathTracer::ComputeRadiance(const Scene& scene, const Vector3f& rayDir, const Vector3f& pos, const Vector3f& normal, const Material* material, uint32_t depth)
{
    Random& rand = m_rand;

    if (material != nullptr && material->emissive.x != 0) {
        return material->emissive;
    }

    if (depth >= 14) {
        return Vector3f(0);
    }

    Vector3f mpos = pos;
    Vector3f dir;

    Vector3f resultColor(0);

    float l;
    if (material->reflectionType == ReflectionType::kDiffuse) {
        float r2 = rand.generate();
        float r2sq = std::sqrt(r2);
        float r1 = rand.generate();
        Vector3f u = (fabsf(normal.x) > 0.1f) ? Vector3f(0, 1.0f, 0.0f) : Vector3f(1.0f, 0, 0);
        auto tangent = normalize(cross(normal, u));
        auto binormal = normalize(cross(tangent, normal));

        float theta = 2.0f*kPi*r1;
        // Cosine-weighted distribution
        dir = r2sq*std::cos(theta) * binormal + r2sq*std::sin(theta)*tangent + (1 - r2)*normal;

        l = 1.0f;
    } else if (material->reflectionType == ReflectionType::kSpecular) {
        dir = rayDir - normal*2*dot(normal, rayDir);
        l = 1.0f;
    } else {
        bool into = dot(rayDir, normal) < 0; // Ray from outside going in?

        constexpr float etaA = 1.0f; // Air
        constexpr float etaT = 1.5f; // Transmissive
        auto N = into ? normal : -normal;
        float eta = into ? etaA/etaT : etaT/etaA;
        float NI = -dot(N, rayDir);
        constexpr float F0 = (1.0f - etaT)*(1.0f - etaT)/((1.0f + etaT)*(1.0f + etaT));

        float F = F0 + (1.0f - F0)*(powf(1.0 - NI, 5.0f));

        auto reflDir = rayDir - normal * 2 * dot(normal, rayDir);

        reflDir = normalize(reflDir);
 
        float cosThetaI = NI;
        float sin2ThetaI = std::max(0.0f, 1.0f - cosThetaI*cosThetaI);
        float sin2ThetaT = eta*eta*sin2ThetaI;

        if (sin2ThetaT >= 1.0f) {
            // TODO don't fire a ray for refraction
            F = 1.0f;
        } else {
            float cosThetaT = std::sqrt(1.0f - sin2ThetaT);
            dir = eta * rayDir + (eta * NI - cosThetaT) * N;
        }

        if (depth < 2) {
            Ray ray;
            ray.maxT = 10000.0f;
            ray.org = mpos;
            ray.dir = reflDir;

            ray.prepare();
            RayIntersection intr;
            scene.intersect(intr, ray);

            if (intr.isHit()) {
                SurfaceProperties prop;
                scene.getSurfaceProperties(prop, intr);
                auto radiance = ComputeRadiance(scene, reflDir, ray.dir*intr.t + ray.org, prop.normal, prop.material, depth + 1);

                resultColor = F*radiance*material->diffuse;
            }

            l = 1.0f - F;
        } else {
            l = 1.0f;
            if (F > rand.generate()) {
                dir = reflDir;
            }
        }
    }
    
    dir = normalize(dir);

    Ray ray;
    ray.maxT = 10000.0f;
    ray.org = mpos;
    ray.dir = dir;
    ray.prepare();

    RayIntersection intr;
    scene.intersect(intr, ray);

    if (intr.isHit()) {
        SurfaceProperties prop;
        scene.getSurfaceProperties(prop, intr);
        auto radiance = ComputeRadiance(scene, dir, ray.dir*intr.t + ray.org, prop.normal, prop.material, depth + 1);
        if (m_verbose) {
        }

        resultColor = resultColor + l*radiance*material->diffuse;
    }

    return resultColor;
}

} // namespace prt