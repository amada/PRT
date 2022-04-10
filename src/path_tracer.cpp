#include <stdio.h>
#include <cmath>

#include "vecmath.h"
#include "triangle.h"
#include "camera.h"
#include "ray.h"
#include "bvh.h"
#include "scene.h"
#include "image.h"
#include "path_tracer.h"


namespace prt
{

void PathTracer::TraceBlock(Image& image, uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1, const Scene& scene, const Camera& camera, uint32_t samples)
{
    for (uint32_t y = y0; y <= y1; y++) {
        for (uint32_t x = x0; x <= x1; x++) {
            Vector3f color = Trace(camera, scene, x, y, samples);

            color = color/samples;

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
        SingleRayHitPacket hitPacket;
        scene.intersect(hitPacket, SingleRayPacket(ray));

        const auto& hit = hitPacket.hit;
        if (hit.isHit()) {
            auto pos = hit.t*ray.dir + ray.org;
            SurfaceProperties prop;
            scene.getSurfaceProperties(prop, hit);
            color = color + ComputeRadiance(scene, ray.dir, pos, prop);
        } 
    }

    return color;
}

Vector3f PathTracer::SoaTrace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples)
{
    Vector3f color(0);

    for (uint32_t i = 0; i < samples/RayPacket::kSize; i++) {
        auto packet = camera.GenerateJitteredRayPacket(m_rand, x, y);

        RayHitPacket hitPacket;
        scene.intersect(hitPacket, packet);

        for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
            const auto& hit = hitPacket.hits[v];
            const auto& ray = packet.rays[v];
            auto pos = hit.t*ray.dir + ray.org;

            for (uint32_t lane = 0; lane < SoaConstants::kLaneCount; lane++) {
                RayHit sr;
                sr.t = hit.t.getLane(lane);
                sr.i = hit.i.getLane(lane);
                sr.j = hit.j.getLane(lane);
                sr.k = hit.k.getLane(lane);
                sr.primId = hit.primId.getLane(lane);
                sr.meshId = hit.meshId.getLane(lane);
    
                if (sr.isHit()) {
                    SurfaceProperties prop;
                    scene.getSurfaceProperties(prop, sr);
                    
                    color = color + ComputeRadiance(scene, ray.dir.getLane(lane), pos.getLane(lane), prop);
    //                color = color + ComputeRadiance(scene, ray.dir.getLane(lane), pos.getLane(lane), prop, 0);
                } 
            }
        }
    }

    return color;
}


Vector3f PathTracer::ComputeRadiance(const Scene& scene, const Vector3f& rayDir, const Vector3f& apos, const SurfaceProperties& aprop)
{
    Random& rand = m_rand;
    auto prop = aprop;
    auto pos = apos;

    Vector3f result(0.0);
    Vector3f beta(1.0);
    uint32_t depth = 0;

    while (depth < 14) {
        auto material = prop.material;
        auto normal = prop.material->sampleBump(prop);

        if (material != nullptr && material->emissive.x != 0) {
            result = result + beta*material->emissive;
            break;
        }

        Vector3f lightRadiance(0.0);
        Vector3f dir(0.0);
        if (material->reflectionType == ReflectionType::kDiffuse) {
            float r2 = rand.generate();
            float r2sq = std::sqrt(r2);
            float r1 = rand.generate();

            // TODO: should be able to use derivatives in prop
            Vector3f u = (fabsf(normal.x) > 0.1f) ? Vector3f(0, 1.0f, 0.0f) : Vector3f(1.0f, 0, 0);
            auto tangent = normalize(cross(normal, u));
            auto binormal = normalize(cross(tangent, normal));

            float theta = 2.0f*kPi*r1;
            // Cosine-weighted distribution
            dir = r2sq*std::cos(theta) * binormal + r2sq*std::sin(theta)*tangent + (1 - r2)*normal;

            beta = beta*material->sampleDiffuse(prop.uv);

            auto light = scene.getDirectionalLight();

            if (light.intensity.isNonZero()) {
                Ray shadowRay;
                shadowRay.maxT = 100000.0f;
                shadowRay.org = pos;
                shadowRay.dir = light.dir;
                shadowRay.prepare();

                if (!scene.occluded<bool, Ray>(shadowRay)) {
                    lightRadiance = light.intensity*std::max(dot(light.dir, normal), 0.0f)/kPi;
                }
            }
        } else if (material->reflectionType == ReflectionType::kSpecular) {
            float r2 = rand.generate();
            float r2sq = std::sqrt(r2);
            float r1 = rand.generate();
            Vector3f u = (fabsf(normal.x) > 0.1f) ? Vector3f(0, 1.0f, 0.0f) : Vector3f(1.0f, 0, 0);
            auto tangent = normalize(cross(normal, u));
            auto binormal = normalize(cross(tangent, normal));

            float theta = 2.0f*kPi*r1;
            // Cosine-weighted distribution
            auto diffuseDir = r2sq*std::cos(theta) * binormal + r2sq*std::sin(theta)*tangent + (1 - r2)*normal;
            auto reflectDir = rayDir - normal*2*dot(normal, rayDir);

            dir = 0.9f*reflectDir + 0.1f*diffuseDir;
        }

        result = result + beta*lightRadiance;
        dir = normalize(dir);

        Ray ray;
        ray.maxT = 10000.0f;
        ray.org = pos;
        ray.dir = dir;
        ray.prepare();

        SingleRayHitPacket hitPacket;
        scene.intersect(hitPacket, SingleRayPacket(ray));

        const auto& hit = hitPacket.hit;
        if (hit.isHit()) {
            scene.getSurfaceProperties(prop, hit);

            pos = hit.t*ray.dir + ray.org;
        } else {
            break;
        }

        // Russian roulette
        if (depth > 4) {
            float q = std::max(0.05f, 1.0f - length(beta));
            if (rand.generate() < q) {
                break;
            }

            beta = beta/(1.0f - q);
        }

        depth++;
    }

    return result;
}
//Vector3f PathTracer::ComputeRadiance(const Scene& scene, const Vector3f& rayDir, const Vector3f& pos, const Vector3f& normal, const Material* material, uint32_t depth)
Vector3f PathTracer::ComputeRadiance(const Scene& scene, const Vector3f& rayDir, const Vector3f& pos, const SurfaceProperties& prop, uint32_t depth)
{
    Random& rand = m_rand;

    auto material = prop.material;
    auto normal = prop.normal;

    if (material != nullptr && material->emissive.x != 0) {
//        return 0;
        return material->emissive;
    }

    if (depth >= 14) {
        return Vector3f(0);
    }

    Vector3f mpos = pos;
    Vector3f dir;

    Vector3f resultColor(0);

    Vector3f lightRadiance(0.0f);

    Vector3f diffuse(0.0f);

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
            RayHit intr;
            scene.intersect(intr, ray);

            if (intr.isHit()) {
                SurfaceProperties prop;
                scene.getSurfaceProperties(prop, intr);
                auto radiance = ComputeRadiance(scene, reflDir, ray.dir*intr.t + ray.org, prop, depth + 1);

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

    RayHit intr;
    scene.intersect(intr, ray);

    if (intr.isHit()) {
        SurfaceProperties prop;
        scene.getSurfaceProperties(prop, intr);
        auto radiance = ComputeRadiance(scene, dir, ray.dir*intr.t + ray.org, prop, depth + 1);
        if (m_verbose) {
        }

        resultColor = resultColor + (l*radiance + lightRadiance)*diffuse;
    }

    return resultColor;
}

} // namespace prt