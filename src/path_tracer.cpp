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
#if 0
    TraverseStackCache stackCache;
    scene.createStackCache(stackCache, camera.getPosition(), camera.getDirection());
#endif
    for (uint32_t y = y0; y <= y1; y++) {
        for (uint32_t x = x0; x <= x1; x++) {
//            Vector3f color = SoaTrace(camera, scene, x, y, samples, &stackCache);
            Vector3f color = SoaTrace(camera, scene, x, y, samples, nullptr);

            color = color/samples;

            image.writePixel(x, y, color);
        }
    }    
}

Vector3f PathTracer::Trace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples)
{
//    return SoaTrace(camera, scene, x, y, samples);
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

// Name should be packet tracing
Vector3f PathTracer::SoaTrace(const Camera& camera, const Scene& scene, uint32_t x, uint32_t y, uint32_t samples, const TraverseStackCache* stackCache)
{
    Vector3f color(0);

#ifdef PRT_ENABLE_STATS
    m_stats.raysTraced += samples;
#endif

    for (uint32_t i = 0; i < samples/RayPacket::kSize; i++) {
        auto packet = camera.GenerateJitteredRayPacket(m_rand, x, y);

        RayHitPacket hitPacket;
        scene.intersect(hitPacket, packet, stackCache);

#if 1
        color = color + ComputeRadiance(scene, hitPacket, packet);

#else
#ifdef PRT_ENABLE_STATS
        m_stats.merge(hitPacket.stats);
#endif

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
        #endif
    }

    return color;
}

Vector3f PathTracer::ComputeRadiance(const Scene& scene, const RayHitPacket& hitPacket, const RayPacket& packet)
{
    Random& rand = m_rand;

    Vector3f result[RayPacket::kSize];
    Vector3f beta[RayPacket::kSize];

    for (uint32_t i = 0; i < RayPacket::kSize; i++) {
        beta[i] = 1.0f;
        result[i] = 0.0f;
    }

    Vector3f pos[RayPacket::kSize];
    Vector3f rayDir[RayPacket::kSize];
    SurfaceProperties props[RayPacket::kSize];
    const Material* materials[RayPacket::kSize];
    Vector3f normals[RayPacket::kSize];

    uint32_t alivePaths = 0;

    for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
        const auto& hit = hitPacket.hits[v];
        const auto& ray = packet.rays[v];
        auto vpos = hit.t*ray.dir + ray.org;

        for (uint32_t lane = 0; lane < SoaConstants::kLaneCount; lane++) {
            RayHit sr;
            sr.t = hit.t.getLane(lane);
            sr.i = hit.i.getLane(lane);
            sr.j = hit.j.getLane(lane);
            sr.k = hit.k.getLane(lane);
            sr.primId = hit.primId.getLane(lane);
            sr.meshId = hit.meshId.getLane(lane);

            if (sr.isHit()) {
                scene.getSurfaceProperties(props[alivePaths], sr);
                materials[alivePaths] = props[alivePaths].material;
                normals[alivePaths] = materials[alivePaths]->sampleBump(props[alivePaths]);
                pos[alivePaths] = vpos.getLane(lane);
                rayDir[alivePaths] = ray.dir.getLane(lane);
                alivePaths++;
            }
        }
    }

    uint32_t depth = 0;

    while (depth < 14) {
        Vector3f lightDir[RayPacket::kSize];
        Vector3f lightIntensity[RayPacket::kSize];
        Vector3f nextRayDir[RayPacket::kSize];

        bool directLighting = false;

        // Generate next scatter ray and occlude ray for direct lighting
        for (uint32_t path = 0; path < alivePaths; path++) {
            auto material = materials[path];
            auto normal = normals[path];
            auto prop = props[path];
            
            if (material->emissive.x != 0) {
                result[path] = result[path] + beta[path]*material->emissive;
            }

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
                nextRayDir[path] = r2sq*std::cos(theta) * binormal + r2sq*std::sin(theta)*tangent + (1 - r2)*normal;

                beta[path] = beta[path]*material->sampleDiffuse(prop.uv);

                if (scene.isLightAvailable(LightType::kInfiniteArea)) {
                    auto& light = scene.getInfiniteAreaLight();
                    light.sample(lightDir[path], lightIntensity[path], Vector2f(rand.generate(), rand.generate()));
                    directLighting = true;
                } else if (scene.isLightAvailable(LightType::kDirectional)) {
                    auto light = scene.getDirectionalLight();
                    lightDir[path] = light.dir;
                    lightIntensity[path] = light.intensity;
                    directLighting = true;
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
                auto rdir = rayDir[path];
                auto reflectDir = rdir - normal*2*dot(normal, rdir);

                nextRayDir[path] = 0.9f*reflectDir + 0.1f*diffuseDir;
            }
        }

        const float kFar = 2.0f*scene.getRadius();
        const float kEpsilon = 0.0008f; // TODO may want this to be relative value to kFar

        // Trace occlude rays
        if (directLighting) {
            // Whether dispatching ray packet or single ray
            bool grouping = (alivePaths & 0xf) > 2;
            if (grouping) {
                RayPacket shadowRay;

                uint32_t maskBits = 0;

                auto vectorCount = (alivePaths + SoaConstants::kLaneCount - 1)/SoaConstants::kLaneCount;
                for (uint32_t v = 0; v < vectorCount; v++) {
                    auto& ray = shadowRay.rays[v];
                    auto vlightDir = SoaVector3f(&lightDir[v*SoaConstants::kLaneCount]);
                    auto vpos = SoaVector3f(&pos[v*SoaConstants::kLaneCount]);
                    ray.maxT = kFar - kEpsilon;
                    ray.org = vpos + SoaFloat(kFar)*vlightDir;
                    ray.dir = vlightDir*-1.0f;
                    ray.prepare();
                }

                maskBits = (1 << alivePaths) - 1;
                shadowRay.avgDir = 0.0f; // avgDir isn't used for occlude

#ifdef PRT_ENABLE_STATS
                m_stats.raysTraced += alivePaths;
                m_stats.occludedTraced += alivePaths;
#endif                

                auto omask = scene.occluded<RayPacketMask, RayPacket>(RayPacketMask(maskBits), shadowRay);

                for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
                    for (auto bits = omask.masks[v].computeNot().ballot(); bits; bits &= bits - 1) {
                        auto index = bitScanForward(bits);
                        auto path = v*SoaConstants::kLaneCount + index;
                        auto lightRadiance = lightIntensity[path]*std::max(dot(lightDir[path], normals[path]), 0.0f)/kPi;
                        result[path] = result[path] + beta[path]*lightRadiance;
                    }
                }
            } else {
                for (uint32_t i = 0; i < alivePaths; i++) {
                    Ray shadowRay;
                    shadowRay.maxT = kFar - kEpsilon;
                    shadowRay.org = pos[i] + kFar*lightDir[i];
                    shadowRay.dir = -lightDir[i];

                    shadowRay.prepare();
#ifdef PRT_ENABLE_STATS
                    m_stats.raysTraced++;
                    m_stats.occludedTraced++;
#endif
        
                    if (!scene.occluded<bool, SingleRayPacket>(RayPacketMask(), shadowRay)) {
                        auto lightRadiance = lightIntensity[i]*std::max(dot(lightDir[i], normals[i]), 0.0f)/kPi;
                        result[i] = result[i] + beta[i]*lightRadiance;
                    }
                }
            }
        }

        // Remove paths according Russian roulette and trace scatter rays
        uint32_t ci = 0; // compacted index;
        for (uint32_t i = 0; i < alivePaths; i++) {
            // Russian roulette
            if (depth > 4) {
                float q = std::max(0.05f, 1.0f - length(beta[i]));
                if (rand.generate() < q) {
                    continue;
                } else {
                    beta[ci] = beta[i]/(1.0f - q);
                }
            }

            auto dir = normalize(nextRayDir[i]);

            Ray ray;
            ray.maxT = kFar;
            ray.org = pos[i];
            ray.dir = dir;
            ray.prepare();

#ifdef PRT_ENABLE_STATS
            m_stats.raysTraced++;;
#endif
            SingleRayHitPacket hitPacket;
            scene.intersect(hitPacket, SingleRayPacket(ray));

            const auto& hit = hitPacket.hit;
            if (hit.isHit()) {
                scene.getSurfaceProperties(props[ci], hit);

                pos[ci] = hit.t*ray.dir + ray.org;
                normals[ci] = props[i].normal;
                materials[ci] = props[i].material;
                normals[ci] = materials[ci]->sampleBump(props[ci]);
                rayDir[ci] = dir;

                ci++;
            }
        }

        if (ci == 0)
            break;

        alivePaths = ci;

        depth++;
    }

    Vector3f res(0.0f);
    for (uint32_t i = 0; i < RayPacket::kSize; i++) {
        res = res + result[i];
    }
    return res;
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

            const float kFar = 2.0f*scene.getRadius();
            const float kEpsilon = 0.0008f; // TODO may want this to be relative value to kFar

            const uint32_t kMaxNumOccludeRays = 1;
            bool groupingOccludeRays = false;
            bool directLighting = false;
            uint32_t numOccludeRays = 0;
            Vector3f lightDir[kMaxNumOccludeRays];
            Vector3f lightIntensity[kMaxNumOccludeRays];
            Vector3f avgDir(0.0f);

            if (scene.isLightAvailable(LightType::kInfiniteArea)) {
                auto& light = scene.getInfiniteAreaLight();

                float dotmul = 1.0f;
                for (uint32_t l = 0; l < kMaxNumOccludeRays; l++) {
                    light.sample(lightDir[l], lightIntensity[l], Vector2f(rand.generate(), rand.generate()));
                    if (l > 0) {
                        dotmul *= std::abs(dot(lightDir[l], lightDir[l - 1]));
                    }
                    avgDir = avgDir + lightDir[l];
                }

                groupingOccludeRays = kMaxNumOccludeRays > 1 && dotmul > 0.5f;
                directLighting = true;
                numOccludeRays = kMaxNumOccludeRays;
            } else if (scene.isLightAvailable(LightType::kDirectional)) {
                auto light = scene.getDirectionalLight();
                lightDir[0] = light.dir;
                lightIntensity[0] = light.intensity;
                directLighting = true;
                numOccludeRays = 1;
            }

            if (directLighting) {
                if (groupingOccludeRays) {
                    static_assert(kMaxNumOccludeRays < RayPacket::kSize, "Not firing more than a single RayPacket");
                    SoaRay shadowRay;
                    shadowRay.maxT = kFar - kEpsilon;
                    shadowRay.dir = SoaVector3f(lightDir)*-1.0f;
                    shadowRay.org = SoaVector3f(pos) - (SoaFloat(kFar)*shadowRay.dir);
                    shadowRay.prepare();

                    RayPacket packet;
                    packet.rays[0] = shadowRay;
                    packet.avgDir = avgDir/RayPacket::kSize;

                    RayPacketMask mask(~0xf); // 4 rays;

    #ifdef PRT_ENABLE_STATS
                    m_stats.raysTraced += kMaxNumOccludeRays;
                    m_stats.occludedTraced += kMaxNumOccludeRays;
    #endif

                    auto omask = scene.occluded<RayPacketMask, RayPacket>(mask, packet);

                    for (uint32_t v = 0; v < RayPacket::kVectorCount; v++) {
                        auto bits = omask.masks[v].computeNot().ballot();
                        while (bits) {
                            auto index = bitScanForward(bits);
                            lightRadiance = lightRadiance + lightIntensity[index]*std::max(dot(lightDir[index], normal), 0.0f)/kPi;

                            bits &= bits - 1;
                        }
                    }
                } else {
                    for (uint32_t l = 0; l < numOccludeRays; l++) {
                        Ray shadowRay;
                        shadowRay.maxT = kFar - kEpsilon;
                        shadowRay.org = pos + kFar*lightDir[l];
                        shadowRay.dir = -lightDir[l];

                        shadowRay.prepare();
    #ifdef PRT_ENABLE_STATS
                        m_stats.raysTraced++;
                        m_stats.occludedTraced++;
    #endif
                        if (!scene.occluded<bool, SingleRayPacket>(RayPacketMask(), {shadowRay})) {
                            lightRadiance = lightRadiance + lightIntensity[l]*std::max(dot(lightDir[l], normal), 0.0f)/kPi;
                        }
                    }
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

#ifdef PRT_ENABLE_STATS
    m_stats.raysTraced++;;
#endif
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

#if 0
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
#endif

} // namespace prt