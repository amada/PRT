#pragma once

#include <stdint.h>
#include <math.h>
#include <cmath>

//#define R_NEON
#define R_AVX4L
//#define R_AVX8L

#if defined(R_NEON)
//#define __aarch64__
#include <arm_neon.h>
#else
#include <xmmintrin.h>
#include <immintrin.h>

#if defined(R_AVX4L)
#define AVX_INT(x) _mm_ ## x
#elif defined(R_AVX8L)
#define AVX_INT(x) _mm256_ ## x
#endif

#endif


namespace prt
{

#if defined(R_NEON)
using floatvec_t = float32x4_t;
using intvec_t = int32x4_t;
using maskvec_t = uint32x4_t;
#elif defined(R_AVX4L)
using floatvec_t = __m128;
using intvec_t = __m128i;
using maskvec_t = __m128;
#elif defined(R_AVX8L)
using floatvec_t = __m256;
using intvec_t = __m256i;
using maskvec_t = __m256;
#endif

// TODO: Should have something like kRayPacketSize somewhere
class SoaConstants {
public:
#ifdef R_AVX8L
    static const uint32_t kLaneCount = 8;
#else
    static const uint32_t kLaneCount = 4;
#endif
};

// clang
inline int32_t bitScanForward(int32_t bits) {
    return __builtin_ctz(bits);
}

#ifdef R_NEON
// Arm Neon
inline floatvec_t _neon_div(floatvec_t n, floatvec_t d) {
    // aarch64 vdivq_f32
    auto r = vrecpeq_f32(d);
    r = vmulq_f32(vrecpsq_f32(d, r), r);
    r = vmulq_f32(vrecpsq_f32(d, r), r);
    return vmulq_f32(n, r);
}

inline floatvec_t _neon_sqrt(floatvec_t f) {
    // aarch64 vsqrtq_f32
    auto e = vrsqrteq_f32(f); // Estimation of 1/sqrt(f)
    // vrsqrtsq_f32(d, xn) = (3 - d*xn)/2
    // e*(3-f*e*e)/2
    e = vmulq_f32(vrsqrtsq_f32(vmulq_f32(f, e), e), e);
    return vmulq_f32(f, e); // f/sqrt(f)
}

inline int32_t _neon_movemask(maskvec_t m) {
    int32_t r = (vgetq_lane_u32(m, 0) ? 1 : 0) << 0;
    r |= (vgetq_lane_u32(m, 1) ? 1 : 0) << 1;
    r |= (vgetq_lane_u32(m, 2) ? 1 : 0) << 2;
    r |= (vgetq_lane_u32(m, 3) ? 1 : 0) << 3;

    return r;
}

inline bool _neon_anytrue(maskvec_t m) {
    uint32_t b = vgetq_lane_u32(m, 0);
    b |= vgetq_lane_u32(m, 1);
    b |= vgetq_lane_u32(m, 2);
    b |= vgetq_lane_u32(m, 3);
    return (b != 0);
}

template<typename V, typename T>
inline T _neon_getLane(V v, int32_t lane) {
    union {
        T i[SoaConstants::kLaneCount];
        V v;
    } u;
    u.v = v;
    return u.i[lane];
}
#endif

static const float kPi = M_PI;


// TODO: Add unaligned Vector3f
class Vector3f
{
public:
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float v[3];
    };

    Vector3f() = default;
    Vector3f(float f) : x(f), y(f), z(f) {}
    Vector3f(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

    uint32_t GetLongestElement() const {
        if (x > y) {
            if (x > z) {
                return 0;
            } else {
                return 2;
            }
        } else {
            if (y > z) {
                return 1;
            } else {
                return 2;
            }
        }
    }

    void print() const;

    void set(float _x, float _y, float _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    Vector3f operator-() const {
        return Vector3f(-x, -y, -z);
    }

    Vector3f operator-(const Vector3f& v) const {
        return Vector3f(x - v.x, y - v.y, z - v.z);
    }

    Vector3f operator+(const Vector3f& v) const {
        return Vector3f(x + v.x, y + v.y, z + v.z);
    }

    Vector3f operator*(const Vector3f& v) const {
        return Vector3f(x*v.x, y*v.y, z*v.z);
    }

    Vector3f operator/(const Vector3f& v) const {
        return Vector3f(x/v.x, y/v.y, z/v.z);
    }

    Vector3f operator*(float f) const {
        return Vector3f(f*x, f*y, f*z);
    }

    friend Vector3f operator*(float f, const Vector3f& v) {
        return Vector3f(f*v.x, f*v.y, f*v.z);
    }

    friend Vector3f operator/(float f, const Vector3f& v) {
        return Vector3f(f/v.x, f/v.y, f/v.z);
    }
};

class Vector4
{
public:
    float x;
    float y;
    float z;
    float w;
};



class SoaMask
{
    friend class SoaFloat;
    friend class SoaVector3f;

public:
#if defined(R_AVX8L)
    static const int32_t kAllTrue = 0xff;
#else
    static const int32_t kAllTrue = 0xf;
#endif

    SoaMask() = default;

    SoaMask(maskvec_t mask) : m_mask(mask) {
    }

    static SoaMask initAllTrue() {
        SoaMask r;
#if defined(R_NEON)
        r.m_mask = vdupq_n_u32(0xffffffffu);
#elif defined(R_AVX4L)
        r.m_mask = _mm_castsi128_ps(_mm_set1_epi32(0xffffffff));
#elif defined(R_AVX8L)
        r.m_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0xffffffff));
#endif
        return r;
    }

    maskvec_t getRawValue() const {
        return m_mask;
    }

    int32_t getLane(int32_t lane) const {
#if defined(R_NEON)
        return _neon_getLane<intvec_t, int32_t>(m_mask, lane);
#else
        union {
            float f;
            int32_t i;
        } temp;
#if defined(R_AVX4L)
        temp.f = _mm_cvtss_f32(_mm_permutevar_ps(m_mask, _mm_set1_epi32(lane)));
#elif defined(R_AVX8L)
        temp.f = _mm256_cvtss_f32(_mm256_permutevar_ps(m_mask, _mm256_set1_epi32(lane)));
#endif
        return temp.i;
#endif
    }

    SoaMask computeXor(const SoaMask& m) const {
#if defined(R_NEON)
        return veorq_u32(m_mask, m.m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(xor_ps)(m_mask, m.m_mask);
#endif
    }

    SoaMask computeOr(const SoaMask& m) const {
#if defined(R_NEON)
        return vorrq_u32(m_mask, m.m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(or_ps)(m_mask, m.m_mask);
#endif
    }

    SoaMask computeNot() const {
#if defined(R_NEON)
        return vmvnq_u32(m_mask);
#elif defined(R_AVX4L)
        return SoaMask(_mm_xor_ps(m_mask, 
            _mm_cmpeq_epi32(_mm_castps_si128(m_mask), _mm_castps_si128(m_mask))));
#elif defined(R_AVX8L)
        return SoaMask(_mm256_xor_ps(m_mask, 
            _mm256_cmpeq_epi32(_mm256_castps_si256(m_mask), _mm256_castps_si256(m_mask))));
#endif
    }

    SoaMask computeAnd(const SoaMask& m) const {
#if defined(R_NEON)
        return vandq_u32(m_mask, m.m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(and_ps)(m_mask, m.m_mask);
#endif
    }

    SoaMask operator|(const SoaMask& m) const {
        return computeOr(m);
    }

    SoaMask operator&(const SoaMask& m) const {
        return computeAnd(m);
    }

    int32_t moveMask() const {
#if defined(R_NEON)
        return _neon_movemask(m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(movemask_ps)(m_mask);
#endif
    }

    int32_t ballot() const {
#if defined(R_NEON)
        return _neon_movemask(m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(movemask_ps)(m_mask);
#endif
    }

    bool anyTrue() const {
#if defined(R_NEON)
        return _neon_anytrue(m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return ballot() != 0;
#endif
    }

#if 0
    bool allTrue() const {
#if defined(R_NEON)
        return false;
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return ballot() == kAllTrue;
#endif
    }
#endif

private:
    maskvec_t m_mask;
};


class SoaInt
{
public:
    SoaInt() = default;
    SoaInt(int i) {
#if defined(R_NEON)
        m_i = vdupq_n_s32(i);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        m_i = AVX_INT(set1_epi32)(i);
#endif
    }

    SoaInt(intvec_t i) {
        m_i = i;
    }

    intvec_t getRawValue() const {
        return m_i;
    }

    int32_t getLane(int32_t lane) const {
#if defined(R_NEON)
        return _neon_getLane<intvec_t, int32_t>(m_i, lane);
#else
        union {
            float f;
            int32_t i;
        } temp;
#if defined(R_AVX4L)
        temp.f = _mm_cvtss_f32(_mm_permutevar_ps(_mm_castps_si128(m_i), _mm_set1_epi32(lane)));
#elif defined(R_AVX8L)
        temp.f = _mm256_cvtss_f32(_mm256_permutevar_ps(_mm256_castps_si256(m_i), _mm256_set1_epi32(lane)));
#endif
        return temp.i;
#endif
    }    

private:
    intvec_t m_i;
};

class SoaFloat
{
    friend class SoaVector3f;

public:
    SoaFloat() = default;
    SoaFloat(float f) {
#if defined(R_NEON)
        m_f = vdupq_n_f32(f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        m_f = AVX_INT(set1_ps)(f);
#endif
    }

    SoaFloat(floatvec_t f) : m_f(f) {
    }

    floatvec_t getRawValue() const {
        return m_f;
    }

    float getLane(int32_t lane) const {
#if defined(R_NEON)
        return _neon_getLane<floatvec_t, float>(m_f, lane);
#elif defined(R_AVX4L)
        return _mm_cvtss_f32(_mm_permutevar_ps(m_f, _mm_set1_epi32(lane)));
#elif defined(R_AVX8L)
        return _mm256_cvtss_f32(_mm256_permutevar_ps(m_f, _mm256_set1_epi32(lane)));
#endif
    }

    SoaFloat replaceLane(int32_t lane, float f) const {
        union {
            floatvec_t v;
            float f[SoaConstants::kLaneCount];
        } u;
        u.v = m_f;
        u.f[lane] = f;
        return u.v;
    }

    SoaFloat min(const SoaFloat& f) const {
#if defined(R_NEON)
        return vminq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(min_ps)(m_f, f.m_f);
#endif
    }

    SoaMask lessThan(const SoaFloat& f) {
#if defined(R_NEON)
        return vcltq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_LT_OQ);
#endif
    }

    SoaMask lessThanOrEqual(const SoaFloat& f) {
#if defined(R_NEON)
        return vcleq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return  AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_LE_OQ);
#endif
    }

    SoaMask greaterThan(const SoaFloat& f) {
#if defined(R_NEON)
        return vcgtq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_GT_OQ);
#endif
    }

    SoaMask greaterThanOrEqual(const SoaFloat& f) {
#if defined(R_NEON)
        return vcgeq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_GE_OQ);
#endif
    }

    SoaMask equals(const SoaFloat& f) const {
#if defined(R_NEON)
        return vceqq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_EQ_OQ);
#endif
    }

    SoaMask notEquals(const SoaFloat& f) const {
#if defined(R_NEON)
        return vmvnq_u32(vceqq_f32(m_f, f.m_f));
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_NEQ_OQ);
#endif
    }

    SoaFloat operator*(const SoaFloat& f) const {
#if defined(R_NEON)
        return vmulq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(mul_ps)(m_f, f.m_f);
#endif
    }

    SoaFloat operator+(const SoaFloat& f) const {
#if defined(R_NEON)
        return vaddq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(add_ps)(m_f, f.m_f);
#endif
    }

    SoaFloat operator-(const SoaFloat& f) const {
#if defined(R_NEON)
        return vsubq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(sub_ps)(m_f, f.m_f);
#endif
    }

    SoaFloat operator/(const SoaFloat& f) const {
#if defined(R_NEON)
        return _neon_div(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(div_ps)(m_f, f.m_f);
#endif
    }

    friend SoaFloat operator/(float f, const SoaFloat& v) {
#if defined(R_NEON)
        return _neon_div(vdupq_n_f32(f), v.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(div_ps)(AVX_INT(set1_ps)(f), v.m_f);
#endif
    }

private:
    floatvec_t m_f;
};


class SoaVector3f
{
public:

    SoaVector3f() = default;

    SoaVector3f(const Vector3f& v) {
#if defined(R_NEON)
        m_x = vdupq_n_f32(v.x);
        m_y = vdupq_n_f32(v.y);
        m_z = vdupq_n_f32(v.z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        m_x = AVX_INT(set1_ps)(v.x);
        m_y = AVX_INT(set1_ps)(v.y);
        m_z = AVX_INT(set1_ps)(v.z);
#endif
    }

    SoaVector3f(const SoaFloat& x, const SoaFloat& y, const SoaFloat& z) {
        m_x = x.m_f;
        m_y = y.m_f;
        m_z = z.m_f;
    }

    SoaVector3f(const Vector3f* v) {
#if defined(R_NEON)
        m_x = {v[0].x, v[1].x, v[2].x, v[3].x};
        m_y = {v[0].y, v[1].y, v[2].y, v[3].y};
        m_z = {v[0].z, v[1].z, v[2].z, v[3].z};
#elif defined(R_AVX4L)
        m_x = _mm_set_ps(v[3].x, v[2].x, v[1].x, v[0].x);
        m_y = _mm_set_ps(v[3].y, v[2].y, v[1].y, v[0].y);
        m_z = _mm_set_ps(v[3].z, v[2].z, v[1].z, v[0].z);
#elif defined(R_AVX8L)
        m_x = _mm256_set_ps(v[7].x, v[6].x, v[5].x, v[4].x, v[3].x, v[2].x, v[1].x, v[0].x);
        m_y = _mm256_set_ps(v[7].y, v[6].y, v[5].y, v[4].y, v[3].y, v[2].y, v[1].y, v[0].y);
        m_z = _mm256_set_ps(v[7].z, v[6].z, v[5].z, v[4].z, v[3].z, v[2].z, v[1].z, v[0].z);
#endif
    }

    // Recommended not to touch use getRaw*() as it's touching platform dependent vector variable
    floatvec_t getRawX() const {
        return m_x;
    }

    floatvec_t getRawY() const {
        return m_y;
    }

    floatvec_t getRawZ() const {
        return m_z;
    }

    // TODO: dangerous to use getRefRaw*()
    floatvec_t& getRefRawX() {
        return m_x;
    }

    floatvec_t& getRefRawY() {
        return m_y;
    }

    floatvec_t& getRefRawZ() {
        return m_z;
    }

    SoaFloat getX() const {
        return m_x;
    }

    SoaFloat getY() const {
        return m_y;
    }

    SoaFloat getZ() const {
        return m_z;
    }

    Vector3f getLane(int32_t lane) const {
        Vector3f r;
#if defined(R_NEON)
        r.x = _neon_getLane<floatvec_t, float>(m_x, lane);
        r.y = _neon_getLane<floatvec_t, float>(m_y, lane);
        r.z = _neon_getLane<floatvec_t, float>(m_z, lane);
#else        
        auto index = AVX_INT(set1_epi32)(lane);
        r.x = AVX_INT(cvtss_f32)(AVX_INT(permutevar_ps)(m_x, index));
        r.y = AVX_INT(cvtss_f32)(AVX_INT(permutevar_ps)(m_y, index));
        r.z = AVX_INT(cvtss_f32)(AVX_INT(permutevar_ps)(m_z, index));
#endif
        return r;
    }

    SoaVector3f replaceLane(int32_t lane, const Vector3f& v) const {
        SoaVector3f r;
#if 1
        union {
            floatvec_t v;
            float f[SoaConstants::kLaneCount];
        } u;
        u.v = m_x; u.f[lane] = v.x; r.m_x = u.v;
        u.v = m_y; u.f[lane] = v.y; r.m_y = u.v;
        u.v = m_z; u.f[lane] = v.z; r.m_z = u.v;
#endif
        return r;
    }

    SoaVector3f operator-(const SoaVector3f& v) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vsubq_f32(m_x, v.m_x);
        r.m_y = vsubq_f32(m_y, v.m_y);
        r.m_z = vsubq_f32(m_z, v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(sub_ps)(m_x, v.m_x);
        r.m_y = AVX_INT(sub_ps)(m_y, v.m_y);
        r.m_z = AVX_INT(sub_ps)(m_z, v.m_z);
#endif
        return r;
    }

    SoaVector3f operator+(const SoaVector3f& v) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vaddq_f32(m_x, v.m_x);
        r.m_y = vaddq_f32(m_y, v.m_y);
        r.m_z = vaddq_f32(m_z, v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(add_ps)(m_x, v.m_x);
        r.m_y = AVX_INT(add_ps)(m_y, v.m_y);
        r.m_z = AVX_INT(add_ps)(m_z, v.m_z);
#endif
        return r;
    }

    SoaVector3f operator*(const SoaVector3f& v) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vmulq_f32(m_x, v.m_x);
        r.m_y = vmulq_f32(m_y, v.m_y);
        r.m_z = vmulq_f32(m_z, v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(mul_ps)(m_x, v.m_x);
        r.m_y = AVX_INT(mul_ps)(m_y, v.m_y);
        r.m_z = AVX_INT(mul_ps)(m_z, v.m_z);
#endif
        return r;
    }

    SoaVector3f operator/(const SoaVector3f& v) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = _neon_div(m_x, v.m_x);
        r.m_y = _neon_div(m_y, v.m_y);
        r.m_z = _neon_div(m_z, v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(div_ps)(m_x, v.m_x);
        r.m_y = AVX_INT(div_ps)(m_y, v.m_y);
        r.m_z = AVX_INT(div_ps)(m_z, v.m_z);
#endif
        return r;
    }

    friend SoaVector3f operator/(const SoaFloat& f, const SoaVector3f& v) {
        SoaVector3f r;
#if defined(R_NEON)
        // Clang 11 doesn't accept dirct access to SoaFloat.m_f for some reason
        r.m_x = _neon_div(f.getRawValue(), v.m_x);
        r.m_y = _neon_div(f.getRawValue(), v.m_y);
        r.m_z = _neon_div(f.getRawValue(), v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(div_ps)(f.m_f, v.m_x);
        r.m_y = AVX_INT(div_ps)(f.m_f, v.m_y);
        r.m_z = AVX_INT(div_ps)(f.m_f, v.m_z);
#endif
        return r;
    }

    SoaVector3f operator*(const SoaFloat& f) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vmulq_f32(m_x, f.m_f);
        r.m_y = vmulq_f32(m_y, f.m_f);
        r.m_z = vmulq_f32(m_z, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(mul_ps)(m_x, f.m_f);
        r.m_y = AVX_INT(mul_ps)(m_y, f.m_f);
        r.m_z = AVX_INT(mul_ps)(m_z, f.m_f);
#endif
        return r;
    }

    SoaVector3f operator*(float f) const {
#if defined(R_NEON)
        return (*this)*SoaFloat(f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        SoaVector3f r;
        r.m_x = AVX_INT(mul_ps)(m_x, AVX_INT(set1_ps)(f));
        r.m_y = AVX_INT(mul_ps)(m_y, AVX_INT(set1_ps)(f));
        r.m_z = AVX_INT(mul_ps)(m_z, AVX_INT(set1_ps)(f));
        return r;        
#endif
    }

    SoaVector3f min(const SoaVector3f& v) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vminq_f32(m_x, v.m_x);
        r.m_y = vminq_f32(m_y, v.m_y);
        r.m_z = vminq_f32(m_z, v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(min_ps)(m_x, v.m_x);
        r.m_y = AVX_INT(min_ps)(m_y, v.m_y);
        r.m_z = AVX_INT(min_ps)(m_z, v.m_z);
#endif
        return r;
    }

    SoaVector3f max(const SoaVector3f& v) const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vmaxq_f32(m_x, v.m_x);
        r.m_y = vmaxq_f32(m_y, v.m_y);
        r.m_z = vmaxq_f32(m_z, v.m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        r.m_x = AVX_INT(max_ps)(m_x, v.m_x);
        r.m_y = AVX_INT(max_ps)(m_y, v.m_y);
        r.m_z = AVX_INT(max_ps)(m_z, v.m_z);
#endif
        return r;
    }

    SoaFloat maximumElement() const {
#if defined(R_NEON)
        return vmaxq_f32(m_x, vmaxq_f32(m_y, m_z));
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(max_ps)(m_x, AVX_INT(max_ps)(m_y, m_z));
#endif
    }

    SoaFloat minimumElement() const {
#if defined(R_NEON)
        return vminq_f32(m_x, vminq_f32(m_y, m_z));
#elif defined(R_AVX4L) || defined(R_AVX8L)
        return AVX_INT(min_ps)(m_x, AVX_INT(min_ps)(m_y, m_z));
#endif
    }

/*
    void swapXZ(const SoaFloat& mask) {
#if defined(R_NEON)
#else        
        auto temp = _mm_blendv_ps(m_x, m_z, mask.m_f);
        m_x = _mm_blendv_ps(m_z, m_x, mask.m_f);
        m_z = temp;
#endif
    }
*/

    SoaVector3f abs() const {
        SoaVector3f r;
#if defined(R_NEON)
        r.m_x = vabsq_f32(m_x);
        r.m_y = vabsq_f32(m_y);
        r.m_z = vabsq_f32(m_z);
#elif defined(R_AVX4L) || defined(R_AVX8L)
        static const floatvec_t kSignMask = AVX_INT(set1_ps)(-0.0f);
        r.m_x = AVX_INT(andnot_ps)(kSignMask, m_x);
        r.m_y = AVX_INT(andnot_ps)(kSignMask, m_y);
        r.m_z = AVX_INT(andnot_ps)(kSignMask, m_z);
#endif
        return r;
    }

    friend SoaVector3f operator*(const Vector3f& v, const SoaVector3f& sv) {
        return SoaVector3f(v)*sv;
    }
    friend SoaVector3f operator*(const SoaFloat& f, const SoaVector3f& v) {
        return v*f;
    }

private:
    floatvec_t m_x;
    floatvec_t m_y;
    floatvec_t m_z;
};


struct BBox
{
    static constexpr float kNoHit = -1.0f;

    float intersect(const Vector3f& org, const Vector3f& dir, const Vector3f& invDir) const;
    SoaFloat intersect(const SoaVector3f& org, const SoaVector3f& dir, const SoaVector3f& invDir) const;

    void print() const;

    static BBox init();

//    void init();
    void grow(const BBox& bbox);
    void grow(const Vector3f& p);
    float surfaceArea() const;

    Vector3f lower;
    Vector3f upper;
};


// Functions

inline Vector3f min(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(std::fmin(v0.x, v1.x), std::fmin(v0.y, v1.y), std::fmin(v0.z, v1.z));
}

inline Vector3f max(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(std::fmax(v0.x, v1.x), std::fmax(v0.y, v1.y), std::fmax(v0.z, v1.z));
}

inline Vector3f abs(const Vector3f& v)
{
    return Vector3f(std::abs(v.x), std::abs(v.y), std::abs(v.z));
}


inline float clamp(float f, float x, float y)
{
    if (f < x) return x;
    if (f > y) return y;
    return f;
}


inline float dot(const Vector3f& v0, const Vector3f& v1)
{
    auto v = v0*v1;
    return v.x + v.y + v.z;
//    v = v.xyz + v.yyw;
//    return v.x + v.z;
}

inline Vector3f cross(const Vector3f& v0, const Vector3f& v1)
{
    return Vector3f(v0.y*v1.z - v0.z*v1.y, v0.z*v1.x - v0.x*v1.z, v0.x*v1.y - v0.y*v1.x);
}


inline float length(const Vector3f& v)
{
    return sqrtf(dot(v, v));
}

inline Vector3f normalize(const Vector3f& v)
{
    float invlen = 1.0f/length(v);
    return invlen*v;
}



inline SoaVector3f operator*(const SoaFloat& f, const Vector3f& v) {
    return f*SoaVector3f(v);
}

// TODO add rcpLength
inline SoaFloat length(const SoaVector3f& v) {
    auto x = v.getRawX();
    auto y = v.getRawY();
    auto z = v.getRawZ();
#if defined(R_NEON)
    // vsqrtq_f32
    auto x2 = vmulq_f32(x, x);
    auto y2 = vmulq_f32(y, y);
    auto z2 = vmulq_f32(z, z);
    auto lensq = x2 + y2 + z2;
    return _neon_sqrt(lensq);
//    auto e = vrsqrteq_f32(lensq);
//    e = vmulq_f32(vrsqrtsq_f32(vmulq_f32(lensq, e), e), e);
//    return vmulq_f32(lensq, e);
#elif defined(R_AVX4L) || defined(R_AVX8L)
//    auto x = v.getRawX();
//    auto y = v.getRawY();
//    auto z = v.getRawZ();
    return AVX_INT(sqrt_ps)(AVX_INT(add_ps)(AVX_INT(mul_ps)(x, x), AVX_INT(add_ps)(AVX_INT(mul_ps)(y, y), AVX_INT(mul_ps)(z, z))));
//    return _mm_sqrt_ps(_mm_add_ps(_mm_mul_ps(x, x), _mm_add_ps(_mm_mul_ps(y, y), _mm_mul_ps(z, z))));
#endif
}

inline SoaVector3f normalize(const SoaVector3f& v) {
    auto invlen = 1.0f/length(v);
    return v*invlen;
}

inline SoaFloat select(const SoaFloat& f0, const SoaFloat& f1, const SoaMask& mask)
{
#if defined(R_NEON)
    return vbslq_f32(mask.getRawValue(), f1.getRawValue(), f0.getRawValue());
#elif defined(R_AVX4L) || defined(R_AVX8L)
    return SoaFloat(AVX_INT(blendv_ps)(f0.getRawValue(), f1.getRawValue(), mask.getRawValue()));
//    return SoaFloat(_mm_blendv_ps(f0.getRawValue(), f1.getRawValue(), mask.getRawValue()));
#endif
}

inline SoaInt select(const SoaInt& i0, const SoaInt& i1, const SoaMask& mask)
{
#if defined(R_NEON)
    return vbslq_s32(mask.getRawValue(), i1.getRawValue(), i0.getRawValue());
#elif defined(R_AVX4L)
    return SoaInt(_mm_castps_si128(_mm_blendv_ps(
        _mm_castsi128_ps(i0.getRawValue()), _mm_castsi128_ps(i1.getRawValue()), mask.getRawValue())));
#elif defined(R_AVX8L)
    return SoaInt(_mm256_castps_si256(_mm256_blendv_ps(
        _mm256_castsi256_ps(i0.getRawValue()), _mm256_castsi256_ps(i1.getRawValue()), mask.getRawValue())));
#endif
}


/*
inline SoaVector3f select(const SoaVector3f& v0, const SoaVector3f& v1, const SoaMask& mask)
{
#if defined(R_NEON)
    return SoaVector3f();
#else
    SoaVector3f r;
    r.m_x = _mm_blendv_ps(v0.getRawX(), v1.getRawX(), mask.m_mask);
    r.m_x = _mm_blendv_ps(v0.getRawY(), v1.getRawY(), mask.m_mask);
    r.m_x = _mm_blendv_ps(v0.getRawZ(), v1.getRawZ(), mask.m_mask);
    return r;
#endif
    }
*/

} // namespace prt

