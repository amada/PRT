#pragma once

#include <stdint.h>
#include <math.h>
#include <cmath>
#include <limits>

#include "platform.h"

#if defined(R_NEON)
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

	using floatvec4_t = float32x4_t;
#elif defined(R_AVX4L)
	using floatvec_t = __m128;
	using intvec_t = __m128i;
	using maskvec_t = __m128;

	using floatvec4_t = __m128;
#elif defined(R_AVX8L)
	using floatvec_t = __m256;
	using intvec_t = __m256i;
	using maskvec_t = __m256;

	using floatvec4_t = __m128;
#endif

	class SoaConstants final {
	public:
#ifdef R_AVX8L
		static const int32_t kLaneCount = 8;
#else
		static const int32_t kLaneCount = 4;
#endif
		static const int32_t kFullLaneMask = (1 << kLaneCount) - 1;
	};

	class VectorConstants final {
	public:
		static const uint32_t kDimensions = 3;
	};

	// clang
	inline int32_t bitScanForward(int32_t bits) {
#ifdef R_MSVC
		unsigned long ret;
		_BitScanForward(&ret, bits);
		return (int)ret;
#else
		return __builtin_ctz(bits);
#endif
	}

#ifdef R_NEON
    inline uint32x4_t _neon_load_uint32x4(uint32_t x, uint32_t y, uint32_t z, uint32_t w)
    {
#ifdef R_MSVC
		return { .n128_u32{ x, y, z, w} };
#else
		return {x, y, z, w};
#endif
    }
#endif

#ifdef R_NEON
    inline float32x4_t _neon_load_float32x4(float x, float y, float z, float w)
    {
#ifdef R_MSVC
		return { .n128_f32{ x, y, z, w } };
#else
		return {x, y, z, w};
#endif
    }
#endif

#ifdef R_NEON
	// Arm Neon
	inline floatvec_t _neon_div(floatvec_t n, floatvec_t d) {
#ifdef R_ARM64
		return vdivq_f32(n, d);
#else // R_ARM64
		// aarch64 vdivq_f32
		auto r = vrecpeq_f32(d);
		r = vmulq_f32(vrecpsq_f32(d, r), r);
		r = vmulq_f32(vrecpsq_f32(d, r), r);
		return vmulq_f32(n, r);
#endif // __arch64__
	}

	inline floatvec_t _neon_sqrt(floatvec_t f) {
#ifdef R_ARM64
		return vsqrtq_f32(f);
#else // R_ARM64
		// aarch64 vsqrtq_f32
		auto e = vrsqrteq_f32(f); // Estimation of 1/sqrt(f)
		// vrsqrtsq_f32(d, xn) = (3 - d*xn)/2
		// e*(3-f*e*e)/2
		e = vmulq_f32(vrsqrtsq_f32(vmulq_f32(f, e), e), e);
		return vmulq_f32(f, e); // f/sqrt(f)
#endif // R_ARM64
	}

	inline int32_t _neon_movemask(maskvec_t m) {
		const uint32x4_t mask = _neon_load_uint32x4(1, 2, 4, 8);
		return vaddvq_u32(vandq_u32(m, mask));
	}

	inline bool _neon_anytrue(maskvec_t m) {
		//    uint32x2_t temp = vorr_u32(vget_low_u32(m), vget_high_u32(m));
		//    return vget_lane_u32(vpmax_u32(temp, temp), 0);

		return vmaxvq_u32(m) != 0; // Should be safe as long as mask is used via vecmath functions
	}

	inline bool _neon_alltrue(maskvec_t m) {
		return vminvq_u32(m) != 0;
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

#if defined(R_AVX4L) || defined(R_AVX8L)
	template<typename V, typename T>
	inline T _avx_getLane(V v, int32_t lane) {
		union {
			T i[SoaConstants::kLaneCount];
			V v;
		} u;
		u.v = v;
		return u.i[lane];
	}
#endif

	static const float kPi = 3.14159265358979323846f;


	class Vector2f
	{
	public:
		Vector2f() = default;

		Vector2f(float f) : x(f), y(f) {}

		Vector2f(float _x, float _y) : x(_x), y(_y) {}

		Vector2f operator+(const Vector2f& v) const {
			return Vector2f(x + v.x, y + v.y);
		}

		Vector2f operator-(const Vector2f& v) const {
			return Vector2f(x - v.x, y - v.y);
		}

		Vector2f operator*(const Vector2f& v) const {
			return Vector2f(x * v.x, y * v.y);
		}

		friend Vector2f operator*(float f, const Vector2f& v) {
			return Vector2f(f * v.x, f * v.y);
		}

		float x;
		float y;
	};

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
		Vector3f(const float* v, uint32_t size) {
			x = v[0];
			y = v[1];
			z = v[2];
		}

		uint32_t GetLongestElement() const {
			if (x > y) {
				if (x > z) {
					return 0;
				}
				else {
					return 2;
				}
			}
			else {
				if (y > z) {
					return 1;
				}
				else {
					return 2;
				}
			}
		}

		bool isNonZero() const {
			return x != 0 || y != 0 || z != 0;
		}

		void print(const char* tag = nullptr) const;

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
			return Vector3f(x * v.x, y * v.y, z * v.z);
		}

		Vector3f operator/(const Vector3f& v) const {
			return Vector3f(x / v.x, y / v.y, z / v.z);
		}

		Vector3f operator*(float f) const {
			return Vector3f(f * x, f * y, f * z);
		}

		friend Vector3f operator*(float f, const Vector3f& v) {
			return Vector3f(f * v.x, f * v.y, f * v.z);
		}

		friend Vector3f operator/(float f, const Vector3f& v) {
			return Vector3f(f / v.x, f / v.y, f / v.z);
		}
	};


	class Vector4f
	{
	public:
		Vector4f() = default;
		Vector4f(float _x, float _y, float _z, float _w) : x(_x), y(_y), z(_z), w(_w) {}
		Vector4f(float f) : x(f), y(f), z(f), w(f) {}

		Vector3f getXYZ() const { return Vector3f(x, y, z); }

		Vector4f operator+(const Vector4f& v) const {
			return Vector4f(x + v.x, y + v.y, z + v.z, w + v.w);
		}

		friend Vector4f operator*(float f, const Vector4f& v) {
			return Vector4f(f * v.x, f * v.y, f * v.z, f * v.w);
		}

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

		SoaMask(int32_t mask) {
#define SET_MASK(i) ((i) ? 0xffffffffu : 0u)
#if defined(R_NEON)
			m_mask = vdupq_n_u32(SET_MASK(mask & 0x1));
			m_mask = vsetq_lane_u32(SET_MASK(mask & 0x2), m_mask, 1);
			m_mask = vsetq_lane_u32(SET_MASK(mask & 0x4), m_mask, 2);
			m_mask = vsetq_lane_u32(SET_MASK(mask & 0x8), m_mask, 3);
#elif defined(R_AVX4L)
			m_mask = _mm_set_epi32(SET_MASK(mask & 0x8), SET_MASK(mask & 0x4), SET_MASK(mask & 0x2), SET_MASK(mask & 0x1));
#elif defined(R_AVX8L)
			m_mask = _mm256_set_epi32(SET_MASK(mask & 0x80), SET_MASK(mask & 0x40), SET_MASK(mask & 0x20), SET_MASK(mask & 0x10),
				SET_MASK(mask & 0x8), SET_MASK(mask & 0x4), SET_MASK(mask & 0x2), SET_MASK(mask & 0x1));
#endif
		}

		void setAll(bool b) {
#define SET_MASK(i) ((i) ? 0xffffffffu : 0u)
#if defined(R_NEON)
			m_mask = vdupq_n_u32(SET_MASK(b));
#elif defined(R_AVX4L)
			m_mask = _mm_castsi128_ps(_mm_set1_epi32(SET_MASK(b)));
#elif defined(R_AVX8L)
			m_mask = _mm256_castsi256_ps(_mm256_set1_epi32(SET_MASK(b)));
#endif
		}

		maskvec_t getRawValue() const {
			return m_mask;
		}

		int32_t getLane(int32_t lane) const {
#if defined(R_NEON)
			return _neon_getLane<intvec_t, int32_t>(m_mask, lane);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return _avx_getLane<maskvec_t, int32_t>(m_mask, lane);
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

		SoaMask operator||(const SoaMask& m) const {
			return computeOr(m);
		}

		SoaMask operator&&(const SoaMask& m) const {
			return computeAnd(m);
		}

		SoaMask operator!() const {
			return computeNot();
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

		bool allTrue() const {
#if defined(R_NEON)
			return _neon_alltrue(m_mask);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return ballot() == kAllTrue;
#endif
		}

	private:
		maskvec_t m_mask;
	};


	class SoaFloat;

	class SoaInt
	{
		friend class SoaFloat;
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

		SoaInt(const SoaFloat& f);

		intvec_t getRawValue() const {
			return m_i;
		}

		int32_t getLane(int32_t lane) const {
#if defined(R_NEON)
			return _neon_getLane<intvec_t, int32_t>(m_i, lane);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return _avx_getLane<intvec_t, int32_t>(m_i, lane);
#endif
		}

		SoaInt operator+(const SoaInt& i) const {
#if defined(R_NEON)
			return vaddq_s32(m_i, i.m_i);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return AVX_INT(add_epi32)(m_i, i.m_i);
#endif
		}

		SoaInt operator*(const SoaInt& i) const {
#if defined(R_NEON)
			return vmulq_s32(m_i, i.m_i);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return AVX_INT(mullo_epi32)(m_i, i.m_i);
#endif
		}

	private:
		intvec_t m_i;
	};

	class SoaFloat
	{
		friend class SoaInt;
		friend class SoaVector2f;
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

		SoaFloat(const SoaInt& i) {
#if defined(R_NEON)
			m_f = vcvtq_f32_s32(i.m_i);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			m_f = AVX_INT(cvtepi32_ps)(i.m_i);
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
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return _avx_getLane<floatvec_t, float>(m_f, lane);
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

		SoaFloat floor() const {
#if defined(R_NEON)
			return vrndmq_f32(m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return AVX_INT(floor_ps)(m_f);
#endif
		}

		SoaMask lessThan(const SoaFloat& f) const {
#if defined(R_NEON)
			return vcltq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_LT_OQ);
#endif
		}

		SoaMask lessThanOrEqual(const SoaFloat& f) const {
#if defined(R_NEON)
			return vcleq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return  AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_LE_OQ);
#endif
		}

		SoaMask greaterThan(const SoaFloat& f) const {
#if defined(R_NEON)
			return vcgtq_f32(m_f, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			return AVX_INT(cmp_ps)(m_f, f.m_f, _CMP_GT_OQ);
#endif
		}

		SoaMask greaterThanOrEqual(const SoaFloat& f) const {
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

		SoaFloat operator*(float f) const {
			return (*this) * SoaFloat(f);
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

		SoaMask operator<(const SoaFloat& f) const {
			return lessThan(f);
		}

		SoaMask operator<=(const SoaFloat& f) const {
			return lessThanOrEqual(f);
		}

		SoaMask operator>(const SoaFloat& f) const {
			return greaterThan(f);
		}

		SoaMask operator>=(const SoaFloat& f) const {
			return greaterThanOrEqual(f);
		}

		SoaMask operator==(const SoaFloat& f) const {
			return equals(f);
		}

		SoaMask operator!=(const SoaFloat& f) const {
			return notEquals(f);
		}

	private:
		floatvec_t m_f;
	};

	class SoaVector2f
	{
	public:
		SoaVector2f() = default;

		SoaVector2f(const Vector2f& v) {
#if defined(R_NEON)
			m_x = vdupq_n_f32(v.x);
			m_y = vdupq_n_f32(v.y);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			m_x = AVX_INT(set1_ps)(v.x);
			m_y = AVX_INT(set1_ps)(v.y);
#endif
		}

		SoaVector2f(const Vector2f* v) {
#if defined(R_NEON)
			m_x = _neon_load_float32x4(v[0].x, v[1].x, v[2].x, v[3].x);
			m_y = _neon_load_float32x4(v[0].y, v[1].y, v[2].y, v[3].y);
#elif defined(R_AVX4L)
			m_x = _mm_set_ps(v[3].x, v[2].x, v[1].x, v[0].x);
			m_y = _mm_set_ps(v[3].y, v[2].y, v[1].y, v[0].y);
#elif defined(R_AVX8L)
			m_x = _mm256_set_ps(v[7].x, v[6].x, v[5].x, v[4].x, v[3].x, v[2].x, v[1].x, v[0].x);
			m_y = _mm256_set_ps(v[7].y, v[6].y, v[5].y, v[4].y, v[3].y, v[2].y, v[1].y, v[0].y);
#endif
		}

		SoaVector2f(const SoaFloat& x, const SoaFloat& y, const SoaFloat& z) {
			m_x = x.m_f;
			m_y = y.m_f;
		}

		SoaFloat getX() const {
			return m_x;
		}

		SoaFloat getY() const {
			return m_y;
		}

		Vector2f getLane(int32_t lane) const {
			Vector2f r;
#if defined(R_NEON)
			r.x = _neon_getLane<floatvec_t, float>(m_x, lane);
			r.y = _neon_getLane<floatvec_t, float>(m_y, lane);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			r.x = _avx_getLane<floatvec_t, float>(m_x, lane);
			r.y = _avx_getLane<floatvec_t, float>(m_y, lane);
#endif
			return r;
		}

		SoaVector2f broadcast(int32_t lane) const {
			return SoaVector2f(getLane(lane));
		}

		SoaVector2f operator*(const SoaFloat& f) const {
			SoaVector2f r;
#if defined(R_NEON)
			r.m_x = vmulq_f32(m_x, f.m_f);
			r.m_y = vmulq_f32(m_y, f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			r.m_x = AVX_INT(mul_ps)(m_x, f.m_f);
			r.m_y = AVX_INT(mul_ps)(m_y, f.m_f);
#endif
			return r;
		}

		SoaVector2f operator+(const SoaVector2f& v) const {
			SoaVector2f r;
#if defined(R_NEON)
			r.m_x = vaddq_f32(m_x, v.m_x);
			r.m_y = vaddq_f32(m_y, v.m_y);
#elif defined(R_AVX4L) || defined(R_AVX8L)
			r.m_x = AVX_INT(add_ps)(m_x, v.m_x);
			r.m_y = AVX_INT(add_ps)(m_y, v.m_y);
#endif
			return r;
		}

	private:
		floatvec_t m_x;
		floatvec_t m_y;
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
			m_x = _neon_load_float32x4(v[0].x, v[1].x, v[2].x, v[3].x);
			m_y = _neon_load_float32x4(v[0].y, v[1].y, v[2].y, v[3].y);
			m_z = _neon_load_float32x4(v[0].z, v[1].z, v[2].z, v[3].z);
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

		void print(const char* tag = nullptr) const;

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
			r.x = _avx_getLane<floatvec_t, float>(m_x, lane);
			r.y = _avx_getLane<floatvec_t, float>(m_y, lane);
			r.z = _avx_getLane<floatvec_t, float>(m_z, lane);
#endif
			return r;
		}

		SoaVector3f replaceLane(int32_t lane, const Vector3f& v) const {
			SoaVector3f r;

			union {
				floatvec_t v;
				float f[SoaConstants::kLaneCount];
			} u;
			u.v = m_x; u.f[lane] = v.x; r.m_x = u.v;
			u.v = m_y; u.f[lane] = v.y; r.m_y = u.v;
			u.v = m_z; u.f[lane] = v.z; r.m_z = u.v;

			return r;
		}

		SoaVector3f broadcast(int32_t lane) const {
			return SoaVector3f(getLane(lane));
		}

		SoaVector3f swapXZ(const SoaMask& swap) const {
#if defined(R_NEON)
			auto temp = vbslq_f32(swap.m_mask, m_x, m_z);
			auto x = vbslq_f32(swap.m_mask, m_z, m_x);
			auto z = temp;
#else
			auto temp = AVX_INT(blendv_ps)(m_z, m_x, swap.m_mask);
			auto x = AVX_INT(blendv_ps)(m_x, m_z, swap.m_mask);
			auto z = temp;
#endif
			return SoaVector3f(x, m_y, z);
		}

		SoaVector3f swapYZ(const SoaMask& swap) const {
#if defined(R_NEON)
			auto temp = vbslq_f32(swap.m_mask, m_y, m_z);
			auto y = vbslq_f32(swap.m_mask, m_z, m_y);
			auto z = temp;
#else
			auto temp = AVX_INT(blendv_ps)(m_z, m_y, swap.m_mask);
			auto y = AVX_INT(blendv_ps)(m_y, m_z, swap.m_mask);
			auto z = temp;
#endif
			return SoaVector3f(m_x, y, z);
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
			return (*this) * SoaFloat(f);
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
			return SoaVector3f(v) * sv;
		}
		friend SoaVector3f operator*(const SoaFloat& f, const SoaVector3f& v) {
			return v * f;
		}

	private:
		floatvec_t m_x;
		floatvec_t m_y;
		floatvec_t m_z;
	};


	struct BBox
	{
		static constexpr float kNoHit = std::numeric_limits<float>::infinity();

		inline float intersect(const Vector3f& org, const Vector3f& dir, const Vector3f& invDir) const;
		inline bool intersect(const Vector3f& org, const Vector3f& dir, const Vector3f& invDir, float maxT) const;

		inline SoaFloat intersect(const SoaVector3f& org, const SoaVector3f& dir, const SoaVector3f& invDir) const;
		inline SoaMask intersect(const SoaVector3f& org, const SoaVector3f& dir, const SoaVector3f& invDir, const SoaFloat& maxT) const;

		bool contains(const Vector3f& p) const;

		void print() const;

		static BBox init();

		//    void init();
		void merge(const BBox& bbox);
		void merge(const Vector3f& p);
		float surfaceArea() const;

		Vector3f center() const { return 0.5f * (upper + lower); }

		Vector3f lower;
		Vector3f upper;
	};


	// Functions

	inline SoaInt::SoaInt(const SoaFloat& f)
	{
#if defined(R_NEON)
		m_i = vcvtq_s32_f32(f.m_f);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		m_i = AVX_INT(cvtps_epi32)(f.m_f);
#endif
	}


	inline float dot(const Vector2f& v0, const Vector2f& v1)
	{
		auto v = v0 * v1;
		return v.x + v.y;
	}

	inline float length(const Vector2f& v)
	{
		return sqrtf(dot(v, v));
	}

	inline Vector2f normalize(const Vector2f& v)
	{
		float invlen = 1.0f / length(v);
		return invlen * v;
	}

	inline Vector2f safeNormalize(const Vector2f& v)
	{
		float len = length(v);
		if (len < 0.00001f) {
			return Vector2f(0.0f, 0.0f);
		}
		float invlen = 1.0f / len;
		return invlen * v;
	}


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
		return std::fmin(std::fmax(f, x), y);
	}

	inline Vector3f clamp(const Vector3f& v, float l, float u)
	{
		return Vector3f(clamp(v.x, l, u), clamp(v.y, l, u), clamp(v.z, l, u));
	}

	inline float dot(const Vector3f& v0, const Vector3f& v1)
	{
		auto v = v0 * v1;
		return v.x + v.y + v.z;
		//    v = v.xyz + v.yyw;
		//    return v.x + v.z;
	}

	inline Vector3f cross(const Vector3f& v0, const Vector3f& v1)
	{
		return Vector3f(v0.y * v1.z - v0.z * v1.y, v0.z * v1.x - v0.x * v1.z, v0.x * v1.y - v0.y * v1.x);
	}


	inline float length(const Vector3f& v)
	{
		return sqrtf(dot(v, v));
	}

	inline Vector3f normalize(const Vector3f& v)
	{
		float invlen = 1.0f / length(v);
		return invlen * v;
	}

	inline bool isnan(const Vector3f& v)
	{
		return std::isnan(v.x) || std::isnan(v.y) || std::isnan(v.z);
	}

	inline SoaInt operator+(int32_t i0, const SoaInt& i1) {
		return SoaInt(i0) + i1;
	}

	inline SoaInt operator*(int32_t i0, const SoaInt& i1) {
		return SoaInt(i0) * i1;
	}

	inline SoaInt min(const SoaInt& a, const SoaInt& b) {
#if defined(R_NEON)
		return vminq_s32(a.getRawValue(), b.getRawValue());
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return AVX_INT(min_epi32)(a.getRawValue(), b.getRawValue());
#endif
	}

	inline SoaInt max(const SoaInt& a, const SoaInt& b) {
#if defined(R_NEON)
		return vmaxq_s32(a.getRawValue(), b.getRawValue());
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return AVX_INT(max_epi32)(a.getRawValue(), b.getRawValue());
#endif
	}


	inline SoaFloat operator-(float f0, const SoaFloat& f1) {
		return SoaFloat(f0) - f1;
	}

	inline SoaVector2f operator*(const SoaFloat& f, const Vector2f& v) {
		return SoaVector2f(v) * f;
	}

	inline SoaFloat min(const SoaFloat& a, const SoaFloat& b) {
#if defined(R_NEON)
		return vminq_f32(a.getRawValue(), b.getRawValue());
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return AVX_INT(min_ps)(a.getRawValue(), b.getRawValue());
#endif
	}

	inline SoaFloat max(const SoaFloat& a, const SoaFloat& b) {
#if defined(R_NEON)
		return vmaxq_f32(a.getRawValue(), b.getRawValue());
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return AVX_INT(max_ps)(a.getRawValue(), b.getRawValue());
#endif
	}

	inline SoaFloat floor(const SoaFloat& f) {
#if defined(R_NEON)
		return vrndmq_f32(f.getRawValue());
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return AVX_INT(floor_ps)(f.getRawValue());
#endif
	}


	inline SoaVector3f operator*(const SoaFloat& f, const Vector3f& v) {
		return f * SoaVector3f(v);
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
		auto lensq = vaddq_f32(vaddq_f32(x2, y2), z2);
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
		auto invlen = 1.0f / length(v);
		return v * invlen;
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

	inline float reduceMax4(floatvec4_t a)
	{
#if defined(R_NEON)
		return vmaxvq_f32(a);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		auto temp = _mm_max_ps(a, _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 3, 2)));
		a = _mm_max_ps(temp, _mm_shuffle_ps(temp, temp, _MM_SHUFFLE(0, 0, 0, 1)));
		return _mm_cvtss_f32(a);
#endif
	}

	inline float reduceMin4(floatvec4_t a)
	{
#if defined(R_NEON)
		return vminvq_f32(a);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		auto temp = _mm_min_ps(a, _mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 3, 2)));
		a = _mm_min_ps(temp, _mm_shuffle_ps(temp, temp, _MM_SHUFFLE(0, 0, 0, 1)));
		return _mm_cvtss_f32(a);
#endif
	}

	inline floatvec4_t max(floatvec4_t a, floatvec4_t b)
	{
#if defined(R_NEON)
		return vmaxq_f32(a, b);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return _mm_max_ps(a, b);
#endif
	}

	inline floatvec4_t min(floatvec4_t a, floatvec4_t b)
	{
#if defined(R_NEON)
		return vminq_f32(a, b);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return _mm_min_ps(a, b);
#endif
	}

	inline floatvec4_t add(floatvec4_t a, floatvec4_t b)
	{
#if defined(R_NEON)
		return vaddq_f32(a, b);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return _mm_add_ps(a, b);
#endif
	}

	inline floatvec4_t sub(floatvec4_t a, floatvec4_t b)
	{
#if defined(R_NEON)
		return vsubq_f32(a, b);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return _mm_sub_ps(a, b);
#endif
	}

	inline floatvec4_t mul(floatvec4_t a, floatvec4_t b)
	{
#if defined(R_NEON)
		return vmulq_f32(a, b);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return _mm_mul_ps(a, b);
#endif
	}

	inline floatvec4_t floatvec4(float x, float y, float z, float w)
	{
#if defined(R_NEON)
		return _neon_load_float32x4(x, y, z, w);
#elif defined(R_AVX4L) || defined(R_AVX8L)
		return _mm_set_ps(w, z, y, x);
#endif
	}


	float BBox::intersect(const Vector3f& org, const Vector3f& dir, const Vector3f& invDir) const
	{
#if 1
		auto vlower = floatvec4(lower.x, lower.y, lower.z, -1.0f);
		auto vupper = floatvec4(upper.x, upper.y, upper.z, 1.0f);
		auto vorg = floatvec4(org.x, org.y, org.z, 0.0f);
		auto vinvDir = floatvec4(invDir.x, invDir.y, invDir.z, std::numeric_limits<float>::infinity());
		//   auto vinvDir = floatvec4(invDir.x, invDir.y, invDir.z, 1.0f / 0.0f);

		auto tmp_t0 = mul(sub(vlower, vorg), vinvDir);
		auto tmp_t1 = mul(sub(vupper, vorg), vinvDir);

		auto t0 = min(tmp_t0, tmp_t1);
		auto t1 = max(tmp_t0, tmp_t1);

		auto max_t0 = reduceMax4(t0);
		auto min_t1 = reduceMin4(t1);

		if (min_t1 < max_t0) {
			return kNoHit;
		}

		return max_t0;
#else
		auto tmp_t0 = (lower - org) * invDir;
		auto tmp_t1 = (upper - org) * invDir;

		auto t0 = min(tmp_t0, tmp_t1);
		auto t1 = max(tmp_t0, tmp_t1);

		auto max_t0 = std::max(t0.x, std::max(t0.y, t0.z));
		auto min_t1 = std::min(t1.x, std::min(t1.y, t1.z));

		if (min_t1 < max_t0) {
			return kNoHit;
		}

		// inside bbox
		if (max_t0 < 0.0f) {
			return 0.0f;
		}

		return max_t0;
#endif
	}


	bool BBox::intersect(const Vector3f& org, const Vector3f& dir, const Vector3f& invDir, float maxT) const
	{
#if 1
		auto vlower = floatvec4(lower.x, lower.y, lower.z, std::numeric_limits<float>::lowest());
		auto vupper = floatvec4(upper.x, upper.y, upper.z, maxT);
		auto vorg = floatvec4(org.x, org.y, org.z, 0.0f);
		auto vinvDir = floatvec4(invDir.x, invDir.y, invDir.z, 1.0f);

		auto tmp_t0 = mul(sub(vlower, vorg), vinvDir);
		auto tmp_t1 = mul(sub(vupper, vorg), vinvDir);

		auto t0 = min(tmp_t0, tmp_t1);
		auto t1 = max(tmp_t0, tmp_t1);

		auto max_t0 = reduceMax4(t0);
		auto min_t1 = reduceMin4(t1);

		return min_t1 > max_t0;
#else
		auto tmp_t0 = (lower - org) * invDir;
		auto tmp_t1 = (upper - org) * invDir;

		auto t0 = min(tmp_t0, tmp_t1);
		auto t1 = max(tmp_t0, tmp_t1);

		auto max_t0 = std::max(t0.x, std::max(t0.y, t0.z));
		auto min_t1 = std::min(t1.x, std::min(t1.y, t1.z));

		return (min_t1 > max_t0 && max_t0 < maxT);
#endif
	}


	SoaFloat BBox::intersect(const SoaVector3f& org, const SoaVector3f& dir, const SoaVector3f& invDir) const
	{
		auto tmp_t0 = (SoaVector3f(lower) - org) * invDir;
		auto tmp_t1 = (SoaVector3f(upper) - org) * invDir;

		auto t0 = tmp_t0.min(tmp_t1);
		auto t1 = tmp_t0.max(tmp_t1);

		auto max_t0 = t0.maximumElement();
		auto min_t1 = t1.minimumElement();

		auto mask = min_t1 >= max_t0;

		SoaFloat r(max_t0);
		r = select(kNoHit, r, mask);

		// inside box
		auto temp = select(max_t0, 0.0f, max_t0 < 0.0f);

		return select(r, temp, mask);
	}

	SoaMask BBox::intersect(const SoaVector3f& org, const SoaVector3f& dir, const SoaVector3f& invDir, const SoaFloat& maxT) const
	{
		auto tmp_t0 = (SoaVector3f(lower) - org) * invDir;
		auto tmp_t1 = (SoaVector3f(upper) - org) * invDir;

		auto t0 = tmp_t0.min(tmp_t1);
		auto t1 = tmp_t0.max(tmp_t1);

		auto max_t0 = t0.maximumElement();
		auto min_t1 = t1.minimumElement();

		auto mask = min_t1 >= max_t0;

		return max_t0 < maxT&& mask;
	}


} // namespace prt

