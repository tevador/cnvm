/*
Copyright (c) 2018 tevador (tevador@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "cnvm.h"
#include <cstdlib>
#include <cmath>
#include <utility>
#include <limits>
#include <algorithm>
#include <iostream>
#include <iomanip>

void cnvm::run() {
	//VM initialization
	X = EXECUTION_LENGTH;
	M = 0;
	R.u64 = memory[0].u64;
	P = 0;

	//program execution
	while (X > 0) {
		instruction_t i = program[P & PROGRAM_MASK];
		byte opcode = ocmap[i.opcode & INSTRUCTION_MASK];
		instruction_func f = engine[opcode].getFunc();

#ifdef DEBUG
		std::cout << "M = " << std::hex << M << std::endl;
		std::cout << "R = " << R.u64 << std::endl;
		std::cout << std::dec;
#endif

		(this->*f)(i.payload);

		X--;
		M = memory[M & MEMORY_MASK_ALIGN8].u64;
		R.u64 += M;
		P++;

		if ((i.opcode & ~INSTRUCTION_MASK) != 0)
			M *= X;
	}
}

#define ENGINE_INSTR(x) engine_t(&cnvm::x, #x)

engine_t cnvm::engine[INSTRUCTION_COUNT] = {
	ENGINE_INSTR(ABS_I32),				ENGINE_INSTR(ABS_I64),			ENGINE_INSTR(ABSDIFF_I32),			ENGINE_INSTR(ABSDIFF_I64),
	ENGINE_INSTR(ADD_F32),				ENGINE_INSTR(ADD_F64),			ENGINE_INSTR(ADD_U32),				ENGINE_INSTR(ADD_U64),
	ENGINE_INSTR(ADD2_I32),				ENGINE_INSTR(ADD2_I64),			ENGINE_INSTR(ADDK_F32),				ENGINE_INSTR(ADDK_I32),
	ENGINE_INSTR(ADDMK_I32),			ENGINE_INSTR(ADDR_U32),			ENGINE_INSTR(ADDR_U64),				ENGINE_INSTR(ADDSUB_I32),
	ENGINE_INSTR(ADDSUB_I64),			ENGINE_INSTR(BITCNT0_B32),		ENGINE_INSTR(BITCNT0_B64),			ENGINE_INSTR(BITCNT1_B32),
	ENGINE_INSTR(BITCNT1_B64),			ENGINE_INSTR(BITFLIP_B32),		ENGINE_INSTR(BITFLIP_B64),			ENGINE_INSTR(BITSET0_B32),
	ENGINE_INSTR(BITSET0_B64),			ENGINE_INSTR(BITSET1_B32),		ENGINE_INSTR(BITSET1_B64),			ENGINE_INSTR(CADDMK_I32),
	ENGINE_INSTR(CMOV_I32),				ENGINE_INSTR(CMOV_I64),			ENGINE_INSTR(CMOVK_I32),			ENGINE_INSTR(CSELECT_B32),
	ENGINE_INSTR(CSELECT_B64),			ENGINE_INSTR(CSWAP_B32),		ENGINE_INSTR(CSWAP_B64),			ENGINE_INSTR(CVT_F32_I32),
	ENGINE_INSTR(CVT_F32_U32),			ENGINE_INSTR(CVT_F64_F32),		ENGINE_INSTR(CVT_F64_I32),			ENGINE_INSTR(CVT_F64_U32),
	ENGINE_INSTR(CVT_I32_F32),			ENGINE_INSTR(CYCLE_CCW_B32),	ENGINE_INSTR(CYCLE_CCW_B64),		ENGINE_INSTR(CYCLE_CW_B32),
	ENGINE_INSTR(CYCLE_CW_B64),			ENGINE_INSTR(DEC_U32),			ENGINE_INSTR(DEC_U64),				ENGINE_INSTR(DIV_F64),
	ENGINE_INSTR(DIV_U64_U32),			ENGINE_INSTR(FMA_F32),			ENGINE_INSTR(FMA_F64),				ENGINE_INSTR(FMA_U32),
	ENGINE_INSTR(FMA_U64),				ENGINE_INSTR(FMAR_U32),			ENGINE_INSTR(FMAR_U64),				ENGINE_INSTR(FMX_U32),
	ENGINE_INSTR(FMX_U64),				ENGINE_INSTR(INC_U32),			ENGINE_INSTR(INC_U64),				ENGINE_INSTR(JMP),
	ENGINE_INSTR(MAX_F32),				ENGINE_INSTR(MAX_F64),			ENGINE_INSTR(MAX_I32),				ENGINE_INSTR(MAX_I64),
	ENGINE_INSTR(MAX_U32),				ENGINE_INSTR(MAX_U64),			ENGINE_INSTR(MAX3_U32),				ENGINE_INSTR(MAX3_U64),
	ENGINE_INSTR(MED_U32),				ENGINE_INSTR(MIN_F32),			ENGINE_INSTR(MIN_F64),				ENGINE_INSTR(MIN_I32),
	ENGINE_INSTR(MIN_I64),				ENGINE_INSTR(MIN_U32),			ENGINE_INSTR(MIN_U64),				ENGINE_INSTR(MIN3_U32),
	ENGINE_INSTR(MIN3_U64),				ENGINE_INSTR(MOV_B32),			ENGINE_INSTR(MOV_B64),				ENGINE_INSTR(MOVK_I32),
	ENGINE_INSTR(MOVR_B32),				ENGINE_INSTR(MOVR_B64),			ENGINE_INSTR(MUL_F32),				ENGINE_INSTR(MUL_F64),
	ENGINE_INSTR(MUL_I32),				ENGINE_INSTR(MUL_I64),			ENGINE_INSTR(MULK_F32),				ENGINE_INSTR(MULK_I32),
	ENGINE_INSTR(MULR_I32),				ENGINE_INSTR(MULR_I64),			ENGINE_INSTR(NEG_I32),				ENGINE_INSTR(NEG_I64),
	ENGINE_INSTR(NOP),					ENGINE_INSTR(NOR_OR_B32),		ENGINE_INSTR(NOR_OR_B64),			ENGINE_INSTR(NOT_B32),
	ENGINE_INSTR(NOT_B64),				ENGINE_INSTR(ORN2_ANDN2_B32),	ENGINE_INSTR(ORN2_ANDN2_B64),		ENGINE_INSTR(RCP_F64),
	ENGINE_INSTR(RTL_B32),				ENGINE_INSTR(RTL_B64),			ENGINE_INSTR(RTR_B32),				ENGINE_INSTR(RTR_B64),
	ENGINE_INSTR(SHLXOR_B32),			ENGINE_INSTR(SHLXOR_B64),		ENGINE_INSTR(SHRXOR_B32),			ENGINE_INSTR(SHRXOR_B64),
	ENGINE_INSTR(SQRT_F32),				ENGINE_INSTR(SQRT_F64),			ENGINE_INSTR(SUB_F32),				ENGINE_INSTR(SUB_F64),
	ENGINE_INSTR(SUB_I32),				ENGINE_INSTR(SUB_I64),			ENGINE_INSTR(SUBK_I32),				ENGINE_INSTR(SUBR_I32),
	ENGINE_INSTR(SUBR_I64),				ENGINE_INSTR(SWAP_B32),			ENGINE_INSTR(SWAP_B64),				ENGINE_INSTR(SWAPR_B32),
	ENGINE_INSTR(SWAPR_B64),			ENGINE_INSTR(XNOR_B32),			ENGINE_INSTR(XNOR_B64),				ENGINE_INSTR(XOR_B32),
	ENGINE_INSTR(XOR_B64),				ENGINE_INSTR(XORK_B32),			ENGINE_INSTR(XORR_B32),				ENGINE_INSTR(XORR_B64),
};

const char* cnvm::getInstructionName(int i) {
	return engine[i].getName();
}

#define DST(i) memory[(M & MEMORY_MASK_ALIGN64) + (i)]
#define SC1(i) memory[((M + (ip.addr[0] << 3)) & MEMORY_MASK_ALIGN64) + (i)]
#define SC2(i) memory[((M + (ip.addr[1] << 3)) & MEMORY_MASK_ALIGN64) + (i)]
#define SIMD(f) for(int i = 0; i < 8; ++i) { f; }
#define CONDITIONAL (R.u16 < ip.addr[0]) 

//this solves possible undefined behavior of using the standard abs() function
//https://graphics.stanford.edu/~seander/bithacks.html#IntegerAbs
inline int32_t absu(int32_t x) {
	return (x + (x >> BITMASK_32)) ^ (x >> BITMASK_32);
}
inline int64_t absu(int64_t x) {
	return (x + (x >> BITMASK_64)) ^ (x >> BITMASK_64);
}

//this solves possible undefined behavior
inline int32_t float2intsafe(float x) {
	if (x >= std::numeric_limits<int32_t>::max())
		return std::numeric_limits<int32_t>::max();
	else if (x <= std::numeric_limits<int32_t>::min())
		return std::numeric_limits<int32_t>::min();
	else if (x != x) //NaN
		return 0;
	else
		return (int32_t)x;
}

template<class T> inline
const T& max3(const T& a, const T& b, const T& c) {
	return std::max(std::max(a, b), c);
}

template<class T> inline
const T& min3(const T& a, const T& b, const T& c) {
	return std::min(std::min(a, b), c);
}

template<class T> inline
const T& median(const T& a, const T& b, const T& c) {
	return  std::max(std::min(a, b), std::min(std::max(a, b), c));
}

//2s complement
template<typename T> inline
const T neg(const T& x) {
	return ~x + 1;
}

template<typename T> inline void cycle_cw(T& t1, T& t2, T& t3) {
	T temp = std::move(t1);
	t1 = std::move(t3);
	t3 = std::move(t2);
	t2 = std::move(temp);
}

template<typename T> inline void cycle_ccw(T& t1, T& t2, T& t3) {
	T temp = std::move(t1);
	t1 = std::move(t2);
	t2 = std::move(t3);
	t3 = std::move(temp);
}

template<typename T> inline
const T rotl(const T& x, T c) {
	auto bitCount = sizeof(T) * 8;
	c = c & (bitCount - 1);
	return (x << c) | (x >> (bitCount - c));
}

template<typename T> inline
const T rotr(const T& x, T c) {
	auto bitCount = sizeof(T) * 8;
	c = c & (bitCount - 1);
	return (x >> c) | (x << (bitCount - c));
}

//rounds off the last N binary digits
template<typename T, unsigned N> inline
void round_binary(T& x) {
	x += 1L << (N - 1);
	x &= ~((1L << N) - 1);
}

inline void sqrtu32(memory_t& x) {
	x.f32 = sqrt((float)x.u32);
	round_binary<uint32_t, 3>(x.u32);
}

inline void sqrtu64(memory_t& x) {
	x.f64 = sqrt((double)x.u64);
	round_binary<uint64_t, 3>(x.u64);
}

//----------------------------------------------------------------------------------------

void cnvm::ABS_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 = absu(DST(i).i32))
}

void cnvm::ABS_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 = absu(DST(i).i64))
}

void cnvm::ABSDIFF_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 = absu(SC1(i).i32 - SC2(i).i32))
}

void cnvm::ABSDIFF_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 = absu(SC1(i).i64 - SC2(i).i64))
}

void cnvm::ADD_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)DST(i).i32 + (float)SC1(i).i32)
}

void cnvm::ADD_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)DST(i).i64 + (double)SC1(i).i64)
}


void cnvm::ADD_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 += SC1(i).u32)
}

void cnvm::ADD_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 += SC1(i).u64)
}

void cnvm::ADD2_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 += SC1(i).i32 + SC2(i).i32)
}

void cnvm::ADD2_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 += SC1(i).i64 + SC2(i).i64)
}

void cnvm::ADDK_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)DST(i).i32 + (float)ip.i32)
}

void cnvm::ADDK_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 += ip.i32)
}

void cnvm::ADDMK_I32(instruction_payload_t ip) {
	M += (int64_t)ip.i32;
}

void cnvm::ADDR_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 += R.u32)
}

void cnvm::ADDR_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 += R.u64)
}

void cnvm::ADDSUB_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 += SC1(i).i32 - SC2(i).i32)
}

void cnvm::ADDSUB_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 += SC1(i).i64 - SC2(i).i64)
}

void cnvm::BITCNT0_B32(instruction_payload_t ip) {
	SIMD(DST(i).u8 = 32 - (byte)__popcnt(SC1(i).u32))
}

void cnvm::BITCNT0_B64(instruction_payload_t ip) {
	SIMD(DST(i).u8 = 64 - (byte)__popcnt64(SC1(i).u64))
}

void cnvm::BITCNT1_B32(instruction_payload_t ip) {
	SIMD(DST(i).u8 = (byte)__popcnt(SC1(i).u32))
}

void cnvm::BITCNT1_B64(instruction_payload_t ip) {
	SIMD(DST(i).u8 = (byte)__popcnt64(SC1(i).u64))
}

void cnvm::BITFLIP_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 ^= 1U << (SC1(i).u8 & BITMASK_32))
}

void cnvm::BITFLIP_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 ^= 1UL << (SC1(i).u8 & BITMASK_64))
}

void cnvm::BITSET0_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 &= ~(1U << (SC1(i).u8 & BITMASK_32)))
}

void cnvm::BITSET0_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 &= ~(1UL << (SC1(i).u8 & BITMASK_64)))
}

void cnvm::BITSET1_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 |= (1U << (SC1(i).u8 & BITMASK_32)))
}

void cnvm::BITSET1_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 |= (1UL << (SC1(i).u8 & BITMASK_64)))
}

void cnvm::CADDMK_I32(instruction_payload_t ip) {
	if(CONDITIONAL) {
		M += (int64_t)ip.i32;
	}
}

void cnvm::CMOV_I32(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(DST(i).i32 = SC1(i).i32)
	}
}

void cnvm::CMOV_I64(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(DST(i).i64 = SC1(i).i64)
	}
}

void cnvm::CMOVK_I32(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(DST(i).i32 = ip.i32)
	}
}

void cnvm::CSELECT_B32(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(DST(i).u32 = SC1(i).u32)
	}
	else {
		SIMD(DST(i).u32 = SC2(i).u32)
	}
}

void cnvm::CSELECT_B64(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(DST(i).u64 = SC1(i).u64)
	}
	else {
		SIMD(DST(i).u64 = SC2(i).u64)
	}
}

void cnvm::CSWAP_B32(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(std::swap(DST(i).u32, SC1(i).u32))
	}
}

void cnvm::CSWAP_B64(instruction_payload_t ip) {
	if(CONDITIONAL) {
		SIMD(std::swap(DST(i).u64, SC1(i).u64))
	}
}

void cnvm::CVT_F32_I32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)SC1(i).i32)
}

void cnvm::CVT_F32_U32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)SC1(i).u32)
}

void cnvm::CVT_F64_F32(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)((float)SC1(i).i32))
}

void cnvm::CVT_F64_I32(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)SC1(i).i32)
}

void cnvm::CVT_F64_U32(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)SC1(i).u32)
}

void cnvm::CVT_I32_F32(instruction_payload_t ip) {
	SIMD(DST(i).i32 = float2intsafe(SC1(i).f32))
}


void cnvm::CYCLE_CCW_B32(instruction_payload_t ip) {
	SIMD(cycle_ccw(DST(i).u32, SC1(i).u32, SC2(i).u32))
}

void cnvm::CYCLE_CCW_B64(instruction_payload_t ip) {
	SIMD(cycle_ccw(DST(i).u64, SC1(i).u64, SC2(i).u64))
}

void cnvm::CYCLE_CW_B32(instruction_payload_t ip) {
	SIMD(cycle_cw(DST(i).u32, SC1(i).u32, SC2(i).u32))
}

void cnvm::CYCLE_CW_B64(instruction_payload_t ip) {
	SIMD(cycle_cw(DST(i).u64, SC1(i).u64, SC2(i).u64))
}

void cnvm::DEC_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32--)
}

void cnvm::DEC_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64--)
}

void cnvm::DIV_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)DST(i).i64 / (double)SC1(i).i64)
}

void cnvm::DIV_U64_U32(instruction_payload_t ip) {
	SIMD(DST(i).u64 = DST(i).u64 / (SC1(i).u32 | 5))
}

void cnvm::FMA_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)DST(i).i32 * (float)SC1(i).i32 + (float)SC2(i).i32)
}

void cnvm::FMA_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)DST(i).i64 * (double)SC1(i).i64 + (double)SC2(i).i64)
}

void cnvm::FMA_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = DST(i).u32 * SC1(i).u32 + SC2(i).u32)
}

void cnvm::FMA_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = DST(i).u64 * SC1(i).u64 + SC2(i).u64)
}

void cnvm::FMAR_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = DST(i).u32 * SC1(i).u32 + R.u32)
}

void cnvm::FMAR_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = DST(i).u64 * SC1(i).u64 + R.u64)
}

void cnvm::FMX_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = DST(i).u32 * SC1(i).u32 ^ SC2(i).u32)
}

void cnvm::FMX_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = DST(i).u64 * SC1(i).u64 ^ SC2(i).u64)
}

void cnvm::INC_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32++)
}

void cnvm::INC_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64++)
}

void cnvm::JMP(instruction_payload_t ip) {
	if(CONDITIONAL){
#ifdef DEBUG
		auto P1 = P;
#endif
#ifdef GPU_FRIENDLY
		P++; //skip the next instruction
#else
		P += (ip.addr[1] & JMP_MASK);
#endif
#ifdef DEBUG
		std::cout << "JMP (" << std::hex << std::setw(4) << ip.addr[0] << std::dec << ") " << (P1 & PROGRAM_MASK) << " -> " << (P & PROGRAM_MASK) << std::endl;
#endif
	}
#ifdef DEBUG
	else {
		std::cout << "JMP (" << std::hex << std::setw(4) << ip.addr[0] << std::dec << ") not taken" << std::endl;
	}
#endif
}


void cnvm::MAX_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = std::max((float)SC1(i).i32, (float)SC2(i).i32))
}

void cnvm::MAX_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = std::max((double)SC1(i).i64, (double)SC2(i).i64))
}

void cnvm::MAX_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 = std::max(SC1(i).i32, SC2(i).i32))
}

void cnvm::MAX_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 = std::max(SC1(i).i64, SC2(i).i64))
}

void cnvm::MAX_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = std::max(SC1(i).u32, SC2(i).u32))
}

void cnvm::MAX_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = std::max(SC1(i).u64, SC2(i).u64))
}

void cnvm::MAX3_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = max3(DST(i).u32, SC1(i).u32, SC2(i).u32))
}

void cnvm::MAX3_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = max3(DST(i).u64, SC1(i).u64, SC2(i).u64))
}

void cnvm::MED_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = median(DST(i).u32, SC1(i).u32, SC2(i).u32))
}

void cnvm::MIN_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = std::min((float)SC1(i).i32, (float)SC2(i).i32))
}

void cnvm::MIN_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = std::min((double)SC1(i).i64, (double)SC2(i).i64))
}

void cnvm::MIN_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 = std::min(SC1(i).i32, SC2(i).i32))
}

void cnvm::MIN_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 = std::min(SC1(i).i64, SC2(i).i64))
}

void cnvm::MIN_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = std::min(SC1(i).u32, SC2(i).u32))
}

void cnvm::MIN_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = std::min(SC1(i).u64, SC2(i).u64))
}


void cnvm::MIN3_U32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = min3(DST(i).u32, SC1(i).u32, SC2(i).u32))
}

void cnvm::MIN3_U64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = min3(DST(i).u64, SC1(i).u64, SC2(i).u64))
}

void cnvm::MOV_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = SC1(i).u32)
}

void cnvm::MOV_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = SC1(i).u64)
}

void cnvm::MOVK_I32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = ip.u32)
}

void cnvm::MOVR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = R.u32)
}

void cnvm::MOVR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = R.u64)
}

void cnvm::MUL_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)DST(i).i32 * (float)SC1(i).i32)
}

void cnvm::MUL_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)DST(i).i64 * (double)SC1(i).i64)
}

void cnvm::MUL_I32(instruction_payload_t ip) {
	SIMD(DST(i).u32 *= SC1(i).u32)
}

void cnvm::MUL_I64(instruction_payload_t ip) {
	SIMD(DST(i).u64 *= SC1(i).u64)
}

void cnvm::MULK_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)DST(i).i32 * (float)ip.i32)
}

void cnvm::MULK_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 *= ip.i32)
}

void cnvm::MULR_I32(instruction_payload_t ip) {
	SIMD(DST(i).u32 *= R.u32)
}

void cnvm::MULR_I64(instruction_payload_t ip) {
	SIMD(DST(i).u64 *= R.u64)
}

void cnvm::NEG_I32(instruction_payload_t ip) {
	SIMD(DST(i).i32 = neg(DST(i).i32))
}

void cnvm::NEG_I64(instruction_payload_t ip) {
	SIMD(DST(i).i64 = neg(DST(i).i64))
}

void cnvm::NOP(instruction_payload_t ip) {
}

void cnvm::NOR_OR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = ~(DST(i).u32 | SC1(i).u32) | SC2(i).u32)
}

void cnvm::NOR_OR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = ~(DST(i).u64 | SC1(i).u64) | SC2(i).u64)
}

void cnvm::NOT_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = ~(DST(i).u32))
}

void cnvm::NOT_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = ~(DST(i).u64))
}

void cnvm::ORN2_ANDN2_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = (DST(i).u32 | ~SC1(i).u32) & ~SC2(i).u32)
}

void cnvm::ORN2_ANDN2_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = (DST(i).u64 | ~SC1(i).u64) & ~SC2(i).u64)
}

void cnvm::RCP_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = 1.0 / DST(i).i64)
}

void cnvm::RTL_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = rotl(DST(i).u32,  SC1(i).u32))
}

void cnvm::RTL_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = rotl(DST(i).u32, SC1(i).u32))
}

void cnvm::RTR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = rotr(DST(i).u32, SC1(i).u32))
}

void cnvm::RTR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = rotr(DST(i).u32, SC1(i).u32))
}

void cnvm::SHLXOR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = (DST(i).u32 << (SC1(i).u32 & BITMASK_32)) ^ SC2(i).u32)
}

void cnvm::SHLXOR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = (DST(i).u64 << (SC1(i).u64 & BITMASK_64)) ^ SC2(i).u64)
}

void cnvm::SHRXOR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = (DST(i).u32 >> (SC1(i).u32 & BITMASK_32)) ^ SC2(i).u32)
}

void cnvm::SHRXOR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = (DST(i).u64 >> (SC1(i).u64 & BITMASK_64)) ^ SC2(i).u64)
}

void cnvm::SQRT_F32(instruction_payload_t ip) {
	SIMD(sqrtu32(DST(i)))
}

void cnvm::SQRT_F64(instruction_payload_t ip) {
	SIMD(sqrtu64(DST(i)))
}

void cnvm::SUB_F32(instruction_payload_t ip) {
	SIMD(DST(i).f32 = (float)DST(i).i32 - (float)SC1(i).i32)
}

void cnvm::SUB_F64(instruction_payload_t ip) {
	SIMD(DST(i).f64 = (double)DST(i).i64 - (double)SC1(i).i64)
}

void cnvm::SUB_I32(instruction_payload_t ip) {
	SIMD(DST(i).u32 -= SC1(i).u32)
}

void cnvm::SUB_I64(instruction_payload_t ip) {
	SIMD(DST(i).u64 -= SC1(i).u64)
}

void cnvm::SUBK_I32(instruction_payload_t ip) {
	SIMD(DST(i).u32 -= ip.u32)
}

void cnvm::SUBR_I32(instruction_payload_t ip) {
	SIMD(DST(i).u32 -= R.u32)
}

void cnvm::SUBR_I64(instruction_payload_t ip) {
	SIMD(DST(i).u64 -= R.u64)
}

void cnvm::SWAP_B32(instruction_payload_t ip) {
	SIMD(std::swap(DST(i).u32, SC1(i).u32))
}

void cnvm::SWAP_B64(instruction_payload_t ip) {
	SIMD(std::swap(DST(i).u64, SC1(i).u64))
}

void cnvm::SWAPR_B32(instruction_payload_t ip) {
	SIMD(std::swap(DST(i).u32, R.u32))
}

void cnvm::SWAPR_B64(instruction_payload_t ip) {
	SIMD(std::swap(DST(i).u64, R.u64))
}

void cnvm::XNOR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 = ~(DST(i).u32 ^ SC1(i).u32))
}

void cnvm::XNOR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 = ~(DST(i).u64 ^ SC1(i).u64))
}

void cnvm::XOR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 ^= SC1(i).u32)
}

void cnvm::XOR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 ^= SC1(i).u64)
}

void cnvm::XORK_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 ^= ip.u32)
}

void cnvm::XORR_B32(instruction_payload_t ip) {
	SIMD(DST(i).u32 ^= R.u32)
}

void cnvm::XORR_B64(instruction_payload_t ip) {
	SIMD(DST(i).u64 ^= R.u64)
}