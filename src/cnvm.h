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

#pragma once

#include <cstdint>

#ifdef __GNUC__
#define __popcnt(x) __builtin_popcount(x)
#define __popcnt64(x) __builtin_popcountl(x)
#elif _MSC_VER
#include <intrin.h>
#endif

typedef uint8_t byte;

typedef union {
	uint16_t addr[2];
	uint32_t u32;
	int32_t i32;
	float f32;
} instruction_payload_t;

typedef struct {
	byte opcode;
	instruction_payload_t payload;
} instruction_t;

typedef union {
	uint64_t u64;
	int64_t i64;
	double f64;
	uint32_t u32;
	int32_t i32;
	float f32;
	uint16_t u16;
	byte u8;
} memory_t;

class cnvm;

typedef void(cnvm::*instruction_func)(instruction_payload_t);

class engine_t {
private:
	instruction_func func;
	const char* name;
public:
	engine_t(instruction_func f, const char* n) {
		func = f;
		name = n;
	}
	const char* getName() { return name; }
	instruction_func getFunc() { return func; }
};

const uint64_t MEMORY_SIZE = 4 * 1024 * 1024;
const uint64_t MEMORY_LENGTH = MEMORY_SIZE / 8;
const uint64_t PROGRAM_LENGTH = 512;
const uint64_t PROGRAM_SIZE = PROGRAM_LENGTH * sizeof(instruction_t);
const uint64_t INSTRUCTION_COUNT = 128;
const uint64_t EXECUTION_LENGTH = 65536;

const uint64_t MEMORY_MASK_ALIGN8  = MEMORY_LENGTH - 1;
const uint64_t MEMORY_MASK_ALIGN64 = MEMORY_LENGTH - 8;
const uint64_t PROGRAM_MASK = PROGRAM_LENGTH - 1;
const uint64_t INSTRUCTION_MASK = INSTRUCTION_COUNT - 1;
const int16_t JMP_MASK = INSTRUCTION_MASK >> 1;
const uint32_t BITMASK_32 = 32 - 1;
const uint64_t BITMASK_64 = 64 - 1;

class cnvm {
private:
	static engine_t engine[INSTRUCTION_COUNT];
	
	//registers
	memory_t R;
	uint64_t X, M, P;

	//128 instructions
	void ABS_I32(instruction_payload_t);
	void ABS_I64(instruction_payload_t);
	void ABSDIFF_I32(instruction_payload_t);
	void ABSDIFF_I64(instruction_payload_t);
	void ADD_F32(instruction_payload_t);
	void ADD_F64(instruction_payload_t);
	void ADD_U32(instruction_payload_t);
	void ADD_U64(instruction_payload_t);
	void ADD2_I32(instruction_payload_t);
	void ADD2_I64(instruction_payload_t);
	void ADDK_F32(instruction_payload_t);
	void ADDK_I32(instruction_payload_t);
	void ADDMK_I32(instruction_payload_t);
	void ADDR_U32(instruction_payload_t);
	void ADDR_U64(instruction_payload_t);
	void ADDSUB_I32(instruction_payload_t);
	void ADDSUB_I64(instruction_payload_t);
	void BITCNT0_B32(instruction_payload_t);
	void BITCNT0_B64(instruction_payload_t);
	void BITCNT1_B32(instruction_payload_t);
	void BITCNT1_B64(instruction_payload_t);
	void BITFLIP_B32(instruction_payload_t);
	void BITFLIP_B64(instruction_payload_t);
	void BITSET0_B32(instruction_payload_t);
	void BITSET0_B64(instruction_payload_t);
	void BITSET1_B32(instruction_payload_t);
	void BITSET1_B64(instruction_payload_t);
	void CADDMK_I32(instruction_payload_t);
	void CMOV_I32(instruction_payload_t);
	void CMOV_I64(instruction_payload_t);
	void CMOVK_I32(instruction_payload_t);
	void CSELECT_B32(instruction_payload_t);
	void CSELECT_B64(instruction_payload_t);
	void CSWAP_B32(instruction_payload_t);
	void CSWAP_B64(instruction_payload_t);
	void CVT_F32_I32(instruction_payload_t);
	void CVT_F32_U32(instruction_payload_t);
	void CVT_F64_F32(instruction_payload_t);
	void CVT_F64_I32(instruction_payload_t);
	void CVT_F64_U32(instruction_payload_t);
	void CVT_I32_F32(instruction_payload_t);
	void CYCLE_CCW_B32(instruction_payload_t);
	void CYCLE_CCW_B64(instruction_payload_t);
	void CYCLE_CW_B32(instruction_payload_t);
	void CYCLE_CW_B64(instruction_payload_t);
	void DEC_U32(instruction_payload_t);
	void DEC_U64(instruction_payload_t);
	void DIV_F64(instruction_payload_t);
	void DIV_U64_U32(instruction_payload_t);
	void FMA_F32(instruction_payload_t);
	void FMA_F64(instruction_payload_t);
	void FMA_U32(instruction_payload_t);
	void FMA_U64(instruction_payload_t);
	void FMAR_U32(instruction_payload_t);
	void FMAR_U64(instruction_payload_t);
	void FMX_U32(instruction_payload_t);
	void FMX_U64(instruction_payload_t);
	void INC_U32(instruction_payload_t);
	void INC_U64(instruction_payload_t);
	void JMP(instruction_payload_t);
	void MAX_F32(instruction_payload_t);
	void MAX_F64(instruction_payload_t);
	void MAX_I32(instruction_payload_t);
	void MAX_I64(instruction_payload_t);
	void MAX_U32(instruction_payload_t);
	void MAX_U64(instruction_payload_t);
	void MAX3_U32(instruction_payload_t);
	void MAX3_U64(instruction_payload_t);
	void MED_U32(instruction_payload_t);
	void MIN_F32(instruction_payload_t);
	void MIN_F64(instruction_payload_t);
	void MIN_I32(instruction_payload_t);
	void MIN_I64(instruction_payload_t);
	void MIN_U32(instruction_payload_t);
	void MIN_U64(instruction_payload_t);
	void MIN3_U32(instruction_payload_t);
	void MIN3_U64(instruction_payload_t);
	void MOV_B32(instruction_payload_t);
	void MOV_B64(instruction_payload_t);
	void MOVK_I32(instruction_payload_t);
	void MOVR_B32(instruction_payload_t);
	void MOVR_B64(instruction_payload_t);
	void MUL_F32(instruction_payload_t);
	void MUL_F64(instruction_payload_t);
	void MUL_I32(instruction_payload_t);
	void MUL_I64(instruction_payload_t);
	void MULK_F32(instruction_payload_t);
	void MULK_I32(instruction_payload_t);
	void MULR_I32(instruction_payload_t);
	void MULR_I64(instruction_payload_t);
	void NEG_I32(instruction_payload_t);
	void NEG_I64(instruction_payload_t);
	void NOP(instruction_payload_t);
	void NOR_OR_B32(instruction_payload_t);
	void NOR_OR_B64(instruction_payload_t);
	void NOT_B32(instruction_payload_t);
	void NOT_B64(instruction_payload_t);
	void ORN2_ANDN2_B32(instruction_payload_t);
	void ORN2_ANDN2_B64(instruction_payload_t);
	void RCP_F64(instruction_payload_t);
	void RTL_B32(instruction_payload_t);
	void RTL_B64(instruction_payload_t);
	void RTR_B32(instruction_payload_t);
	void RTR_B64(instruction_payload_t);
	void SHLXOR_B32(instruction_payload_t);
	void SHLXOR_B64(instruction_payload_t);
	void SHRXOR_B32(instruction_payload_t);
	void SHRXOR_B64(instruction_payload_t);
	void SQRT_F32(instruction_payload_t);
	void SQRT_F64(instruction_payload_t);
	void SUB_F32(instruction_payload_t);
	void SUB_F64(instruction_payload_t);
	void SUB_I32(instruction_payload_t);
	void SUB_I64(instruction_payload_t);
	void SUBK_I32(instruction_payload_t);
	void SUBR_I32(instruction_payload_t);
	void SUBR_I64(instruction_payload_t);
	void SWAP_B32(instruction_payload_t);
	void SWAP_B64(instruction_payload_t);
	void SWAPR_B32(instruction_payload_t);
	void SWAPR_B64(instruction_payload_t);
	void XNOR_B32(instruction_payload_t);
	void XNOR_B64(instruction_payload_t);
	void XOR_B32(instruction_payload_t);
	void XOR_B64(instruction_payload_t);
	void XORK_B32(instruction_payload_t);
	void XORR_B32(instruction_payload_t);
	void XORR_B64(instruction_payload_t);

public:
	static const char* getInstructionName(int i);
	memory_t memory[MEMORY_LENGTH];
	instruction_t program[PROGRAM_LENGTH];
	byte ocmap[INSTRUCTION_COUNT];
	void run();
};