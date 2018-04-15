# CNVM
CNVM is a proof-of-work concept based on the [CryptoNight hash function](https://cryptonote.org/cns/cns008.txt), but replaces the static memory-hard loop with a simulation of a virtual machine. It is designed specifically for commodity hardware like CPUs and GPUs by taking advantage of their programmability and combines memory-hardness, by requiring 4 MiB of low latency memory per worker thread, with a randomized algorithm which changes every block.

When a new block is to be solved, a random program generated from the hash of the previous block is loaded into a virtual machine and the following steps are performed:

1. The block header (template + chosen nonce) is expanded into a 4 MiB scratchpad using the "Scratchpad Initialization" algorithm from the CryptoNight hash function.
1. The initialized scratchpad is loaded into the virtual machine. When the machine is started, it randomly modifies the scratchpad and halts.
1. The modified scratchpad is collapsed into a 256-bit final value by applying the "Result Calculation" part of the CryptoNight hash function.
1. The final value is compared to the difficulty target as usual.

## The Virtual Machine

![VM Schema](https://i.imgur.com/AsFlb0p.png)

#### Registers
All registers are 64 bits wide.

* __P__ - The program counter (only the last 9 bits are used).
* __X__ - Holds the number of instructions to be executed. When X reaches 0, the machine will halt. 
* __M__ - Holds the base memory address for the currently executed instruction. All memory addressed specified by instructions are offsets from this base address. M is also the address in memory where the result of the current instruction is written (unless the output is a register). Only the last 19 bits of the register are used (8-byte aligned access).
* __R__ - This register is used as the input or output of some instructions.

#### Program ring buffer

The VM reads instructions from a 2560-byte ring buffer, which means that after the last instruction is processed, execution jumps automatically back to the first instruction. This is repeated until the value of the X register reaches 0. All instructions are 40 bits long, consisting of an 8-bit opcode followed by a 32-bit payload. The interpretation of the payload depends on the instruction. For example, it can be decoded into two 16-bit addresses for operands or into one 32-bit inline constant.

#### Opcode map

The opcode map is a random permutation of numbers from 0 to 127 and is used to decode the instruction operation. When an instruction is loaded from the ring buffer, the MSB of the first byte is cleared and the result is used as an index to lookup the operation associated with this instruction. The permutation changes every block, therefore no instruction has a static opcode.

### Instruction set

The machine understands 128 instructions which are listed below. Each program consists of 512 instructions, so a total of 2^3584 distinct programs are possible (not taking into account difference in instruction parameters).

The instruction set was designed so that any random bitstring is a valid program. The instruction set mixes 32-bit, 64-bit and even some 8-bit operations. Instructions were chosen to preserve the entropy of the scrachpad as much as possible, while using the widest possible range of primitive operations. All but 4 instructions are SIMD - they perform the same operation on 8 quadwords, modifying up to 64 bytes of memory (1 cache line of a modern CPU).

On average, one instruction performs 2 random 64-byte reads/writes into the memory.

#### Floating point

All basic floating point operations must give bit-exact results according to the the IEEE 754 standard. In the case of the SQRT instruction, which has platform-specific accuracy, the last 3 bits of the mantissa are rounded off to prevent errors. With the exception of the CVT_I32_F32 instruction (float to int32 conversion), floating point values are never read directly from memory, but the operands are read as signed integers and then converted to floating point. This is done to avoid NaN and infinite values.

#### Branching

There are several conditional instructions. All use a static branching probability to enable branch prediction. The branching probability is hardcoded into the instruction payload and ranges from 0/65536 (0%) to 65535/65536 (99.998%).

One special case is the JMP instruction, which has 2 possible modes:

1. Full branch (equivalent to a `goto` statement to a random position in the program).
1. "GPU friendly" branch, which simply skips the next instruction.

The first mode is not supported by all GPUs and can have significant impact on GPU mining performance. The reference implementation uses the second mode.

--------|LIST OF|INSTRUCTIONS|--------
-|-|-|-
ABS_I32|ABS_I64|ABSDIFF_I32|ABSDIFF_I64
ADD_F32|ADD_F64|ADD_U32|ADD_U64
ADD2_I32|ADD2_I64|ADDK_F32|ADDK_I32
ADDMK_I32|ADDR_U32|ADDR_U64|ADDSUB_I32
ADDSUB_I64|BITCNT0_B32|BITCNT0_B64|BITCNT1_B32
BITCNT1_B64|BITFLIP_B32|BITFLIP_B64|BITSET0_B32
BITSET0_B64|BITSET1_B32|BITSET1_B64|CADDMK_I32
CMOV_I32|CMOV_I64|CMOVK_I32|CSELECT_B32
CSELECT_B64|CSWAP_B32|CSWAP_B64|CVT_F32_I32
CVT_F32_U32|CVT_F64_F32|CVT_F64_I32|CVT_F64_U32
CVT_I32_F32|CYCLE_CCW_B32|CYCLE_CCW_B64|CYCLE_CW_B32
CYCLE_CW_B64|DEC_U32|DEC_U64|DIV_F64
DIV_U64_U32|FMA_F32|FMA_F64|FMA_U32
FMA_U64|FMAR_U32|FMAR_U64|FMX_U32
FMX_U64|INC_U32|INC_U64|JMP
MAX_F32|MAX_F64|MAX_I32|MAX_I64
MAX_U32|MAX_U64|MAX3_U32|MAX3_U64
MED_U32|MIN_F32|MIN_F64|MIN_I32
MIN_I64|MIN_U32|MIN_U64|MIN3_U32
MIN3_U64|MOV_B32|MOV_B64|MOVK_I32
MOVR_B32|MOVR_B64|MUL_F32|MUL_F64
MUL_I32|MUL_I64|MULK_F32|MULK_I32
MULR_I32|MULR_I64|NEG_I32|NEG_I64
NOP|NOR_OR_B32|NOR_OR_B64|NOT_B32
NOT_B64|ORN2_ANDN2_B32|ORN2_ANDN2_B64|RCP_F64
RTL_B32|RTL_B64|RTR_B32|RTR_B64
SHLXOR_B32|SHLXOR_B64|SHRXOR_B32|SHRXOR_B64
SQRT_F32|SQRT_F64|SUB_F32|SUB_F64
SUB_I32|SUB_I64|SUBK_I32|SUBR_I32
SUBR_I64|SWAP_B32|SWAP_B64|SWAPR_B32
SWAPR_B64|XNOR_B32|XNOR_B64|XOR_B32
XOR_B64|XORK_B32|XORR_B32|XORR_B64

### Proof of concept code

The PoC code implements just the virtual machine and loads random 4 MiB scratchpad and a random program from /dev/urandom. It is meant as a reference implementation. High performance mining software should include a compiler directly into native code instead of an interpreter.

The reference implementation initializes the machine with X = 65536 (number of instructions executed), which is a good compromise between speed and portion of the scratchpad that is modified by the program.
On a 3.7 GHz CPU with 4+ MiB of L3 cache, an average program takes about 2.5 milliseconds to run and modifies about 23-24% of the 4 MiB scratchpad (measured by the number of bits flipped).
