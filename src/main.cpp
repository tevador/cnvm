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
#include <ctime>
#include <fstream>
#include <iostream>
#include <cstring>

void rnd_from_file(char* buffer, uint64_t count) {
	std::ifstream fin("/dev/urandom", std::ios::in | std::ios::binary);
	fin.read(buffer, count);
	fin.close();
}

void dump(char* buffer, uint64_t count, const char* name) {
	std::ofstream fout(name, std::ios::out | std::ios::binary);
	fout.write(buffer, count);
	fout.close();
}

uint64_t diff(memory_t* a, memory_t* b, uint64_t length) {
	uint64_t count = 0;
	for (uint64_t i = 0; i < length; ++i) {
		count += __popcnt64(a[i].u64 ^ b[i].u64);
	}
	return count;
}

int unbiased_random(int min, int max)
{
	int n = max - min + 1;
	int remainder = RAND_MAX % n;
	int x;
	do
	{
		x = rand();
	} while (x >= RAND_MAX - remainder);
	return min + x % n;
}

int main(int argc, char* argv[])
{
	clock_t startClock, endClock;

	cnvm vm;
	memory_t* initial_mem = new memory_t[MEMORY_LENGTH];

	std::cout << "Loading random memory from /dev/urandom ..." << std::endl;
	rnd_from_file(reinterpret_cast<char*>(vm.memory), MEMORY_SIZE);
	std::memcpy(initial_mem, vm.memory, MEMORY_SIZE);

	std::cout << "Loading random program from /dev/urandom ..." << std::endl;
	rnd_from_file(reinterpret_cast<char*>(vm.program), PROGRAM_SIZE);

	if(argc > 1 && argv[1][0] != '-') { //fill the whole program with one instruction type, e.g. ./cnvm XOR_B64
		for (byte j = 0; j < INSTRUCTION_COUNT; ++j) {
			if (strcmp(argv[1], cnvm::getInstructionName(j)) == 0) {
				std::cout << "Instruction = " << cnvm::getInstructionName(j) << std::endl;
				for (byte i = 0; i < INSTRUCTION_COUNT; ++i) {
					vm.ocmap[i] = j;
				}
				break;
			}
		}
	}
	else { //generate random permutation
		srand(time(NULL));
		for (byte i = 0; i < INSTRUCTION_COUNT; ++i) {
			vm.ocmap[i] = i;
		}
		for (byte i = INSTRUCTION_COUNT - 1; i >= 1; --i) {
			int j = unbiased_random(0, i);
			std::swap(vm.ocmap[j], vm.ocmap[i]);
		}
		std::cout << "Generating random permutation of instructions ... " << std::endl << std::endl;
		for (byte i = 0; i < INSTRUCTION_COUNT; ++i) {
			std::cout << cnvm::getInstructionName(vm.ocmap[i]) << " ";
		}
		std::cout << std::endl << std::endl;
	}

	std::cout << "Executing VM ..." << std::endl << std::endl;

	startClock = clock();
	vm.run();
	endClock = clock();

	double msec = (endClock - startClock) / (double)CLOCKS_PER_SEC * 1000;
	std::cout << "VM halted in " << msec << " ms" << std::endl;

	auto changed = diff(initial_mem, vm.memory, MEMORY_LENGTH);
	auto total_bits = (MEMORY_SIZE * 8);
	auto changed_pct = changed / (double)total_bits;
	std::cout << changed << "/" << total_bits << " bits changed (" << 100 * changed_pct << "%)" << std::endl << std::endl;

	if (argc > 2 && strcmp(argv[2], "-dump") == 0) {
		std::cout << "Dumping memory ..." << std::endl;
		dump(reinterpret_cast<char*>(initial_mem), MEMORY_SIZE, "memory_before");
		dump(reinterpret_cast<char*>(vm.memory), MEMORY_SIZE, "memory_after");
	}

	return 0;
}