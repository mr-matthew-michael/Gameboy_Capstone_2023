#include "main.h"
#include "mmu.h"
#include "main.h"
#include <iostream>
#include <cstdint>

uint8_t boot_rom[256] = {
	0x31, 0xFE, 0xFF, 0xAF, 0x21, 0xFF, 0x9F, 0x32, 0xCB, 0x7C, 0x20, 0xFB,
	0x21, 0x26, 0xFF, 0x0E, 0x11, 0x3E, 0x80, 0x32, 0xE2, 0x0C, 0x3E, 0xF3,
	0xE2, 0x32, 0x3E, 0x77, 0x77, 0x3E, 0xFC, 0xE0, 0x47, 0x11, 0x04, 0x01,
	0x21, 0x10, 0x80, 0x1A, 0xCD, 0x95, 0x00, 0xCD, 0x96, 0x00, 0x13, 0x7B,
	0xFE, 0x34, 0x20, 0xF3, 0x11, 0xD8, 0x00, 0x06, 0x08, 0x1A, 0x13, 0x22,
	0x23, 0x05, 0x20, 0xF9, 0x3E, 0x19, 0xEA, 0x10, 0x99, 0x21, 0x2F, 0x99,
	0x0E, 0x0C, 0x3D, 0x28, 0x08, 0x32, 0x0D, 0x20, 0xF9, 0x2E, 0x0F, 0x18,
	0xF3, 0x67, 0x3E, 0x64, 0x57, 0xE0, 0x42, 0x3E, 0x91, 0xE0, 0x40, 0x04,
	0x1E, 0x02, 0x0E, 0x0C, 0xF0, 0x44, 0xFE, 0x90, 0x20, 0xFA, 0x0D, 0x20,
	0xF7, 0x1D, 0x20, 0xF2, 0x0E, 0x13, 0x24, 0x7C, 0x1E, 0x83, 0xFE, 0x62,
	0x28, 0x06, 0x1E, 0xC1, 0xFE, 0x64, 0x20, 0x06, 0x7B, 0xE2, 0x0C, 0x3E,
	0x87, 0xE2, 0xF0, 0x42, 0x90, 0xE0, 0x42, 0x15, 0x20, 0xD2, 0x05, 0x20,
	0x4F, 0x16, 0x20, 0x18, 0xCB, 0x4F, 0x06, 0x04, 0xC5, 0xCB, 0x11, 0x17,
	0xC1, 0xCB, 0x11, 0x17, 0x05, 0x20, 0xF5, 0x22, 0x23, 0x22, 0x23, 0xC9,
	0xCE, 0xED, 0x66, 0x66, 0xCC, 0x0D, 0x00, 0x0B, 0x03, 0x73, 0x00, 0x83,
	0x00, 0x0C, 0x00, 0x0D, 0x00, 0x08, 0x11, 0x1F, 0x88, 0x89, 0x00, 0x0E,
	0xDC, 0xCC, 0x6E, 0xE6, 0xDD, 0xDD, 0xD9, 0x99, 0xBB, 0xBB, 0x67, 0x63,
	0x6E, 0x0E, 0xEC, 0xCC, 0xDD, 0xDC, 0x99, 0x9F, 0xBB, 0xB9, 0x33, 0x3E,
	0x3C, 0x42, 0xB9, 0xA5, 0xB9, 0xA5, 0x42, 0x3C, 0x21, 0x04, 0x01, 0x11,
	0xA8, 0x00, 0x1A, 0x13, 0xBE, 0x20, 0xFE, 0x23, 0x7D, 0xFE, 0x34, 0x20,
	0xF5, 0x06, 0x19, 0x78, 0x86, 0x23, 0x05, 0x20, 0xFB, 0x86, 0x20, 0xFE,
	0x3E, 0x01, 0xE0, 0x50
};

uint8_t state_of_ppu = 2;				//	drawing mode the PPU currently is in


void create_mmu(mmu_t* mmu)
{
	memset(mmu->memory, 0, sizeof(mmu->memory));
	memset(mmu->rom, 0, sizeof(mmu->rom));
}

void init_mmu(mmu_t* mmu) {
	load_boot_screen(mmu);
}

void load_boot_screen(mmu_t* mmu) {

	for (int i = 0; i < sizeof(boot_rom); i++) {
		mmu->memory[i] = boot_rom[i];
	}
}

void lock_boot_screen(mmu_t* mmu) {
	for (int i = 0; i < sizeof(boot_rom); i++) {
		mmu->memory[i] = mmu->rom[i];
	}
}

//	copy sprites to oam
void dma_transfer(mmu_t* mmu) {
	const uint16_t src = mmu_read_byte(mmu, 0xFF46) * 0x100;
	const uint16_t dst = 0xFE00;
	const size_t count = 40 * 4;

	memcpy(&mmu->memory[dst], &mmu->memory[src], count);
}


//	copy cartridge to memory
void load_game(mmu_t* mmu, uint8_t c[]) {
	for (int i = 0; i < 0x8000; i++) {
		mmu->memory[i] = c[i];
	}
	for (int i = 0; i < 0xFA000; i++) {
		mmu->rom[i] = c[i];
	}

	//	overwrite 256 bytes with bootROM
	load_boot_screen(mmu);
}

//	set the mode, the PPU is currently in (important for OAM / VRAM access / denying access on reads / writes)
void set_ppu_state(uint8_t state) {
	state_of_ppu = state;
}

uint8_t mmu_write_byte(mmu_t* mmu, uint16_t adr, uint8_t val) {

	//	joypad input
	if (adr == 0xff00) {
		mmu->memory[adr] = controller_input(val);
		return 0;
	}

	//	lock bootrom
	else if (adr == 0xff50 && val == 1) {
		lock_boot_screen(mmu);
		mmu->memory[adr] = val;
	}

	//	oam dma transer
	else if (adr == 0xff46) {
		dma_transfer(mmu);
		mmu->memory[adr] = val;
	}

	//	readonly
	if (adr >= 0x8000)
		mmu->memory[adr] = val;

	return 0;
}

uint8_t mmu_write_word(mmu_t* mmu, uint16_t addr, uint16_t data)
{
	uint8_t r = mmu_write_byte(mmu, addr, data & 0xFF) & mmu_write_byte(mmu, addr + 1, data >> 8);
	return r;
}

uint16_t mmu_read_word(mmu_t* mmu, uint16_t addr)
{
	return (mmu_read_byte(mmu, addr) | (mmu_read_byte(mmu, addr + 1) << 8));
}

uint8_t& mmu_read_byte(mmu_t* mmu, uint16_t adr) {
	return mmu->memory[adr];
}

void adjust_mem(mmu_t* mmu, uint16_t adr, int8_t val) {
	mmu->memory[adr] += val;
}
