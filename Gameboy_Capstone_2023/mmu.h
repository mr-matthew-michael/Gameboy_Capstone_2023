#pragma once
#include <cstdint> 

typedef struct _mmu_t
{
	uint8_t memory[0xFA000];
	uint8_t rom[0xFA000];
} mmu_t;

void create_mmu(mmu_t* mmu);
void init_mmu(mmu_t* mmu);
void load_game(mmu_t* mmu, uint8_t c[]);
void load_boot_screen(mmu_t* mmu);
void lock_boot_screen(mmu_t* mmu);
uint8_t mmu_write_byte(mmu_t* mmu, uint16_t, unsigned char);
uint8_t mmu_write_word(mmu_t* mmu, uint16_t addr, uint16_t data);
uint16_t mmu_read_word(mmu_t* mmu, uint16_t addr);
void adjust_mem(mmu_t* mmu, uint16_t, int8_t);
uint8_t& mmu_read_byte(mmu_t* mmu, uint16_t);
void set_ppu_state(uint8_t state);
