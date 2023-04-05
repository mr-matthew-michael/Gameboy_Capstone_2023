#define _CRT_SECURE_NO_DEPRECATE
#include "SDL2/include/SDL.h"
#include <fstream>
#include "cpu.h"
#include "ppu.h"
#include "mmu.h"
#include <string>
#include <string.h>
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <Windows.h>
#include <WinUser.h>
#include "SDL2/include/SDL_syswm.h"
#include "commctrl.h"

#undef main

using namespace std;

uint8_t cartridge[0xFA000];
string filename = "tetris.gb";

//	Main Vars
SDL_Window* mainWindow;
SDL_Event event;
SDL_GameController* controller;
uint8_t joypad = 0xff;
int interrupts_enabled = 0;
int timer_clocksum = 0;
int div_clocksum = 0;
double overhead = 0;
int LYlast = 0;
bool unpaused = 1;

void init_window();
void update_timer(mmu_t* mmu, int clocks);
void window_events(SDL_Event event);
void handle_interrupts(mmu_t* mmu, cpu_t* cpu);

int main() {

	mmu_t* mmu;
	mmu = (mmu_t*)malloc(sizeof(mmu_t));
	create_mmu(mmu);
	cpu_t* cpu = initializeCPU();

	//	load cartridge
	FILE* file = fopen(filename.c_str(), "rb");
	int pos = 0;
	while (fread(&cartridge[pos], 1, 1, file)) {
		pos++;
	}
	fclose(file);
	load_game(mmu, cartridge);

	SDL_Init(SDL_INIT_GAMECONTROLLER);
	controller = SDL_NumJoysticks() ? SDL_GameControllerOpen(0) : NULL;

	init_mmu(mmu);

	init_ppu();

	init_window();

	int sum = 0;
	int cyc = 0;

	//	start CPU
	while (1) {

		if (unpaused) {

			if (!cpu->halt)
				cyc = stepCPU(cpu, mmu);
			else
				cyc = 1;

			stepPPU(mmu, cyc * 4);

			update_timer(mmu, cyc);

			handle_interrupts(mmu, cpu);

			if (mmu_read_byte(mmu, 0xff44) == 154)
				window_events(event);
		}
		else
			window_events(event);
	}

	stopPPU();

	return 0;
}

void update_timer(mmu_t* mmu, int cycles) {
	// Update divider register
	div_clocksum += cycles;
	if (div_clocksum >= 256) {
		div_clocksum -= 256;
		adjust_mem(mmu, 0xff04, 1);
	}

	// Check if timer is enabled
	uint8_t tac = mmu_read_byte(mmu, 0xff07);
	if ((tac >> 2) & 0x1) {
		timer_clocksum += cycles * 4;

		// Set timer frequency based on TAC register
		int freq[] = { 4096, 262144, 65536, 16384 };
		int current_freq = freq[tac & 0x3];

		// Update timer and check for overflow
		while (timer_clocksum >= (4194304 / current_freq)) {
			adjust_mem(mmu, 0xff05, 1);

			if (mmu_read_byte(mmu, 0xff05) == 0x00) {
				mmu_write_byte(mmu, 0xff0f, mmu_read_byte(mmu, 0xff0f) | 4);
				mmu_write_byte(mmu, 0xff05, mmu_read_byte(mmu, 0xff06));
			}
			timer_clocksum -= (4194304 / current_freq);
		}
	}
}

void handle_interrupts(mmu_t* mmu, cpu_t* cpu) {
	uint8_t int_flags = mmu_read_byte(mmu, 0xff0f);
	uint8_t int_enabled = mmu_read_byte(mmu, 0xffff);

	if (int_flags & int_enabled && cpu->halt) {
		cpu->halt = false;
	}

	if (cpu->ime && (int_flags & int_enabled)) {
		static const uint8_t interrupt_masks[] = { 0x01, 0x02, 0x04 };
		static const uint16_t interrupt_vectors[] = { 0x40, 0x48, 0x50 };

		for (size_t i = 0; i < 3; ++i) {
			if (int_enabled & interrupt_masks[i] && int_flags & interrupt_masks[i]) {
				cpu->reg.sp -= 2;
				mmu_write_word(mmu, cpu->reg.sp, cpu->reg.pc);
				cpu->reg.pc = interrupt_vectors[i];
				mmu_write_byte(mmu, 0xff0f, int_flags & ~interrupt_masks[i]);
				cpu->ime = false;
				break;
			}
		}
	}
}


void init_window() {
	mainWindow = getWindow();
	char title[50];
	string rom = filename;
	if (filename.find_last_of("\\") != string::npos)
		rom = filename.substr(filename.find_last_of("\\") + 1);
	snprintf(title, sizeof title, "GameboyEmu [ rom: %s ]", rom.c_str());
	SDL_SetWindowTitle(mainWindow, title);
	SDL_SysWMinfo wmInfo;
	SDL_VERSION(&wmInfo.version);
	SDL_GetWindowWMInfo(mainWindow, &wmInfo);
	HWND hwnd = wmInfo.info.win.window;

	SDL_EventState(SDL_SYSWMEVENT, SDL_ENABLE);
}

void window_events(SDL_Event event) {
	//	poll events from menu
	SDL_PollEvent(&event);

	switch (event.type)
	{
	case SDL_SYSWMEVENT:
		//	close a window
		if (event.syswm.msg->msg.win.msg == WM_CLOSE) {
			DestroyWindow(event.syswm.msg->msg.win.hwnd);
			PostMessage(event.syswm.msg->msg.win.hwnd, WM_CLOSE, 0, 0);
		}
		break;
	};
}

uint8_t controller_input(uint8_t val) {
	const uint8_t* keys = SDL_GetKeyboardState(NULL);
	uint8_t joypad = 0;

	if ((val & 0x30) == 0x10) {
		static const SDL_Scancode button_scancodes[] = { SDL_SCANCODE_A, SDL_SCANCODE_S, SDL_SCANCODE_X, SDL_SCANCODE_Z };
		for (size_t i = 0; i < 4; ++i) {
			joypad |= (keys[button_scancodes[i]] ? 0 : 1) << i;
		}
	}
	else if ((val & 0x30) == 0x20) {
		static const SDL_Scancode direction_scancodes[] = { SDL_SCANCODE_RIGHT, SDL_SCANCODE_LEFT, SDL_SCANCODE_UP, SDL_SCANCODE_DOWN };
		for (size_t i = 0; i < 4; ++i) {
			joypad |= (keys[direction_scancodes[i]] ? 0 : 1) << i;
		}
	}

	val &= 0xf0;
	val |= 0xc0;
	return (val | joypad);
}