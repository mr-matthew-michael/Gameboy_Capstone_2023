#include "SDL2/include/SDL.h"
#include <iostream>
#include "mmu.h"

//	init PPU
SDL_Renderer* renderer;
SDL_Window* window;
SDL_Texture* texture;
SDL_Texture* textureA;

const int FB_SIZE = (256 * 256 * 3);
const int FB_SIZE_A = (256 * 256 * 4);

uint8_t fram_buff_A[160 * 144 * 4];
uint8_t background_map_A[FB_SIZE_A];
uint8_t window_map_A[FB_SIZE_A];
uint8_t sprite_map_A[FB_SIZE_A];

uint8_t SCY, SCX, STAT, LY, LYC, LCDC, WY, WX;
uint8_t last_mode = 0;
uint16_t ppu_cyc, frame_draw_flag = 0, line_calc_flag = 0;

int tile_map;
int tile_data;
int tile_number, color_value, color_pal;
int row, x_offset, y_offset, x_offset_S, y_offset_S, x_offset_A, y_offset_A;

int COLORS[] = {
		0xff,0xff,0xff,
		0xaa,0xaa,0xaa,
		0x55,0x55,0x55,
		0x00,0x00,0x00
};


void init_ppu() {

	//	init and create window and renderer
	SDL_Init(SDL_INIT_VIDEO);
	SDL_SetHint(SDL_HINT_RENDER_VSYNC, "1");
	SDL_CreateWindowAndRenderer(160, 144, 0, &window, &renderer);
	SDL_SetWindowSize(window, 480, 432);
	SDL_RenderSetLogicalSize(renderer, 160, 144);
	SDL_SetWindowResizable(window, SDL_TRUE);

	texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, 160, 144);
	textureA = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA32, SDL_TEXTUREACCESS_STREAMING, 160, 144);

	memset(background_map_A, 0, FB_SIZE_A);
	memset(sprite_map_A, 0, FB_SIZE_A);
	memset(window_map_A, 0, FB_SIZE_A);

	//	reset vars
	SCY = 0;
	SCX = 0;

}

SDL_Window* getWindow() {
	return window;
}

void calculate_background(mmu_t* memory_unit, uint8_t current_row) {
	uint8_t scrollY = mmu_read_byte(memory_unit, 0xff42);
	uint8_t scrollX = mmu_read_byte(memory_unit, 0xff43);
	uint16_t background_map = (((mmu_read_byte(memory_unit, 0xff40) >> 3) & 1) == 1) ? 0x9c00 : 0x9800;
	uint16_t tile_data = (((mmu_read_byte(memory_unit, 0xff40) >> 4) & 1) == 1) ? 0x8000 : 0x8800;
	uint8_t palette = mmu_read_byte(memory_unit, 0xff47);

	for (int i = 0; i < 256; i++) {
		uint8_t offsetY = current_row + scrollY;
		uint8_t offsetX = i + scrollX;
		uint8_t tile_nr = mmu_read_byte(memory_unit, background_map + ((offsetY / 8 * 32) + (offsetX / 8)));

		uint8_t color_value;
		if (tile_data == 0x8800) {
			color_value = (mmu_read_byte(memory_unit, tile_data + 0x800 + ((int8_t)tile_nr * 0x10) + (offsetY % 8 * 2)) >> (7 - (offsetX % 8)) & 0x1) + ((mmu_read_byte(memory_unit, tile_data + 0x800 + ((int8_t)tile_nr * 0x10) + (offsetY % 8 * 2) + 1) >> (7 - (offsetX % 8)) & 0x1) * 2);
		}
		else {
			color_value = (mmu_read_byte(memory_unit, tile_data + (tile_nr * 0x10) + (offsetY % 8 * 2)) >> (7 - (offsetX % 8)) & 0x1) + (mmu_read_byte(memory_unit, tile_data + (tile_nr * 0x10) + (offsetY % 8 * 2) + 1) >> (7 - (offsetX % 8)) & 0x1) * 2;
		}

		uint8_t color_from_palette = (palette >> (2 * color_value)) & 3;
		uint32_t bgmap_offset = (current_row * 256 * 4) + (i * 4);
		for (int j = 0; j < 3; j++) {
			background_map_A[bgmap_offset + j] = COLORS[color_from_palette * 3 + j];
		}
		background_map_A[bgmap_offset + 3] = 0xff;
	}
}


void calculate_window(mmu_t* mmu, uint8_t row) {
	uint16_t tilemap = (((mmu_read_byte(mmu, 0xff40) >> 5) & 1) == 1) ? 0x9c00 : 0x9800;
	uint16_t tiledata = (((mmu_read_byte(mmu, 0xff40) >> 4) & 1) == 1) ? 0x8000 : 0x8800;

	uint8_t LCDC = mmu_read_byte(mmu, 0xff40);
	uint8_t WY = mmu_read_byte(mmu, 0xff4a);
	uint8_t WX = mmu_read_byte(mmu, 0xff4b) - 7;

	if (((LCDC >> 5) & 0x01) == 1) {
		for (int j = 0; j < 256; j++) {
			if (WX <= j && j <= WX + 160 && WY <= row && row <= WY + 144) {
				uint8_t tilenr = mmu_read_byte(mmu, tilemap + (((row - WY) / 8 * 32) + ((j - WX) / 8)));
				uint8_t colorval = ((mmu_read_byte(mmu, tiledata + (tilenr * 0x10) + ((row - WY) % 8 * 2)) >> (7 - ((j - WX) % 8))) & 0x1) + ((mmu_read_byte(mmu, tiledata + (tilenr * 0x10) + ((row - WY) % 8 * 2) + 1) >> (7 - ((j - WX) % 8))) & 0x1) * 2;

				if (tiledata == 0x8800) {
					colorval = ((mmu_read_byte(mmu, tiledata + 0x800 + ((int8_t)tilenr * 0x10) + ((row - WY) % 8 * 2)) >> (7 - ((j - WX) % 8))) & 0x1) + ((mmu_read_byte(mmu, tiledata + 0x800 + ((int8_t)tilenr * 0x10) + ((row - WY) % 8 * 2) + 1) >> (7 - ((j - WX) % 8))) & 0x1) * 2;
				}

				uint8_t colorfrompal = (mmu_read_byte(mmu, 0xff47) >> (2 * colorval)) & 3;
				int offset = (row * 256 * 4) + (j * 4);

				window_map_A[offset] = COLORS[colorfrompal * 3];
				window_map_A[offset + 1] = COLORS[colorfrompal * 3 + 1];
				window_map_A[offset + 2] = COLORS[colorfrompal * 3 + 2];
				window_map_A[offset + 3] = 0xff;
			}
		}
	}
}


void calculate_sprite(mmu_t* mmu, uint8_t row) {
	uint16_t pat = 0x8000, oam = 0xfe00, oam_end = 0xfe9f;

	if ((mmu_read_byte(mmu, 0xff40) >> 1) & 0x01) {
		for (uint16_t i = oam; i <= oam_end; i += 4) {
			uint8_t y = mmu_read_byte(mmu, i), x = mmu_read_byte(mmu, i + 1), tilenr = mmu_read_byte(mmu, i + 2), flags = mmu_read_byte(mmu, i + 3);
			uint8_t flip = (flags >> 5) & 0x01 | (((flags >> 6) & 0x01) << 1);
			uint8_t height = ((mmu_read_byte(mmu, 0xff40) >> 2) & 0x01) ? 16 : 8;

			if (row >= (y - 16) && row <= ((y - 16) + height)) {
				for (int u = 0; u < height; u++) {
					for (int v = 0; v < 8; v++) {
						switch (flags & 0x60) {
						case 0x00: // no flip
							color_value = (mmu_read_byte(mmu, pat + (tilenr * 0x10) + (u * 2)) >> (7 - v) & 0x1) + (mmu_read_byte(mmu, pat + (tilenr * 0x10) + (u * 2) + 1) >> (7 - v) & 0x1) * 2;
							break;
						case 0x20: // only x-flip
							color_value = (mmu_read_byte(mmu, pat + (tilenr * 0x10) + (u * 2)) >> v & 0x1) + (mmu_read_byte(mmu, pat + (tilenr * 0x10) + (u * 2) + 1) >> v & 0x1) * 2;
							break;
						case 0x40: // only y-flip
							color_value = (mmu_read_byte(mmu, pat + (tilenr * 0x10) + ((height - u - 1) * 2)) >> (7 - v) & 0x1) + (mmu_read_byte(mmu, pat + (tilenr * 0x10) + ((height - u - 1) * 2) + 1) >> (7 - v) & 0x1) * 2;
							break;
						case 0x60: // x-flip and y-flip
							color_value = (mmu_read_byte(mmu, pat + (tilenr * 0x10) + ((height - u - 1) * 2)) >> v & 0x1) + (mmu_read_byte(mmu, pat + (tilenr * 0x10) + ((height - u - 1) * 2) + 1) >> v & 0x1) * 2;
							break;
						}


						uint16_t pal = ((flags >> 4) & 1) ? 0xff49 : 0xff48;
						color_pal = (mmu_read_byte(mmu, pal) >> (2 * color_value)) & 3;

						if (color_value != 0 && ((y + u) <= 0xff) && ((x + v) <= 0xff)) {
							int offset = ((y + u - 16) * 256 * 4) + ((x + v - 8) * 4);
							for (int c = 0; c < 3; c++) {
								sprite_map_A[offset + c] = COLORS[color_pal * 3 + c];
							}
							sprite_map_A[offset + 3] = 0xff;
						}
					}
				}
			}
		}
	}
}

void frame_draw() {
	for (int r = 0; r < 144; r++) {
		for (int col = 0; col < 160; col++) {
			int yoffA = r * 256 * 4;
			int xoffA = col * 4;
			int frameOffset = (r * 160 * 4) + xoffA;

			for (int i = 0; i < 4; i++) {
				fram_buff_A[frameOffset + i] = background_map_A[yoffA + xoffA + i];
				if (window_map_A[yoffA + xoffA + 3] != 0x00) {
					fram_buff_A[frameOffset + i] = window_map_A[yoffA + xoffA + i];
				}
				if (sprite_map_A[yoffA + xoffA + 3] != 0x00) {
					fram_buff_A[frameOffset + i] = sprite_map_A[yoffA + xoffA + i];
				}
			}
		}
	}

	SDL_UpdateTexture(textureA, NULL, fram_buff_A, 160 * sizeof(unsigned char) * 4);
	SDL_RenderCopy(renderer, textureA, NULL, NULL);
	SDL_RenderPresent(renderer);
}


void stepPPU(mmu_t* mmu, uint8_t cycles) {
	ppu_cyc += cycles;

	const uint8_t mode = ppu_cyc < 81 ? 2 :
		ppu_cyc < 253 ? 3 :
		ppu_cyc < 457 ? 0 :
		mmu_read_byte(mmu, 0xff44) >= 144 ? 1 : 0;
	const uint8_t stat_mask = (mmu_read_byte(mmu, 0xff41) & 0xfc) | mode;
	mmu_write_byte(mmu, 0xff41, stat_mask);
	set_ppu_state(mode);

	if (ppu_cyc > 252 && mmu_read_byte(mmu, 0xff44) < 145 && !line_calc_flag) {
		calculate_background(mmu, mmu_read_byte(mmu, 0xff44));
		calculate_window(mmu, mmu_read_byte(mmu, 0xff44));
		calculate_sprite(mmu, mmu_read_byte(mmu, 0xff44));
		line_calc_flag = 1;
	}

	if (!frame_draw_flag && mmu_read_byte(mmu, 0xff44) == 144 && mmu_read_byte(mmu, 0xff40) >> 7) {
		mmu_write_byte(mmu, 0xff0f, mmu_read_byte(mmu, 0xff0f) | 1);
		frame_draw();
		frame_draw_flag = 1;
		memset(background_map_A, 0, FB_SIZE);
		memset(sprite_map_A, 0, FB_SIZE);
		memset(window_map_A, 0, FB_SIZE);
	}

	if (ppu_cyc > 456) {
		ppu_cyc -= 456;
		const uint8_t ly = mmu_read_byte(mmu, 0xff44);
		mmu_write_byte(mmu, 0xff44, ly + 1);
		line_calc_flag = 0;
		if (ly == mmu_read_byte(mmu, 0xff45)) {
			const uint8_t stat = mmu_read_byte(mmu, 0xff41);
			if (stat & 0x40 && !(stat & 0x04)) {
				mmu_write_byte(mmu, 0xff0f, mmu_read_byte(mmu, 0xff0f) | 2);
				mmu_write_byte(mmu, 0xff41, stat | 0x04);
			}
		}
		else {
			mmu_write_byte(mmu, 0xff41, mmu_read_byte(mmu, 0xff41) & ~0x04);
		}
	}

	if (mmu_read_byte(mmu, 0xff44) > 154) {
		mmu_write_byte(mmu, 0xff44, 0);
		frame_draw_flag = 0;
	}
}


void stopPPU() {
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}