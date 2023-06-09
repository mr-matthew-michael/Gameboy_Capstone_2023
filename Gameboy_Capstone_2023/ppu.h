void init_ppu();
SDL_Window* getWindow();
void stepPPU(mmu_t* mmu, uint8_t cycles);
void drawLine(mmu_t* mmu);
void drawBGTileset(mmu_t* mmu, SDL_Renderer* tRenderer, SDL_Window* tWindow);
void drawBGMap(mmu_t* mmu, SDL_Renderer* tRenderer, SDL_Window* tWindow);
void drawWindowMap(mmu_t* mmu, SDL_Renderer* tRenderer, SDL_Window* tWindow);
void drawSpriteMap(mmu_t* mmu, SDL_Renderer* tRenderer, SDL_Window* tWindow);
void stopPPU();
void setPalette(uint8_t val);