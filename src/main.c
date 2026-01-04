#include "stm32f4xx.h"
#include <stdlib.h>

// ==========================================
// KONFIGURASI PIN FINAL
// ==========================================
// I2C LCD: PB6 (SCL), PB7 (SDA) -> Mode Open Drain
// Buzzer: PB10 (TIM2 CH3)
// Start Button: PA8
// LED Shift Reg: PA5 (SCK), PA7 (MOSI), PA4 (LATCH)
// Buttons: PA0-PA3, PA6, PB12-PB15

// NOTE: Jika masih tidak tampil, ganti ke 0x3F
#define I2C_ADDR 0x27  

// ==========================================
// VARIABEL GLOBAL
// ==========================================
volatile int lastHitButton = -1; 
volatile uint32_t msTicks = 0;
uint32_t gameSpeed = 1000;
uint32_t randomSeed = 0;

// ==========================================
// 1. FUNGSI SYSTEM & DELAY
// ==========================================
void SysTick_Handler(void) {
    msTicks++;
}

// Delay blocking sederhana
void Delay_us(uint32_t us) {
    uint32_t i;
    // Kalibrasi kasar untuk 100MHz (sekitar 100 cycle per us loop)
    for(i=0; i<us*16; i++) { __NOP(); } 
}

void Delay_ms(uint32_t ms) {
    uint32_t end = msTicks + ms;
    while (msTicks < end);
}

// ==========================================
// 2. DRIVER SHIFT REGISTER (LED)
// ==========================================
// Menggunakan GPIOA ODR langsung
#define SR_LATCH_LOW  (GPIOA->ODR &= ~(1 << 4))
#define SR_LATCH_HIGH (GPIOA->ODR |= (1 << 4))
#define SR_SCK_LOW    (GPIOA->ODR &= ~(1 << 5))
#define SR_SCK_HIGH   (GPIOA->ODR |= (1 << 5))
#define SR_DATA_LOW   (GPIOA->ODR &= ~(1 << 7))
#define SR_DATA_HIGH  (GPIOA->ODR |= (1 << 7))

void ShiftReg_Init(void) {
    // PA4, PA5, PA7 Output Push Pull
    // Reset bit mode dulu
    GPIOA->MODER &= ~((3<<(4*2)) | (3<<(5*2)) | (3<<(7*2)));
    // Set Output mode (01)
    GPIOA->MODER |= (1<<(4*2)) | (1<<(5*2)) | (1<<(7*2));
    
    // Set Output Speed High
    GPIOA->OSPEEDR |= (2<<(4*2)) | (2<<(5*2)) | (2<<(7*2));
    
    SR_LATCH_LOW; SR_SCK_LOW;
}

void ShiftReg_Send(uint16_t data) {
    SR_LATCH_LOW;
    for (int i = 15; i >= 0; i--) {
        SR_SCK_LOW;
        if (data & (1 << i)) SR_DATA_HIGH;
        else SR_DATA_LOW;
        for(int k=0; k<5; k++) __NOP(); 
        SR_SCK_HIGH;
        for(int k=0; k<5; k++) __NOP();
    }
    SR_LATCH_HIGH;
}

// ==========================================
// 3. INTERRUPT SERVICE ROUTINES (TOMBOL)
// ==========================================
volatile uint32_t lastInterruptTime = 0;

void Check_Button(int btnIndex) {
    if ((msTicks - lastInterruptTime) > 150) { // Debounce
        lastHitButton = btnIndex;
        lastInterruptTime = msTicks;
    }
}

void EXTI0_IRQHandler(void) { if(EXTI->PR & (1<<0)) { Check_Button(0); EXTI->PR |= (1<<0); } }
void EXTI1_IRQHandler(void) { if(EXTI->PR & (1<<1)) { Check_Button(1); EXTI->PR |= (1<<1); } }
void EXTI2_IRQHandler(void) { if(EXTI->PR & (1<<2)) { Check_Button(2); EXTI->PR |= (1<<2); } }
void EXTI3_IRQHandler(void) { if(EXTI->PR & (1<<3)) { Check_Button(3); EXTI->PR |= (1<<3); } }
void EXTI9_5_IRQHandler(void) { if(EXTI->PR & (1<<6)) { Check_Button(4); EXTI->PR |= (1<<6); } }
void EXTI15_10_IRQHandler(void) {
    if(EXTI->PR & (1<<12)) { Check_Button(5); EXTI->PR |= (1<<12); }
    if(EXTI->PR & (1<<13)) { Check_Button(6); EXTI->PR |= (1<<13); }
    if(EXTI->PR & (1<<14)) { Check_Button(7); EXTI->PR |= (1<<14); }
    if(EXTI->PR & (1<<15)) { Check_Button(8); EXTI->PR |= (1<<15); }
}

// ==========================================
// 4. DRIVER PWM BUZZER (PB10)
// ==========================================
void PWM_Init_PB10(void) {
    // PB10 Mode AF (10), AF1 (TIM2)
    GPIOB->MODER &= ~(3 << (10 * 2)); 
    GPIOB->MODER |=  (2 << (10 * 2));
    GPIOB->AFR[1] &= ~(0xF << 8); 
    GPIOB->AFR[1] |=  (0x1 << 8); 

    TIM2->PSC = 100 - 1; 
    TIM2->ARR = 1000 - 1; 
    TIM2->CCR3 = 0;
    TIM2->CCMR2 = (6 << 4) | (1 << 3); 
    TIM2->CCER |= (1 << 8); 
    TIM2->CR1 |= (1 << 0);  
}

void Beep_PWM(uint32_t freq_hz, uint32_t duration_ms) {
    if (freq_hz == 0) { TIM2->CCR3 = 0; return; }
    uint32_t period = 1000000 / freq_hz;
    TIM2->ARR = period - 1;
    TIM2->CCR3 = period / 2; 
    Delay_ms(duration_ms);
    TIM2->CCR3 = 0;
}

void Play_Tuturu(void) {
    Beep_PWM(1397, 100); Delay_ms(50);
    Beep_PWM(1760, 100); Delay_ms(50);
    Beep_PWM(2093, 300);
}

// ==========================================
// 5. DRIVER LCD I2C (SOFTWARE BIT-BANGING)
// ==========================================
// PB6 (SCL), PB7 (SDA) -> OPEN DRAIN MODE
#define SDA_HIGH (GPIOB->ODR |= (1<<7))
#define SDA_LOW  (GPIOB->ODR &= ~(1<<7))
#define SCL_HIGH (GPIOB->ODR |= (1<<6))
#define SCL_LOW  (GPIOB->ODR &= ~(1<<6))

// Primitive I2C Delay
void I2C_Delay(void) { Delay_us(5); }

void I2C_Start(void) { SDA_HIGH; SCL_HIGH; I2C_Delay(); SDA_LOW; I2C_Delay(); SCL_LOW; I2C_Delay(); }
void I2C_Stop(void) { SDA_LOW; SCL_LOW; I2C_Delay(); SCL_HIGH; I2C_Delay(); SDA_HIGH; I2C_Delay(); }

void I2C_WriteByte(uint8_t data) {
    for(int i=7; i>=0; i--) {
        if(data & (1<<i)) SDA_HIGH; else SDA_LOW;
        I2C_Delay(); SCL_HIGH; I2C_Delay(); SCL_LOW; I2C_Delay();
    }
    // ACK Pulse (Ignore actual ACK check for simplicity)
    SDA_HIGH; I2C_Delay(); SCL_HIGH; I2C_Delay(); SCL_LOW; I2C_Delay();
}

// Kirim data utuh ke PCF8574
void PCF8574_Write(uint8_t data) {
    I2C_Start();
    I2C_WriteByte(I2C_ADDR << 1); // Address + Write
    I2C_WriteByte(data);
    I2C_Stop();
}

// Kirim Nibble (4-bit) dengan pulse Enable
void LCD_Send_Nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble & 0xF0) | (rs & 0x01) | 0x08; // 0x08 = Backlight ON
    
    // Enable HIGH
    PCF8574_Write(data | 0x04); 
    Delay_us(50); // Delay agak lamaan dikit biar stabil
    
    // Enable LOW (Latch data)
    PCF8574_Write(data & ~0x04);
    Delay_us(50);
}

// --- FIXING BUG: SEND RAW NIBBLE FOR INIT ---
// Fungsi ini mengirim nibble langsung tanpa peduli flag RS/RW
// Wajib dipakai saat inisialisasi awal 8-bit ke 4-bit
void LCD_Send_Nibble_Direct(uint8_t nibble) {
    uint8_t data = (nibble << 4) | 0x08; // Backlight ON, RS=0, RW=0
    
    // Enable HIGH
    PCF8574_Write(data | 0x04);
    Delay_us(50);
    
    // Enable LOW
    PCF8574_Write(data & ~0x04); 
    Delay_us(50);
}

void LCD_Cmd(uint8_t cmd) { 
    LCD_Send_Nibble(cmd & 0xF0, 0);       // Upper
    LCD_Send_Nibble((cmd << 4) & 0xF0, 0); // Lower
    Delay_ms(2); 
}

void LCD_Data(uint8_t data) { 
    LCD_Send_Nibble(data & 0xF0, 1);       // Upper
    LCD_Send_Nibble((data << 4) & 0xF0, 1); // Lower
    Delay_ms(1); 
}

void LCD_String(char *str) { while (*str) LCD_Data(*str++); }

void LCD_Init(void) {
    // Inisialisasi PB6 & PB7 sebagai Open Drain Output
    GPIOB->MODER &= ~((3<<(6*2)) | (3<<(7*2))); 
    GPIOB->MODER |=  ((1<<(6*2)) | (1<<(7*2))); 
    GPIOB->OTYPER |= (1<<6) | (1<<7); // Open Drain
    GPIOB->OSPEEDR |= (3<<(6*2)) | (3<<(7*2)); 
    GPIOB->ODR |= (1<<6) | (1<<7); 

    Delay_ms(50); // Tunggu VCC Stabil

    // === MAGIC SEQUENCE START ===
    // Memaksa LCD reset dari mode 8-bit (default pabrik) ke 4-bit
    // Kita harus kirim sinyal 0x03 sebanyak 3 kali secara manual
    
    LCD_Send_Nibble_Direct(0x03);
    Delay_ms(5); // Wait > 4.1ms
    
    LCD_Send_Nibble_Direct(0x03);
    Delay_ms(1); // Wait > 100us
    
    LCD_Send_Nibble_Direct(0x03);
    Delay_ms(1);
    
    // Sekarang kirim 0x02 untuk switch ke 4-bit mode
    LCD_Send_Nibble_Direct(0x02);
    Delay_ms(1);
    // === MAGIC SEQUENCE END ===

    // Sekarang LCD sudah mode 4-bit, bisa pakai perintah biasa
    LCD_Cmd(0x28); // Function Set: 4-bit, 2 Line, 5x8 Dots
    LCD_Cmd(0x0C); // Display ON, Cursor OFF
    LCD_Cmd(0x06); // Entry Mode: Increment
    LCD_Cmd(0x01); // Clear Display
    Delay_ms(10);
}

void LCD_UpdateStatus(int score, int timeLeftSec, int level) {
    LCD_Cmd(0x80); // Baris 1
    LCD_String("Scr:"); LCD_Data((score/10)+'0'); LCD_Data((score%10)+'0');
    LCD_String(" Lvl:"); LCD_Data(level+'0');
    
    LCD_Cmd(0xC0); // Baris 2
    LCD_String("Time:"); 
    LCD_Data((timeLeftSec/10)+'0'); LCD_Data((timeLeftSec%10)+'0'); 
    LCD_String("s ");
}

uint32_t GetRandom(void) {
    if (randomSeed == 0) randomSeed = msTicks + 12345;
    randomSeed = randomSeed * 1664525 + 1013904223;
    return randomSeed;
}

// ==========================================
// 6. MAIN PROGRAM
// ==========================================
int main(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000); // 1ms Tick

    // Enable Clocks
    RCC->AHB1ENR |= (1 << 0) | (1 << 1); // Enable GPIOA, GPIOB
    RCC->APB1ENR |= (1 << 0); // TIM2
    RCC->APB2ENR |= (1 << 14); // SYSCFG

    // Setup Start Button (PA8) Input Pull-Up
    GPIOA->MODER &= ~(3 << (8*2)); 
    GPIOA->PUPDR |= (1 << (8*2));

    // Setup Buttons Input Pull-Up
    GPIOA->MODER &= ~((3<<(0*2))|(3<<(1*2))|(3<<(2*2))|(3<<(3*2))|(3<<(6*2)));
    GPIOA->PUPDR |=  ((1<<(0*2))|(1<<(1*2))|(1<<(2*2))|(1<<(3*2))|(1<<(6*2)));

    GPIOB->MODER &= ~((3<<(12*2))|(3<<(13*2))|(3<<(14*2))|(3<<(15*2)));
    GPIOB->PUPDR |=  ((1<<(12*2))|(1<<(13*2))|(1<<(14*2))|(1<<(15*2)));

    // Setup EXTI (Interrupts)
    SYSCFG->EXTICR[0] &= ~0xFFFF; // PA0-PA3
    SYSCFG->EXTICR[1] &= ~(0xF << 8); // PA6
    SYSCFG->EXTICR[3] = 0x1111; // PB12-PB15 (0001 = Port B)

    EXTI->IMR |= 0xF04F; 
    EXTI->FTSR |= 0xF04F; 

    NVIC_EnableIRQ(EXTI0_IRQn); NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn); NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI9_5_IRQn); 
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    // Init Drivers
    ShiftReg_Init();
    PWM_Init_PB10();
    LCD_Init(); 

    int currentLevel = 1;

    // --- MAIN LOOP ---
    while (1) {
        // A. MODE STANDBY
        LCD_Cmd(0x80); LCD_String("WHACK-A-MOLE    ");
        LCD_Cmd(0xC0); LCD_String("PRESS START!    ");
        
        while ((GPIOA->IDR & (1 << 8))) { // Wait PA8 LOW (Start)
             randomSeed++; 
             Delay_ms(10);
        }

        // B. GAME START
        LCD_Cmd(0x01); LCD_String("GO! GO! GO!");
        Play_Tuturu(); 

        int score = 0;
        int32_t timeRemainingMs = 60000;
        int lastMole = -1;

        // C. GAME LOOP
        while (timeRemainingMs > 0) {
            gameSpeed = 1300 - (currentLevel * 100);
            LCD_UpdateStatus(score, timeRemainingMs / 1000, currentLevel);

            int currentMole;
            do { currentMole = GetRandom() % 9; } while (currentMole == lastMole);
            lastMole = currentMole;

            ShiftReg_Send(1 << currentMole); // LED ON
            lastHitButton = -1;

            int hit = 0;
            int timeSpent = 0;
            int step = 10;

            for (int t = 0; t < gameSpeed; t += step) {
                if (lastHitButton == currentMole) {
                    hit = 1; score++;
                    ShiftReg_Send(0); // LED OFF Cepat
                    Beep_PWM(2000, 100);
                    timeSpent = t;
                    lastHitButton = -1;
                    
                    if (score > 0 && score % 10 == 0) {
                        currentLevel++;
                        timeRemainingMs = 60000; 
                        Beep_PWM(3000, 200);
                    }
                    break;
                }
                else if (lastHitButton != -1 && lastHitButton != currentMole) {
                    Beep_PWM(200, 300); 
                    if(currentLevel > 1) currentLevel--; 
                    score = 0; 
                    timeRemainingMs = 60000; 
                    
                    LCD_Cmd(0x01); LCD_String("WRONG HIT!");
                    Delay_ms(1000);
                    lastHitButton = -1;
                    break; 
                }
                Delay_ms(step);
            }
            
            if (!hit && lastHitButton == -1) {
                timeSpent = gameSpeed;
            }
            
            timeRemainingMs -= timeSpent;
            ShiftReg_Send(0);
            Delay_ms(150); timeRemainingMs -= 150;
        }

        LCD_Cmd(0x01); LCD_String("GAME OVER!");
        LCD_Cmd(0xC0); LCD_String("Score: ");
        LCD_Data((score/10)+'0'); LCD_Data((score%10)+'0');
        Beep_PWM(1000, 500); Delay_ms(3000);
    }
}