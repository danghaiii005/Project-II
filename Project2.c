// Project: Motor Control TB6612FNG & PID Park Mode Hold
// Dev: NguyenDangHai_20233380 (Updated: PID Position Control for Park Mode)

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h> // S? d?ng hàm abs()

// Mã hex LED 7 do?n Anode chung (0 -> 9)
const unsigned char led7_hex[10] = {0x88, 0xEB, 0x4C, 0x49, 0x2B, 0x19, 0x18, 0xCB, 0x08, 0x09};
#define HEX_P 0x0E 

// Tr?ng thái h? th?ng: 0 = OFF, 1 = RUN, 2 = PARK (PID Hold)
uint8_t system_mode = 0; 
uint8_t prev_mode = 0;

// --- BI?N ENCODER VÀ PID ---
volatile int32_t current_pos = 0; // V? trí hi?n t?i c?a d?ng co
int32_t target_pos = 0;           // V? trí m?c tiêu khi khóa bánh

float Kp = 15.0;  // H? s? góc (L?c gi? chính - tang n?u bánh l?ng)
float Ki = 0.5;   // H? s? tích phân (Kéo bánh v? dúng sai s? 0)
float Kd = 2.0;   // H? s? vi phân (Ch?ng rung, gi?m dao d?ng)

float integral = 0;
int32_t last_error = 0;

// C?u hình ng?t ngoài INT0 (Chân PD2) d? d?c Encoder Kênh A
void Interrupt_Init() {
    MCUCR |= (1 << ISC01); // Kích ho?t ng?t khi có su?n xu?ng (Falling Edge) trên INT0
    GICR |= (1 << INT0);   // Cho phép ng?t INT0
}

// Trình ph?c v? ng?t INT0 (M?i khi Kênh A có su?n xu?ng)
ISR(INT0_vect) {
    // Ð?c tr?ng thái Kênh B (PD3) d? xác d?nh chi?u
    if (PIND & (1 << PD3)) {
        current_pos++; // Quay thu?n
    } else {
        current_pos--; // Quay ngh?ch
    }
}

// C?u hình PWM trên Timer2 (Chân PD7 - OC2)
void Timer2_PWM_Init() {
    TCCR2 = (1 << WGM20) | (1 << WGM21) | (1 << COM21) | (1 << CS22); // Fast PWM, Prescaler 64
    OCR2 = 0;
}

// Hàm di?u khi?n d?ng co chung
void Motor_Drive(int16_t speed) {
    if (speed > 0) {
        PORTD |= (1 << PD0);
        PORTD &= ~(1 << PD1);
        OCR2 = (speed > 255) ? 255 : speed;
    } else if (speed < 0) {
        PORTD &= ~(1 << PD0);
        PORTD |= (1 << PD1);
        OCR2 = (speed < -255) ? 255 : abs(speed);
    } else {
        // Phanh d?ng nang d? h? tr? ch?ng trôi n?u sai s? PID = 0
        PORTD |= (1 << PD0) | (1 << PD1);
        OCR2 = 255;
    }
}

int main(void) {
    // 1. C?u hình PORT I/O
    DDRD |= (1 << PD0) | (1 << PD1); // Chi?u motor
    
    DDRB &= 0xF0;  // PB0->PB3 làm ngõ vào
    PORTB |= 0x0F; // Tr? kéo lên n?i b?

    DDRC = 0xFF;   // C?ng xu?t LED 7 do?n
    
    // PORTD: PD2(INT0, Kênh A), PD3(Kênh B) là Input. PD7 (PWM) là Output.
    DDRD = (1 << PD7) | (1 << PD0) | (1 << PD1) | (1 << PD4) | (1 << PD5) | (1 << PD6); 
    PORTD |= (1 << PD2) | (1 << PD3); // Tr? kéo lên cho 2 kênh Encoder

    Timer2_PWM_Init();
    Interrupt_Init();
    sei(); // B?t ng?t toàn c?c

    // 2. Bi?n v?n hành bình thu?ng
    uint8_t motor_dir = 0;      
    uint8_t speed_level = 1;    

    while(1) {
        prev_mode = system_mode; // Luu tr?ng thái tru?c dó d? b?t s? ki?n chuy?n Mode

        // --- X? LÝ NÚT NH?N PB0 (CHUY?N MODE) ---
        if ((PINB & (1 << PB0)) == 0) {
            _delay_ms(20);
            if ((PINB & (1 << PB0)) == 0) {
                system_mode++;
                if (system_mode > 2) system_mode = 0;
                while ((PINB & (1 << PB0)) == 0); 
            }
        }

        // --- NÚT PB1: Ð?O CHI?U (Mode RUN) ---
        if ((PINB & (1 << PB1)) == 0) {
            _delay_ms(20);
            if ((PINB & (1 << PB1)) == 0) {
                if (system_mode == 1) motor_dir = !motor_dir;
                while ((PINB & (1 << PB1)) == 0);
            }
        }

        // --- NÚT PB2: GI?M T?C Ð? (Mode RUN) ---
        if ((PINB & (1 << PB2)) == 0) {
            _delay_ms(20);
            if ((PINB & (1 << PB2)) == 0) {
                if (system_mode == 1 && speed_level > 1) speed_level--;
                while ((PINB & (1 << PB2)) == 0);
            }
        }

        // --- NÚT PB3: TANG T?C Ð? (Mode RUN) ---
        if ((PINB & (1 << PB3)) == 0) {
            _delay_ms(20);
            if ((PINB & (1 << PB3)) == 0) {
                if (system_mode == 1 && speed_level < 10) speed_level++;
                while ((PINB & (1 << PB3)) == 0);
            }
        }

        // ==========================================
        //         X? LÝ ÐI?U KHI?N Ð?NG CO
        // ==========================================
        if (system_mode == 1) { 
            // --- MODE 1: RUN BÌNH THU?NG ---
            int16_t current_speed_pwm = (speed_level == 10) ? 255 : (speed_level * 25);
            if (motor_dir == 1) current_speed_pwm = -current_speed_pwm;
            Motor_Drive(current_speed_pwm);

        } else if (system_mode == 2) { 
            // --- MODE 2: PID POSITION PARK HOLD ---
            
            // X?y ra duy nh?t 1 l?n khi v?a nh?y t? Mode khác sang Mode 2
            if (prev_mode != 2) {
                target_pos = current_pos; // Ch?t v? trí hi?n t?i làm m?c tiêu
                integral = 0;             // Reset b? tích phân
                last_error = 0;           // Reset b? vi phân
            }

            // Tính toán PID
            int32_t error = target_pos - current_pos;
            integral += error;
            
            // Gi?i h?n khâu Tích phân (Anti-windup) tránh v?t l? quá l?n
            if (integral > 1000) integral = 1000;
            if (integral < -1000) integral = -1000;

            int32_t derivative = error - last_error;
            
            // Output = P + I + D
            int16_t pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            
            last_error = error;

            Motor_Drive(pid_output);

        } else { 
            // --- MODE 0: OFF ---
            Motor_Drive(0); // Lúc này g?i hàm s? r? vào nhánh hãm d?ng nang. 
            // N?u b?n mu?n bánh xe trôi t? do hoàn toàn, c?u hình IN1=0, IN2=0.
            PORTD &= ~(1 << PD0);
            PORTD &= ~(1 << PD1);
            OCR2 = 0;
        }

        // --- C?P NH?T LED 7 ÐO?N ---
        if (system_mode == 2) {
            PORTC = HEX_P; 
        } else if (system_mode == 1) {
            if (speed_level == 10) PORTC = led7_hex[9] & ~(1 << PC3); 
            else PORTC = led7_hex[speed_level];
        } else {
            PORTC = led7_hex[0];
        }

        _delay_ms(10); // L?y m?u PID và vòng l?p ? t?n s? 100Hz (10ms)
    }
    return 0;
}
