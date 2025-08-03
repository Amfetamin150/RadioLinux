.nolist
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
.list

.equ F_CPU = 8000000
.equ FM_I2C_ADDR = 0x10
.equ AM_I2C_ADDR = 0x11
.equ MODE_PIN = 0
.equ UP_PIN = 1
.equ DOWN_PIN = 2
.equ SELECT_PIN = 3
.equ LCD_TIMEOUT = 10
.equ DEBOUNCE_MS = 20
.equ ADC_PIN = 0
.equ MIN_VOLTAGE = 30
.equ FIRMWARE_CHECK = 0xAA
.equ AM_MIN_FREQ = 153
.equ AM_MAX_FREQ = 18000
.equ AM_STEP_LW_MW = 9
.equ AM_STEP_SW = 5
.equ FM_MIN_FREQ = 8700
.equ FM_MAX_FREQ = 10800
.equ FM_STEP = 10
.equ VOLUME_MIN = 0
.equ VOLUME_MAX = 15
.equ I2C_RETRIES = 3

.equ LCD_PORT = PORTD
.equ LCD_DDR = DDRD
.equ LCD_RS = 4
.equ LCD_EN = 5
.equ LCD_PORTB = PORTB
.equ LCD_DDRB = DDRB
.equ LCD_BL = 6

.macro CHECK_I2C_STATUS expected
    lds r16, TWSR
    andi r16, 0xF8
    cpi r16, \expected
    brne i2c_error
.endmacro

.macro DELAY_MS ms
    ldi r24, \ms
    rcall delay_ms
.endmacro

.macro WDT_RESET
    wdr
.endmacro

.section .vectors
    rjmp main
    rjmp int0_isr
    rjmp int1_isr
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0
    rjmp 0

.section .text

main:
    cli
    ldi r16, lo8(RAMEND)
    out SPL, r16
    ldi r16, hi8(RAMEND)
    out SPH, r16

    ldi r16, (1<<WDE)|(1<<WDP2)|(1<<WDP1)
    sts WDTCSR, r16

    ldi r16, (1<<LCD_RS)|(1<<LCD_EN)|(1<<LCD_BL)
    out LCD_DDRB, r16
    ldi r16, 0x0F
    out LCD_DDR, r16
    ldi r16, (1<<MODE_PIN)|(1<<UP_PIN)|(1<<DOWN_PIN)|(1<<SELECT_PIN)
    out PORTB, r16

    ldi r16, (1<<ISC01)|(1<<ISC11)
    sts EICRA, r16
    ldi r16, (1<<INT0)|(1<<INT1)
    out EIMSK, r16

    ldi r16, 0x00
    sts TWSR, r16
    ldi r16, 72
    sts TWBR, r16
    ldi r16, (1<<TWEN)
    sts TWCR, r16

    ldi r16, (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)
    sts ADCSRA, r16
    ldi r16, (1<<REFS0)|(ADC_PIN & 0x07)
    sts ADMUX, r16

    ldi r16, (1<<CS12)|(1<<CS10)
    sts TCCR1B, r16

    rcall lcd_init
    sbi LCD_PORTB, LCD_BL

    sei
    rcall radio_init
    rjmp main_loop

main_loop:
    WDT_RESET
    rcall check_buttons
    rcall update_display
    DELAY_MS 100
    rjmp main_loop

int0_isr:
    push r16
    in r16, SREG
    push r16
    sbis PINB, UP_PIN
    sts button_up_flag, r16
    pop r16
    out SREG, r16
    pop r16
    reti

int1_isr:
    push r16
    in r16, SREG
    push r16
    sbis PINB, DOWN_PIN
    sts button_down_flag, r16
    pop r16
    out SREG, r16
    pop r16
    reti

radio_init:
    WDT_RESET
    ldi r30, 0
    ldi r31, 0
    rcall eeprom_read_byte
    cpi r16, FIRMWARE_CHECK
    brne fatal_error
    rjmp init_continue
fatal_error:
    rcall lcd_clear
    ldi r17, lo8(str_fatal_error)
    ldi r18, hi8(str_fatal_error)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    ldi r17, lo8(str_fatal_desc)
    ldi r18, hi8(str_fatal_desc)
    rcall lcd_write_string_p
    rcall wdt_trigger_reset
init_continue:
    ldi r16, (1<<ADSC)
    sts ADCSRA, r16
adc_wait:
    lds r16, ADCSRA
    sbrs r16, ADIF
    rjmp adc_wait
    lds r16, ADCH
    cpi r16, MIN_VOLTAGE
    brsh battery_ok
    rcall lcd_clear
    ldi r17, lo8(str_low_battery)
    ldi r18, hi8(str_low_battery)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    ldi r17, lo8(str_charge_now)
    ldi r18, hi8(str_charge_now)
    rcall lcd_write_string_p
    rcall wdt_trigger_reset
battery_ok:
    rcall radio_init_fm
    rcall radio_init_am
    rcall set_volume
    ret

radio_init_fm:
    lds r19, fm_calib
    lds r16, freq_low_fm
    lds r17, freq_high_fm
    rcall apply_calibration
    ldi r20, I2C_RETRIES
radio_init_fm_retry:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne radio_init_fm_fail
    ldi r16, 0x02
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_fm_fail
    ldi r16, 0xD0
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_fm_fail
    ldi r16, 0x01
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_fm_fail
    rcall i2c_stop
    rcall timer_delay_short
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne radio_init_fm_fail
    ldi r16, 0x03
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_fm_fail
    mov r16, r17
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_fm_fail
    mov r16, r18
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_fm_fail
    rcall i2c_stop
    sts freq_low_fm, r18
    sts freq_high_fm, r17
    ret
radio_init_fm_fail:
    dec r20
    brne radio_init_fm_retry
    rjmp i2c_error

radio_init_am:
    lds r19, am_calib
    lds r16, freq_low_am
    lds r17, freq_high_am
    rcall apply_calibration
    ldi r20, I2C_RETRIES
radio_init_am_retry:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne radio_init_am_fail
    ldi r16, 0x01
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_am_fail
    ldi r16, 0x20
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_am_fail
    rcall i2c_stop
    rcall timer_delay_short
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne radio_init_am_fail
    ldi r16, 0x20
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_am_fail
    mov r16, r17
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_am_fail
    mov r16, r18
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_am_fail
    ldi r16, 0x00
    rcall i2c_write
    cpi r16, 0x00
    brne radio_init_am_fail
    rcall i2c_stop
    sts freq_low_am, r18
    sts freq_high_am, r17
    ret
radio_init_am_fail:
    dec r20
    brne radio_init_am_retry
    rjmp i2c_error

radio_check_signal:
    lds r16, last_mode
    cpi r16, 1
    breq check_signal_fm
    ldi r20, I2C_RETRIES
check_signal_am_retry:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne check_signal_am_fail
    ldi r16, 0x23
    rcall i2c_write
    cpi r16, 0x00
    brne check_signal_am_fail
    ldi r16, (AM_I2C_ADDR << 1) | 1
    rcall i2c_start
    cpi r16, 0x00
    brne check_signal_am_fail
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
    check_signal_am_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp check_signal_am_wait
    lds r16, TWDR
    rcall i2c_stop
    ret
check_signal_am_fail:
    dec r20
    brne check_signal_am_retry
    rjmp i2c_error
check_signal_fm:
    ldi r20, I2C_RETRIES
check_signal_fm_retry:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne check_signal_fm_fail
    ldi r16, 0x0A
    rcall i2c_write
    cpi r16, 0x00
    brne check_signal_fm_fail
    ldi r16, (FM_I2C_ADDR << 1) | 1
    rcall i2c_start
    cpi r16, 0x00
    brne check_signal_fm_fail
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
check_signal_fm_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp check_signal_fm_wait
    lds r16, TWDR
    rcall i2c_stop
    ret
check_signal_fm_fail:
    dec r20
    brne check_signal_fm_retry
    rjmp i2c_error

radio_get_freq_fm:
    lds r16, freq_low_fm
    lds r17, freq_high_fm
    ret

radio_get_freq_am:
    lds r16, freq_low_am
    lds r17, freq_high_am
    ret

apply_calibration:
    push r20
    push r21
    mov r20, r19
    mov r18, r16
    mov r19, r17
    ldi r21, 100
    mul r18, r20
    mov r16, r0
    mov r17, r1
    mul r19, r20
    add r17, r0
    mov r18, r16
    cpi r17, 0x03
    brsh calibration_err
    cpi r17, 0x00
    brlo calibration_err
    rjmp calibration_end
calibration_err:
    rcall lcd_clear
    ldi r17, lo8(str_calibration_error)
    ldi r18, hi8(str_calibration_error)
    rcall lcd_write_string_p
    rcall wdt_trigger_reset
calibration_end:
    pop r21
    pop r20
    ret

set_equalizer:
    lds r16, last_mode
    cpi r16, 1
    breq eq_fm
    ldi r20, I2C_RETRIES
eq_am_retry:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne eq_am_fail
    ldi r16, 0x31
    rcall i2c_write
    cpi r16, 0x00
    brne eq_am_fail
    lds r16, equalizer_mode
    cpi r16, 0
    breq eq_am_normal
    cpi r16, 1
    breq eq_am_bass
    ldi r16, 0x02
    rjmp eq_am_write
eq_am_normal:
    ldi r16, 0x00
    rjmp eq_am_write
eq_am_bass:
    ldi r16, 0x01
eq_am_write:
    rcall i2c_write
    cpi r16, 0x00
    brne eq_am_fail
    rcall i2c_stop
    ret
eq_am_fail:
    dec r20
    brne eq_am_retry
    rjmp i2c_error
eq_fm:
    ldi r20, I2C_RETRIES
eq_fm_retry:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne eq_fm_fail
    ldi r16, 0x05
    rcall i2c_write
    cpi r16, 0x00
    brne eq_fm_fail
    lds r16, equalizer_mode
    cpi r16, 0
    breq eq_fm_normal
    cpi r16, 1
    breq eq_fm_bass
    ldi r16, 0x08
    rjmp eq_fm_write
eq_fm_normal:
    ldi r16, 0x00
    rjmp eq_fm_write
eq_fm_bass:
    ldi r16, 0x04
eq_fm_write:
    rcall i2c_write
    cpi r16, 0x00
    brne eq_fm_fail
    rcall i2c_stop
    ret
eq_fm_fail:
    dec r20
    brne eq_fm_retry
    rjmp i2c_error

set_volume:
    lds r16, last_mode
    cpi r16, 1
    breq set_volume_fm
    ldi r20, I2C_RETRIES
set_volume_am_retry:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne set_volume_am_fail
    ldi r16, 0x48
    rcall i2c_write
    cpi r16, 0x00
    brne set_volume_am_fail
    lds r16, volume_level
    rcall i2c_write
    cpi r16, 0x00
    brne set_volume_am_fail
    rcall i2c_stop
    ret
set_volume_am_fail:
    dec r20
    brne set_volume_am_retry
    rjmp i2c_error
set_volume_fm:
    ldi r20, I2C_RETRIES
set_volume_fm_retry:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne set_volume_fm_fail
    ldi r16, 0x05
    rcall i2c_write
    cpi r16, 0x00
    brne set_volume_fm_fail
    lds r16, volume_level
    ori r16, 0x80
    rcall i2c_write
    cpi r16, 0x00
    brne set_volume_fm_fail
    rcall i2c_stop
    ret
set_volume_fm_fail:
    dec r20
    brne set_volume_fm_retry
    rjmp i2c_error

radio_seek:
    lds r16, last_mode
    cpi r16, 1
    breq seek_fm
    ldi r20, I2C_RETRIES
seek_am_retry:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, 0x21
    rcall i2c_write
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, 0x01
    rcall i2c_write
    cpi r16, 0x00
    brne seek_am_fail
    rcall i2c_stop
seek_am_wait:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, 0x23
    rcall i2c_write
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, (AM_I2C_ADDR << 1) | 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
seek_am_wait_loop:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp seek_am_wait_loop
    lds r16, TWDR
    cpi r16, 0x20
    brsh seek_am_found
    rcall i2c_stop
    rjmp seek_am_wait
seek_am_found:
    ldi r16, AM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, 0x22
    rcall i2c_write
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, (AM_I2C_ADDR << 1) | 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_am_fail
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp seek_am_wait_loop
    lds r18, TWDR
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp seek_am_wait_loop
    lds r17, TWDR
    rcall i2c_stop
    sts freq_low_am, r18
    sts freq_high_am, r17
    rcall radio_init_am
    ret
seek_am_fail:
    dec r20
    brne seek_am_retry
    rjmp i2c_error
seek_fm:
    ldi r20, I2C_RETRIES
seek_fm_retry:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, 0x02
    rcall i2c_write
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, 0xD1
    rcall i2c_write
    cpi r16, 0x00
    brne seek_fm_fail
    rcall i2c_stop
seek_fm_wait:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, 0x0A
    rcall i2c_write
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, (FM_I2C_ADDR << 1) | 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
seek_fm_wait_loop:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp seek_fm_wait_loop
    lds r16, TWDR
    cpi r16, 0x20
    brsh seek_fm_found
    rcall i2c_stop
    rjmp seek_fm_wait
seek_fm_found:
    ldi r16, FM_I2C_ADDR << 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, 0x0B
    rcall i2c_write
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, (FM_I2C_ADDR << 1) | 1
    rcall i2c_start
    cpi r16, 0x00
    brne seek_fm_fail
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp seek_fm_wait_loop
    lds r18, TWDR
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp seek_fm_wait_loop
    lds r17, TWDR
    rcall i2c_stop
    sts freq_low_fm, r18
    sts freq_high_fm, r17
    rcall radio_init_fm
    ret
seek_fm_fail:
    dec r20
    brne seek_fm_retry
    rjmp i2c_error

menu_move_up:
    lds r16, menu_state
    cpi r16, 0
    breq menu_up_main
    cpi r16, 2
    breq menu_up_freq
    cpi r16, 3
    breq menu_up_stations
    cpi r16, 4
    breq menu_up_eq
    cpi r16, 5
    breq menu_up_volume
    ret
menu_up_main:
    lds r16, menu_selection
    cpi r16, 0
    breq menu_up_end
    dec r16
    sts menu_selection, r16
    rcall update_display
menu_up_end:
    ret
menu_up_freq:
    lds r16, last_mode
    cpi r16, 1
    breq menu_up_fm
    lds r16, freq_low_am
    lds r17, freq_high_am
    movw r18, r16
    ldi r20, lo8(AM_MAX_FREQ)
    ldi r21, hi8(AM_MAX_FREQ)
    cp r16, r20
    cpc r17, r21
    brsh am_freq_max
    cpi r17, 0x06
    brsh am_step_sw
    ldi r20, AM_STEP_LW_MW
    rjmp am_step_add
am_step_sw:
    ldi r20, AM_STEP_SW
am_step_add:
    clr r21
    add r16, r20
    adc r17, r21
    sts freq_low_am, r16
    sts freq_high_am, r17
    rcall radio_init_am
    rjmp menu_up_update
am_freq_max:
    ldi r16, lo8(AM_MIN_FREQ)
    ldi r17, hi8(AM_MIN_FREQ)
    sts freq_low_am, r16
    sts freq_high_am, r17
    rcall radio_init_am
menu_up_update:
    rcall update_display
    ret
menu_up_fm:
    lds r16, freq_low_fm
    lds r17, freq_high_fm
    adiw r16, FM_STEP
    ldi r20, lo8(FM_MAX_FREQ)
    ldi r21, hi8(FM_MAX_FREQ)
    cp r16, r20
    cpc r17, r21
    brsh fm_freq_max
    sts freq_low_fm, r16
    sts freq_high_fm, r17
    rcall radio_init_fm
    rjmp menu_up_update
fm_freq_max:
    ldi r16, lo8(FM_MIN_FREQ)
    ldi r17, hi8(FM_MIN_FREQ)
    sts freq_low_fm, r16
    sts freq_high_fm, r17
    rcall radio_init_fm
    rjmp menu_up_update
menu_up_stations:
    lds r16, station_slot
    cpi r16, 0
    breq menu_up_end
    dec r16
    sts station_slot, r16
    rcall update_display
    ret
menu_up_eq:
    lds r16, equalizer_mode
    cpi r16, 0
    breq menu_up_end
    dec r16
    sts equalizer_mode, r16
    rcall set_equalizer
    rcall update_display
    ret
menu_up_volume:
    lds r16, volume_level
    cpi r16, VOLUME_MAX
    breq menu_up_end
    inc r16
    sts volume_level, r16
    rcall set_volume
    rcall update_display
    ret

menu_move_down:
    lds r16, menu_state
    cpi r16, 0
    breq menu_down_main
    cpi r16, 2
    breq menu_down_freq
    cpi r16, 3
    breq menu_down_stations
    cpi r16, 4
    breq menu_down_eq
    cpi r16, 5
    breq menu_down_volume
    ret
menu_down_main:
    lds r16, menu_selection
    cpi r16, 5
    breq menu_down_end
    inc r16
    sts menu_selection, r16
    rcall update_display
menu_down_end:
    ret
menu_down_freq:
    lds r16, last_mode
    cpi r16, 1
    breq menu_down_fm
    lds r16, freq_low_am
    lds r17, freq_high_am
    movw r18, r16
    ldi r20, lo8(AM_MIN_FREQ)
    ldi r21, hi8(AM_MIN_FREQ)
    cp r16, r20
    cpc r17, r21
    brlo am_freq_min
    cpi r17, 0x06
    brsh am_step_sw_down
    ldi r20, AM_STEP_LW_MW
    rjmp am_step_sub
am_step_sw_down:
    ldi r20, AM_STEP_SW
am_step_sub:
    clr r21
    sub r16, r20
    sbc r17, r21
    sts freq_low_am, r16
    sts freq_high_am, r17
    rcall radio_init_am
    rjmp menu_down_update
am_freq_min:
    ldi r16, lo8(AM_MAX_FREQ)
    ldi r17, hi8(AM_MAX_FREQ)
    sts freq_low_am, r16
    sts freq_high_am, r17
    rcall radio_init_am
menu_down_update:
    rcall update_display
    ret
menu_down_fm:
    lds r16, freq_low_fm
    lds r17, freq_high_fm
    sbiw r16, FM_STEP
    ldi r20, lo8(FM_MIN_FREQ)
    ldi r21, hi8(FM_MIN_FREQ)
    cp r20, r16
    cpc r21, r17
    brsh fm_freq_min
    sts freq_low_fm, r16
    sts freq_high_fm, r17
    rcall radio_init_fm
    rjmp menu_down_update
fm_freq_min:
    ldi r16, lo8(FM_MAX_FREQ)
    ldi r17, hi8(FM_MAX_FREQ)
    sts freq_low_fm, r16
    sts freq_high_fm, r17
    rcall radio_init_fm
    rjmp menu_down_update
menu_down_stations:
    lds r16, station_slot
    cpi r16, 4
    breq menu_down_end
    inc r16
    sts station_slot, r16
    rcall update_display
    ret
menu_down_eq:
    lds r16, equalizer_mode
    cpi r16, 2
    breq menu_down_end
    inc r16
    sts equalizer_mode, r16
    rcall set_equalizer
    rcall update_display
    ret
menu_down_volume:
    lds r16, volume_level
    cpi r16, VOLUME_MIN
    breq menu_down_end
    dec r16
    sts volume_level, r16
    rcall set_volume
    rcall update_display
    ret

save_station:
    lds r16, station_slot
    lsl r16
    mov r30, r16
    ldi r31, 0
    lds r16, last_mode
    cpi r16, 1
    breq save_station_fm
    lds r16, freq_low_am
    lds r17, freq_high_am
    rjmp save_station_write
save_station_fm:
    lds r16, freq_low_fm
    lds r17, freq_high_fm
save_station_write:
    rcall eeprom_write_byte
    adiw r30, 1
    mov r16, r17
    rcall eeprom_write_byte
    rcall update_display
    ret

load_station:
    lds r16, station_slot
    lsl r16
    mov r30, r16
    ldi r31, 0
    rcall eeprom_read_byte
    mov r18, r16
    adiw r30, 1
    rcall eeprom_read_byte
    mov r17, r16
    lds r16, last_mode
    cpi r16, 1
    breq load_station_fm
    sts freq_low_am, r18
    sts freq_high_am, r17
    rcall radio_init_am
    rjmp load_station_update
load_station_fm:
    sts freq_low_fm, r18
    sts freq_high_fm, r17
    rcall radio_init_fm
load_station_update:
    rcall update_display
    ret

eeprom_write_byte:
    sbic EECR, EEPE
    rjmp eeprom_write_byte
    out EEARL, r30
    out EEARH, r31
    out EEDR, r16
    sbi EECR, EEMPE
    sbi EECR, EEPE
    ret

eeprom_read_byte:
    sbic EECR, EEPE
    rjmp eeprom_read_byte
    out EEARL, r30
    out EEARH, r31
    sbi EECR, EERE
    in r16, EEDR
    ret

check_buttons:
    lds r16, button_up_flag
    cpi r16, 0
    breq check_down
    ldi r16, 0
    sts button_up_flag, r16
    rcall menu_move_up
check_down:
    lds r16, button_down_flag
    cpi r16, 0
    breq check_mode
    ldi r16, 0
    sts button_down_flag, r16
    rcall menu_move_down
check_mode:
    sbis PINB, MODE_PIN
    rcall mode_button_pressed
    sbis PINB, SELECT_PIN
    rcall select_button_pressed
    ret

mode_button_pressed:
    DELAY_MS DEBOUNCE_MS
    sbis PINB, MODE_PIN
    rjmp mode_change
    ret
mode_change:
    lds r16, last_mode
    cpi r16, 1
    breq set_am_mode
    ldi r16, 1
    sts last_mode, r16
    rcall radio_init_fm
    rjmp mode_update
set_am_mode:
    ldi r16, 0
    sts last_mode, r16
    rcall radio_init_am
mode_update:
    rcall update_display
    ret

select_button_pressed:
    DELAY_MS DEBOUNCE_MS
    sbis PINB, SELECT_PIN
    rjmp select_change
    ret
select_change:
    lds r16, menu_state
    cpi r16, 0
    breq enter_submenu
    cpi r16, 3
    breq select_station
    cpi r16, 5
    breq select_volume
    ldi r16, 0
    sts menu_state, r16
    rjmp select_update
enter_submenu:
    lds r16, menu_selection
    inc r16
    sts menu_state, r16
select_update:
    rcall update_display
    ret
select_station:
    rcall load_station
    ldi r16, 0
    sts menu_state, r16
    rjmp select_update
select_volume:
    rcall set_volume
    ldi r16, 0
    sts menu_state, r16
    rjmp select_update

lcd_init:
    DELAY_MS 50
    ldi r16, 0x30
    rcall lcd_send_command
    DELAY_MS 5
    ldi r16, 0x30
    rcall lcd_send_command
    DELAY_MS 1
    ldi r16, 0x30
    rcall lcd_send_command
    ldi r16, 0x20
    rcall lcd_send_command
    ldi r16, 0x28
    rcall lcd_send_command
    ldi r16, 0x08
    rcall lcd_send_command
    ldi r16, 0x01
    rcall lcd_send_command
    DELAY_MS 2
    ldi r16, 0x06
    rcall lcd_send_command
    ldi r16, 0x0C
    rcall lcd_send_command
    sbi LCD_PORTB, LCD_BL
    ret

lcd_clear:
    ldi r16, 0x01
    rcall lcd_send_command
    DELAY_MS 2
    ret

lcd_set_cursor:
    ldi r16, 0xC0
    rcall lcd_send_command
    ret

lcd_set_position:
    push r16
    ori r16, 0x80
    rcall lcd_send_command
    pop r16
    ret

lcd_send_command:
    push r16
    cbi LCD_PORTB, LCD_RS
    mov r0, r16
    swap r0
    andi r0, 0x0F
    out LCD_PORT, r0
    sbi LCD_PORTB, LCD_EN
    nop
    cbi LCD_PORTB, LCD_EN
    mov r0, r16
    andi r0, 0x0F
    out LCD_PORT, r0
    sbi LCD_PORTB, LCD_EN
    nop
    cbi LCD_PORTB, LCD_EN
    DELAY_MS 1
    pop r16
    ret

lcd_write_char:
    push r16
    sbi LCD_PORTB, LCD_RS
    mov r0, r16
    swap r0
    andi r0, 0x0F
    out LCD_PORT, r0
    sbi LCD_PORTB, LCD_EN
    nop
    cbi LCD_PORTB, LCD_EN
    mov r0, r16
    andi r0, 0x0F
    out LCD_PORT, r0
    sbi LCD_PORTB, LCD_EN
    nop
    cbi LCD_PORTB, LCD_EN
    DELAY_MS 1
    pop r16
    ret

lcd_write_string_p:
    push r16
    push r17
    push r18
    push r30
    push r31
    movw r30, r17
lcd_write_string_p_loop:
    lpm r16, Z+
    cpi r16, 0
    breq lcd_write_string_p_end
    rcall lcd_write_char
    rjmp lcd_write_string_p_loop
lcd_write_string_p_end:
    pop r31
    pop r30
    pop r18
    pop r17
    pop r16
    ret

i2c_start:
    ldi r16, (1<<TWINT)|(1<<TWSTA)|(1<<TWEN)
    sts TWCR, r16
i2c_start_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp i2c_start_wait
    lds r16, TWSR
    andi r16, 0xF8
    cpi r16, 0x08
    breq i2c_start_ok
    cpi r16, 0x10
    breq i2c_start_ok
    ldi r16, 0xFF
    ret
i2c_start_ok:
    ldi r16, 0x00
    ret

i2c_write:
    sts TWDR, r16
    ldi r16, (1<<TWINT)|(1<<TWEN)
    sts TWCR, r16
i2c_write_wait:
    lds r16, TWCR
    sbrs r16, TWINT
    rjmp i2c_write_wait
    lds r16, TWSR
    andi r16, 0xF8
    cpi r16, 0x18
    breq i2c_write_ok
    cpi r16, 0x28
    breq i2c_write_ok
    ldi r16, 0xFF
    ret
i2c_write_ok:
    ldi r16, 0x00
    ret

i2c_stop:
    ldi r16, (1<<TWINT)|(1<<TWSTO)|(1<<TWEN)
    sts TWCR, r16
    ldi r16, 0x00
    ret

delay_ms:
    push r16
    push r17
    push r18
    mov r18, r24
delay_ms_loop:
    ldi r16, lo8(8000)
    ldi r17, hi8(8000)
delay_ms_inner:
    sbiw r16, 1
    brne delay_ms_inner
    dec r18
    brne delay_ms_loop
    pop r18
    pop r17
    pop r16
    ret

timer_delay_short:
    ldi r24, 10
    rcall delay_ms
    ret

itoa:
    push r16
    push r17
    push r18
    push r19
    push r20
    push r30
    push r31
    ldi r30, lo8(freq_buffer)
    ldi r31, hi8(freq_buffer)
    movw r18, r24
    ldi r16, 0
    sts freq_buffer, r16
    ldi r20, 0
itoa_loop:
    ldi r16, 10
    clr r17
    div r18, r16
    mov r18, r0
    mov r16, r1
    add r16, '0'
    cpi r20, 2
    brne itoa_no_dot
    ldi r16, '.'
    st Z+, r16
    ldi r16, 0
    add r16, r1
    add r16, '0'
itoa_no_dot:
    st Z+, r16
    inc r20
    cpi r18, 0
    cpc r19, r17
    brne itoa_loop
    ldi r16, 0
    st Z, r16
    ldi r30, lo8(freq_buffer)
    ldi r31, hi8(freq_buffer)
    mov r16, r20
    dec r16
    add r30, r16
    ldi r16, lo8(freq_buffer)
    ldi r17, hi8(freq_buffer)
itoa_reverse:
    ld r18, Z
    ld r19, Y
    st Z, r19
    st Y, r18
    inc r16
    dec r30
    cp r16, r30
    brlo itoa_reverse
    pop r31
    pop r30
    pop r20
    pop r19
    pop r18
    pop r17
    pop r16
    ret

update_display:
    WDT_RESET
    rcall lcd_clear
    lds r16, menu_state
    cpi r16, 0
    breq display_main_menu
    cpi r16, 1
    breq display_mode
    cpi r16, 2
    breq display_freq
    cpi r16, 3
    breq display_stations
    cpi r16, 4
    breq display_eq
    cpi r16, 5
    breq display_volume
    cpi r16, 6
    breq display_status
    ret
display_main_menu:
    lds r16, menu_selection
    cpi r16, 0
    breq main_menu_mode
    cpi r16, 1
    breq main_menu_freq
    cpi r16, 2
    breq main_menu_stations
    cpi r16, 3
    breq main_menu_eq
    cpi r16, 4
    breq main_menu_volume
    ldi r17, lo8(str_menu_status)
    ldi r18, hi8(str_menu_status)
    rjmp main_menu_write
main_menu_mode:
    ldi r17, lo8(str_menu_mode)
    ldi r18, hi8(str_menu_mode)
    rjmp main_menu_write
main_menu_freq:
    ldi r17, lo8(str_menu_freq)
    ldi r18, hi8(str_menu_freq)
    rjmp main_menu_write
main_menu_stations:
    ldi r17, lo8(str_menu_stations)
    ldi r18, hi8(str_menu_stations)
    rjmp main_menu_write
main_menu_eq:
    ldi r17, lo8(str_menu_eq)
    ldi r18, hi8(str_menu_eq)
rjmp main_menu_write
main_menu_volume:
    ldi r17, lo8(str_menu_volume)
    ldi r18, hi8(str_menu_volume)
main_menu_write:
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    ldi r17, lo8(str_menu_opts)
    ldi r18, hi8(str_menu_opts)
    rcall lcd_write_string_p
    ret
display_mode:
    ldi r17, lo8(str_radiolinux)
    ldi r18, hi8(str_radiolinux)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    lds r16, last_mode
    cpi r16, 1
    breq display_fm
    ldi r17, lo8(str_am)
    ldi r18, hi8(str_am)
    rcall lcd_write_string_p
    rcall lcd_write_freq_am
    ret
display_fm:
    ldi r17, lo8(str_fm)
    ldi r18, hi8(str_fm)
    rcall lcd_write_string_p
    rcall lcd_write_freq_fm
    ret
display_freq:
    rcall lcd_set_position
    ldi r17, lo8(str_fm)
    ldi r18, hi8(str_fm)
    rcall lcd_write_string_p
    rcall lcd_write_freq_fm
    ldi r16, 0x48
    rcall lcd_set_position
    ldi r17, lo8(str_am)
    ldi r18, hi8(str_am)
    rcall lcd_write_string_p
    rcall lcd_write_freq_am_short
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    lds r16, last_mode
    cpi r16, 1
    breq display_fm_active
    ldi r17, lo8(str_am_arrow)
    ldi r18, hi8(str_am_arrow)
    rcall lcd_write_string_p
    rjmp display_am_range
display_fm_active:
    ldi r17, lo8(str_fm_arrow)
    ldi r18, hi8(str_fm_arrow)
    rcall lcd_write_string_p
display_am_range:
    lds r16, freq_low_am
    lds r17, freq_high_am
    cpi r17, 0x00
    brlo display_lw
    cpi r17, 0x06
    brlo display_mw
    ldi r17, lo8(str_sw)
    ldi r18, hi8(str_sw)
    rjmp display_am_range_write
display_lw:
    ldi r17, lo8(str_lw)
    ldi r18, hi8(str_lw)
    rjmp display_am_range_write
display_mw:
    ldi r17, lo8(str_am)
    ldi r18, hi8(str_am)
display_am_range_write:
    rcall lcd_write_string_p
    ret
display_stations:
    ldi r17, lo8(str_stations)
    ldi r18, hi8(str_stations)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    lds r16, station_slot
    cpi r16, 0
    breq station_save
    ldi r17, lo8(str_load_station)
    ldi r18, hi8(str_load_station)
    rcall lcd_write_string_p
    rcall load_station
    ret
station_save:
    ldi r17, lo8(str_save_station)
    ldi r18, hi8(str_save_station)
    rcall lcd_write_string_p
    rcall save_station
    ret
display_eq:
    ldi r17, lo8(str_equalizer)
    ldi r18, hi8(str_equalizer)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    lds r16, equalizer_mode
    cpi r16, 0
    breq eq_normal_display
    cpi r16, 1
    breq eq_bass_display
    ldi r17, lo8(str_treble)
    ldi r18, hi8(str_treble)
    rjmp eq_write_display
eq_normal_display:
    ldi r17, lo8(str_normal)
    ldi r18, hi8(str_normal)
    rjmp eq_write_display
eq_bass_display:
    ldi r17, lo8(str_bass)
    ldi r18, hi8(str_bass)
eq_write_display:
    rcall lcd_write_string_p
    ret
display_volume:
    ldi r17, lo8(str_volume)
    ldi r18, hi8(str_volume)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    lds r16, volume_level
    mov r24, r16
    ldi r22, 10
    ldi r23, 0
    rcall itoa
    ldi r17, lo8(freq_buffer)
    ldi r18, hi8(freq_buffer)
    rcall lcd_write_string_p
    ret
display_status:
    ldi r17, lo8(str_signal)
    ldi r18, hi8(str_signal)
    rcall lcd_write_string_p
    rcall lcd_set_cursor
    ldi r16, 1
    rcall lcd_write_char
    rcall radio_check_signal
    cpi r16, 0x20
    brsh display_signal_ok
    ldi r17, lo8(str_no_antenna)
    ldi r18, hi8(str_no_antenna)
    rcall lcd_write_string_p
    ret
display_signal_ok:
    ldi r16, (1<<ADSC)
    sts ADCSRA, r16
adc_wait_status:
    lds r16, ADCSRA
    sbrs r16, ADIF
    rjmp adc_wait_status
    lds r16, ADCH
    cpi r16, MIN_VOLTAGE
    brsh battery_status_ok
    ldi r17, lo8(str_low_battery)
    ldi r18, hi8(str_low_battery)
    rcall lcd_write_string_p
    ret
battery_status_ok:
    ldi r17, lo8(str_rssi)
    ldi r18, hi8(str_rssi)
    rcall lcd_write_string_p
    ret

lcd_write_freq_am_short:
    lds r16, freq_low_am
    lds r17, freq_high_am
    movw r24, r16
    ldi r22, 10
    ldi r23, 0
    rcall itoa
    ldi r17, lo8(freq_buffer)
    ldi r18, hi8(freq_buffer)
    rcall lcd_write_string_p
    ldi r17, lo8(str_khz)
    ldi r18, hi8(str_khz)
    rcall lcd_write_string_p
    ret

lcd_write_freq_fm:
    lds r16, freq_low_fm
    lds r17, freq_high_fm
    movw r24, r16
    ldi r22, 10
    ldi r23, 0
    rcall itoa
    ldi r17, lo8(freq_buffer)
    ldi r18, hi8(freq_buffer)
    rcall lcd_write_string_p
    ldi r17, lo8(str_mhz)
    ldi r18, hi8(str_mhz)
    rcall lcd_write_string_p
    ret

lcd_write_freq_am:
    lds r16, freq_low_am
    lds r17, freq_high_am
    cpi r17, 0x00
    brlo display_lw
    cpi r17, 0x06
    brlo display_mw
    ldi r17, lo8(str_sw)
    ldi r18, hi8(str_sw)
    rcall lcd_write_string_p
    rjmp display_freq_am
display_lw:
    ldi r17, lo8(str_lw)
    ldi r18, hi8(str_lw)
    rcall lcd_write_string_p
    rjmp display_freq_am
display_mw:
    ldi r17, lo8(str_am)
    ldi r18, hi8(str_am)
    rcall lcd_write_string_p
display_freq_am:
    movw r24, r16
    ldi r22, 10
    ldi r23, 0
    rcall itoa
    ldi r17, lo8(freq_buffer)
    ldi r18, hi8(freq_buffer)
    rcall lcd_write_string_p
    ldi r17, lo8(str_khz)
    ldi r18, hi8(str_khz)
    rcall lcd_write_string_p
    ret

wdt_trigger_reset:
    cli
    ldi r16, (1<<WDCE)|(1<<WDE)
    sts WDTCSR, r16
    ldi r16, (1<<WDE)
    sts WDTCSR, r16
wdt_loop:
    rjmp wdt_loop

i2c_error:
    rcall lcd_clear
    ldi r17, lo8(str_i2c_error)
    ldi r18, hi8(str_i2c_error)
    rcall lcd_write_string_p
    rcall wdt_trigger_reset

.section .data
last_mode: .byte 0
freq_low_fm: .byte lo8(FM_MIN_FREQ)
freq_high_fm: .byte hi8(FM_MIN_FREQ)
freq_low_am: .byte lo8(AM_MIN_FREQ)
freq_high_am: .byte hi8(AM_MIN_FREQ)
menu_state: .byte 0
menu_selection: .byte 0
station_slot: .byte 0
equalizer_mode: .byte 0
volume_level: .byte 8
button_up_flag: .byte 0
button_down_flag: .byte 0
freq_buffer: .space 10

.section .eeprom
firmware_flag: .byte FIRMWARE_CHECK
stations: .space 10
fm_calib: .byte 100
am_calib: .byte 100

.section .progmem
str_radiolinux: .asciz "RadioLinux"
str_fm: .asciz "FM "
str_am: .asciz "AM "
str_lw: .asciz "LW "
str_sw: .asciz "SW "
str_khz: .asciz " kHz"
str_mhz: .asciz " MHz"
str_fm_arrow: .asciz ">FM "
str_am_arrow: .asciz "  >AM"
str_menu_mode: .asciz "Menu: Mode"
str_menu_freq: .asciz "Menu: Freq"
str_menu_stations: .asciz "Menu: Stations"
str_menu_eq: .asciz "Menu: Equalizer"
str_menu_volume: .asciz "Menu: Volume"
str_menu_status: .asciz "Menu: Status"
str_menu_opts: .asciz ">Mode Freq Stat"
str_tune_freq: .asciz "Tune Freq"
str_stations: .asciz "Stations"
str_save_station: .asciz ">Save Station"
str_load_station: .asciz ">Load Station"
str_equalizer: .asciz "Equalizer"
str_normal: .asciz ">Normal"
str_bass: .asciz ">Bass"
str_treble: .asciz ">Treble"
str_volume: .asciz "Volume"
str_signal: .asciz "Signal Status"
str_rssi: .asciz "RSSI: 32 dB"
str_i2c_error: .asciz "I2C Error"
str_no_antenna: .asciz "No Antenna"
str_timer_error: .asciz "Timer Error"
str_pwm_error: .asciz "PWM Error"
str_calibration_error: .asciz "Calibration Error"
str_eeprom_error: .asciz "EEPROM Error"
str_fatal_error: .asciz "Fatal System"
str_fatal_desc: .asciz "No Firmware!"
str_low_battery: .asciz "Low Battery!"
str_charge_now: .asciz "Charge Now!"