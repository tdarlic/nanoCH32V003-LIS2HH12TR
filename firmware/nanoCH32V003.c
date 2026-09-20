/* LIS2HH12 accelerometer test firmware, controlled over real UART1
 * (TX=PD5, RX=PD6), read via the WCH-LinkE's RXD/TXD pins at 115200 baud.
 *
 * I2C wiring: SDA -> PC1, SCL -> PC2 (CH32V003 I2C1 default pins, no remap).
 *
 * INT1 wiring: sensor INT1 (breakout J2 pin 2) -> PD4 (EXTI4).
 * INT2 wiring: sensor INT2 (breakout J2 pin 3) -> PD3 (EXTI3).
 * Both driven push-pull, active-high by default, so these are real
 * electrical edge-triggered interrupts, not just I2C status polling.
 * Both pins have a storm circuit breaker: a floating/marginal
 * connection with both edges armed can oscillate fast enough to
 * re-enter the ISR forever and hang the whole board (observed in
 * practice). Past STORM_THRESHOLD edges without the main loop getting
 * a chance to run, the ISR self-disables that one line and reports
 * "tripped=1" in its PINSTATUS output plus a one-shot ERR line.
 * Re-arming (INT ON / WAKE ON / IMPACT ON) clears the trip and
 * re-enables the line, so fixing the wiring and retrying just works.
 *
 * Two product-shaped features sit on top of the generic IG1/IG2 test
 * commands, each intentionally exclusive to one physical pin:
 *   WAKE   - Activity/Inactivity function (a dedicated low-power hardware
 *            block, distinct from IG1/IG2) -> INT1 only. The chip also
 *            auto-drops to 10Hz sampling while still. No I2C status
 *            register exists for this - INT PINSTATUS (the real pin) is
 *            the only way to observe it, which is also how a real
 *            low-power product would use it (MCU asleep, wakes on the
 *            GPIO edge). PIN POLARITY IS INVERTED FROM THE INTUITIVE
 *            READING (verified with a real physical still-then-shake
 *            test, not documented anywhere): PD4 HIGH means INACTIVE/
 *            STILL, LOW means ACTIVE/MOVING. The bit is literally named
 *            INT1_INACT - it reports inactivity, not activity. A boat
 *            sitting still at the dock will correctly show this pin
 *            HIGH almost all the time; that's not a bug.
 *   IMPACT - Interrupt generator 2, all axes, HIGH direction (an impact
 *            can come from any direction) -> INT2 only.
 * WAKE and the generic "INT ON" (IG1) command both target INT1 and will
 * fight over the same pin if both armed - arming one clears the other's
 * routing.
 *
 * Line-based command protocol (send a command + '\n'):
 *   STREAM ON|OFF
 *   ODR <0-6>                              0=off 1=10Hz 2=50Hz 3=100Hz 4=200Hz 5=400Hz 6=800Hz
 *   SELFTEST
 *   FIFO MODE <mode> [thresh 0-31]         mode: BYPASS FIFO STREAM STREAM2FIFO BYPASS2STREAM BYPASS2FIFO
 *   FIFO STATUS
 *   FIFO READ
 *   INT ON <X|Y|Z|ANY> <HIGH|LOW> [thresh_mg] [dur]   (IG1 -> INT1, exploratory)
 *     HIGH: signed axis value exceeds +thresh_mg (positive-direction excursion).
 *     LOW:  |axis value| drops BELOW thresh_mg, regardless of sign - this is
 *           NOT a "large negative excursion" detector (empirically verified;
 *           not documented in the datasheet). This chip's interrupt generator
 *           is designed for 6D/4D orientation recognition, where LOW means
 *           "this axis is near zero", not "swung strongly negative".
 *   INT OFF
 *   INT STATUS
 *   INT PINSTATUS                          real INT1 pin (EXTI4) level + edge count
 *   WAKE ON [thresh_mg] [dur]              Activity/Inactivity -> INT1 (see above)
 *   WAKE OFF
 *   IMPACT ON [thresh_mg] [dur]            IG2, any axis, HIGH -> INT2 (collision)
 *   IMPACT OFF
 *   IMPACT STATUS
 *   IMPACT PINSTATUS                       real INT2 pin (EXTI3) level + edge count
 *   FS [2|4|8]                             set, or with no arg report, the full-scale range in g (default 8)
 *   PEAK                                   report the highest |g| seen since the last reset
 *   PEAK RESET
 *   HELP
 *
 * Output lines: ACC,x,y,z,temp_mC | SELFTEST,axis,normal,st,diff,PASS|FAIL | SELFTEST,DONE |
 *   FIFOSTATUS,fss=..,empty=..,ovr=..,fth=.. | FIFOSAMPLE,x,y,z | FIFOREAD,DONE |
 *   INTSTATUS,ia=..,.. | IMPACTSTATUS,ia=..,.. | INTEVENT,0xNN | IMPACTEVENT,0xNN | PEAK,mg |
 *   INTPINSTATUS,level=..,edges=..,tripped=.. | INTPIN,level=..,edges=..,tripped=.. (spontaneous) |
 *   IMPACTPINSTATUS,level=..,edges=..,tripped=.. | IMPACTPIN,level=..,edges=..,tripped=.. (spontaneous) |
 *   OK[,..] | ERR,..
 *
 * temp_mC is the embedded temperature sensor in milli-degC (relative
 * sensor for thermal-drift compensation, not a calibrated absolute reading).
 */
#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "lis2hh12.h"

#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)

uint8_t accel_addr;
volatile uint8_t streaming = 1;
volatile uint32_t systick_millis;
uint8_t int_armed = 0;
uint8_t impact_armed = 0;
// Default to +-8g: a boat tracker needs headroom to catch real collisions
// without the sensor clipping (at +-2g anything over 2g would just
// saturate and never register as a bigger hit).
int32_t mg_per_lsb = LIS2HH12_MG_PER_LSB_8G;
int32_t fs_mg = 8000; // current full-scale range in mg (2000/4000/8000)
uint32_t peak_mg = 0;

// Real electrical INT1 pin (PD4, EXTI4) and INT2 pin (PD3, EXTI3).
// Updated from the ISR on every edge; the main loop just reads these,
// no I2C access from the ISR. Both lines share one NVIC vector on this
// MCU (EXTI7_0_IRQn covers lines 0-7).
volatile uint8_t int_pin_level = 0;
volatile uint32_t int_pin_edges = 0;
volatile uint8_t int2_pin_level = 0;
volatile uint32_t int2_pin_edges = 0;

// Storm circuit breaker: a floating/marginal connection with both edges
// armed can oscillate fast enough to re-enter the ISR on every return,
// starving the main loop forever (observed in practice - looks exactly
// like a dead board, no serial output at all). Each EXTI line has a
// burst counter that the ISR increments and the main loop resets every
// pass; if the main loop never gets to run between edges, the counter
// keeps climbing and the ISR self-disables that one line once it crosses
// the threshold, instead of hanging the whole board.
#define STORM_THRESHOLD 200
volatile uint32_t int_storm_count = 0;
volatile uint32_t int2_storm_count = 0;
volatile uint8_t int_storm_tripped = 0;
volatile uint8_t int2_storm_tripped = 0;

static void exti_init(void)
{
	// PD4 (INT1), PD3 (INT2): pull-down inputs. The sensor drives both
	// push-pull when actually connected and configured (easily overrides
	// a weak pull), but a bare floating input left both edges armed with
	// nothing to hold the line while disconnected/undriven - any noise
	// pickup free-runs the ISR and starves the main loop. The pull-down
	// is a safety net against exactly that, whatever the root cause.
	funPinMode(PD4, GPIO_CFGLR_IN_PUPD);
	funDigitalWrite(PD4, FUN_LOW);
	funPinMode(PD3, GPIO_CFGLR_IN_PUPD);
	funDigitalWrite(PD3, FUN_LOW);

	AFIO->EXTICR = (AFIO->EXTICR & ~AFIO_EXTICR_EXTI4) | AFIO_EXTICR_EXTI4_PD;
	AFIO->EXTICR = (AFIO->EXTICR & ~AFIO_EXTICR_EXTI3) | AFIO_EXTICR_EXTI3_PD;
	EXTI->INTENR |= EXTI_INTENR_MR4 | EXTI_INTENR_MR3;
	EXTI->RTENR |= EXTI_RTENR_TR4 | EXTI_RTENR_TR3; // rising edge
	EXTI->FTENR |= EXTI_FTENR_TR4 | EXTI_FTENR_TR3; // falling edge
	NVIC_EnableIRQ(EXTI7_0_IRQn);
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void EXTI7_0_IRQHandler(void)
{
	if (EXTI->INTFR & EXTI_INTF_INTF4)
	{
		EXTI->INTFR = EXTI_INTF_INTF4; // write 1 to clear
		int_pin_level = (uint8_t)funDigitalRead(PD4);
		int_pin_edges++;
		if (++int_storm_count > STORM_THRESHOLD)
		{
			EXTI->INTENR &= ~EXTI_INTENR_MR4; // self-disable this line only
			int_storm_tripped = 1;
		}
	}
	if (EXTI->INTFR & EXTI_INTF_INTF3)
	{
		EXTI->INTFR = EXTI_INTF_INTF3;
		int2_pin_level = (uint8_t)funDigitalRead(PD3);
		int2_pin_edges++;
		if (++int2_storm_count > STORM_THRESHOLD)
		{
			EXTI->INTENR &= ~EXTI_INTENR_MR3;
			int2_storm_tripped = 1;
		}
	}
}

// Called from the "ON" commands below: clears a tripped breaker and
// re-enables the line, so fixing the wiring and re-arming just works.
static void exti_rearm1(void)
{
	int_storm_tripped = 0;
	int_storm_count = 0;
	EXTI->INTFR = EXTI_INTF_INTF4; // clear any stale pending flag first
	EXTI->INTENR |= EXTI_INTENR_MR4;
}

static void exti_rearm2(void)
{
	int2_storm_tripped = 0;
	int2_storm_count = 0;
	EXTI->INTFR = EXTI_INTF_INTF3;
	EXTI->INTENR |= EXTI_INTENR_MR3;
}

// Integer sqrt (binary digit-by-digit method) - no libm with -nostdlib.
static uint32_t isqrt32(uint32_t n)
{
	uint32_t res = 0;
	uint32_t bit = 1UL << 30;
	while (bit > n) bit >>= 2;
	while (bit != 0)
	{
		if (n >= res + bit)
		{
			n -= res + bit;
			res = (res >> 1) + bit;
		}
		else
		{
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

static void systick_init(void)
{
	SysTick->CTLR = 0;
	SysTick->CMP = SYSTICK_ONE_MILLISECOND - 1;
	SysTick->CNT = 0;
	systick_millis = 0;
	SysTick->CTLR |= SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
	NVIC_EnableIRQ(SysTick_IRQn);
}

void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	SysTick->CMP += SYSTICK_ONE_MILLISECOND;
	SysTick->SR = 0;
	systick_millis++;
}

// USART1's RX holding register is a single byte with no FIFO. printf()
// busy-waits on TX per byte (~1-2ms per streamed ACC line), and if a new
// RX byte arrives during that window before the main loop gets back to
// polling, it's silently lost. An RX-interrupt ring buffer decouples
// reception from whatever the main loop happens to be blocked on.
#define RX_RING_SIZE 32
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head = 0, rx_tail = 0;

void USART1_IRQHandler(void) __attribute__((interrupt));
void USART1_IRQHandler(void)
{
	if (USART1->STATR & USART_STATR_RXNE)
	{
		uint8_t b = (uint8_t)USART1->DATAR;
		uint8_t next = (uint8_t)((rx_head + 1) % RX_RING_SIZE);
		if (next != rx_tail)
		{
			rx_ring[rx_head] = b;
			rx_head = next;
		}
	}
}

// Non-blocking: returns -1 if no byte is waiting.
static int uart_getchar(void)
{
	if (rx_head == rx_tail)
		return -1;
	uint8_t b = rx_ring[rx_tail];
	rx_tail = (uint8_t)((rx_tail + 1) % RX_RING_SIZE);
	return b;
}

static int parse_int(const char *s)
{
	int neg = 0, v = 0;
	if (!s) return 0;
	while (*s == ' ') s++;
	if (*s == '-') { neg = 1; s++; }
	while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
	return neg ? -v : v;
}

static char *next_token(char **s)
{
	char *p = *s;
	while (*p == ' ') p++;
	if (!*p) { *s = p; return 0; }
	char *start = p;
	while (*p && *p != ' ') p++;
	if (*p) { *p = 0; p++; }
	*s = p;
	return start;
}

static void run_selftest(uint8_t addr)
{
	int16_t x, y, z;
	int32_t sx, sy, sz;
	int32_t normal_mg[3], st_mg[3];
	const int N = 8;
	static const char *names[3] = {"X", "Y", "Z"};

	lis2hh12_write_reg(addr, LIS2HH12_CTRL5, LIS2HH12_ST_NORMAL);
	Delay_Ms(100);
	lis2hh12_read_xyz(addr, &x, &y, &z); // discard first sample after settling
	sx = sy = sz = 0;
	for (int i = 0; i < N; i++)
	{
		Delay_Ms(20);
		lis2hh12_read_xyz(addr, &x, &y, &z);
		sx += x; sy += y; sz += z;
	}
	normal_mg[0] = lis2hh12_to_mg((int16_t)(sx / N), mg_per_lsb);
	normal_mg[1] = lis2hh12_to_mg((int16_t)(sy / N), mg_per_lsb);
	normal_mg[2] = lis2hh12_to_mg((int16_t)(sz / N), mg_per_lsb);

	lis2hh12_write_reg(addr, LIS2HH12_CTRL5, LIS2HH12_ST_POSITIVE);
	Delay_Ms(100);
	lis2hh12_read_xyz(addr, &x, &y, &z);
	sx = sy = sz = 0;
	for (int i = 0; i < N; i++)
	{
		Delay_Ms(20);
		lis2hh12_read_xyz(addr, &x, &y, &z);
		sx += x; sy += y; sz += z;
	}
	st_mg[0] = lis2hh12_to_mg((int16_t)(sx / N), mg_per_lsb);
	st_mg[1] = lis2hh12_to_mg((int16_t)(sy / N), mg_per_lsb);
	st_mg[2] = lis2hh12_to_mg((int16_t)(sz / N), mg_per_lsb);

	lis2hh12_write_reg(addr, LIS2HH12_CTRL5, LIS2HH12_ST_NORMAL);

	for (int i = 0; i < 3; i++)
	{
		int32_t diff = st_mg[i] - normal_mg[i];
		int32_t adiff = diff < 0 ? -diff : diff;
		int pass = (adiff >= 70 && adiff <= 1500); // datasheet spec, Table 3
		printf("SELFTEST,%s,%ld,%ld,%ld,%s\n", names[i],
			(long)normal_mg[i], (long)st_mg[i], (long)diff, pass ? "PASS" : "FAIL");
	}
	printf("SELFTEST,DONE\n");
}

static void fifo_print_status(uint8_t addr)
{
	uint8_t src = 0;
	lis2hh12_read_reg(addr, LIS2HH12_FIFO_SRC, &src);
	printf("FIFOSTATUS,fss=%d,empty=%d,ovr=%d,fth=%d\n",
		src & LIS2HH12_FIFO_SRC_FSS,
		(src & LIS2HH12_FIFO_SRC_EMPTY) ? 1 : 0,
		(src & LIS2HH12_FIFO_SRC_OVR) ? 1 : 0,
		(src & LIS2HH12_FIFO_SRC_FTH) ? 1 : 0);
}

static void fifo_read_all(uint8_t addr)
{
	for (int i = 0; i < 32; i++)
	{
		uint8_t src = 0;
		lis2hh12_read_reg(addr, LIS2HH12_FIFO_SRC, &src);
		if (src & LIS2HH12_FIFO_SRC_EMPTY)
			break;

		int16_t x, y, z;
		if (!lis2hh12_read_xyz(addr, &x, &y, &z))
			break;
		printf("FIFOSAMPLE,%ld,%ld,%ld\n",
			(long)lis2hh12_to_mg(x, mg_per_lsb), (long)lis2hh12_to_mg(y, mg_per_lsb), (long)lis2hh12_to_mg(z, mg_per_lsb));
	}
	printf("FIFOREAD,DONE\n");
}

static void handle_line(char *line)
{
	char *rest = line;
	char *cmd = next_token(&rest);
	if (!cmd)
		return;

	if (!strcmp(cmd, "STREAM"))
	{
		char *arg = next_token(&rest);
		if (arg && !strcmp(arg, "ON")) { streaming = 1; printf("OK\n"); }
		else if (arg && !strcmp(arg, "OFF")) { streaming = 0; printf("OK\n"); }
		else printf("ERR,usage: STREAM ON|OFF\n");
	}
	else if (!strcmp(cmd, "ODR"))
	{
		int n = parse_int(next_token(&rest));
		if (n < 0 || n > 6) { printf("ERR,usage: ODR 0-6\n"); return; }
		lis2hh12_set_odr(accel_addr, (uint8_t)n);
		printf("OK,ODR,%d\n", n);
	}
	else if (!strcmp(cmd, "SELFTEST"))
	{
		run_selftest(accel_addr);
	}
	else if (!strcmp(cmd, "FIFO"))
	{
		char *sub = next_token(&rest);
		if (sub && !strcmp(sub, "MODE"))
		{
			char *modestr = next_token(&rest);
			char *threshstr = next_token(&rest);
			uint8_t fmode;
			if (!modestr) { printf("ERR,usage: FIFO MODE <mode> [thresh]\n"); return; }
			if (!strcmp(modestr, "BYPASS")) fmode = LIS2HH12_FIFO_BYPASS;
			else if (!strcmp(modestr, "FIFO")) fmode = LIS2HH12_FIFO_FIFO;
			else if (!strcmp(modestr, "STREAM")) fmode = LIS2HH12_FIFO_STREAM;
			else if (!strcmp(modestr, "STREAM2FIFO")) fmode = LIS2HH12_FIFO_STREAM_TO_FIFO;
			else if (!strcmp(modestr, "BYPASS2STREAM")) fmode = LIS2HH12_FIFO_BYPASS_TO_STREAM;
			else if (!strcmp(modestr, "BYPASS2FIFO")) fmode = LIS2HH12_FIFO_BYPASS_TO_FIFO;
			else { printf("ERR,unknown fifo mode %s\n", modestr); return; }
			int thresh = threshstr ? parse_int(threshstr) : 0;
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL3, LIS2HH12_CTRL3_FIFO_EN);
			lis2hh12_write_reg(accel_addr, LIS2HH12_FIFO_CTRL, fmode | (thresh & 0x1F));
			printf("OK,FIFO,%s,%d\n", modestr, thresh);
		}
		else if (sub && !strcmp(sub, "STATUS")) fifo_print_status(accel_addr);
		else if (sub && !strcmp(sub, "READ")) fifo_read_all(accel_addr);
		else printf("ERR,usage: FIFO MODE|STATUS|READ\n");
	}
	else if (!strcmp(cmd, "INT"))
	{
		char *sub = next_token(&rest);
		if (sub && !strcmp(sub, "ON"))
		{
			char *axis = next_token(&rest);
			char *dir = next_token(&rest);
			char *thstr = next_token(&rest);
			char *durstr = next_token(&rest);
			if (!axis || !dir) { printf("ERR,usage: INT ON <X|Y|Z|ANY> <HIGH|LOW> [mg] [dur]\n"); return; }
			int thresh_mg = thstr ? parse_int(thstr) : 500;
			int dur = durstr ? parse_int(durstr) : 0;
			int high = !strcmp(dir, "HIGH");
			uint8_t cfg = 0;
			if (!strcmp(axis, "X") || !strcmp(axis, "ANY")) cfg |= high ? LIS2HH12_IG_XHIE : LIS2HH12_IG_XLIE;
			if (!strcmp(axis, "Y") || !strcmp(axis, "ANY")) cfg |= high ? LIS2HH12_IG_YHIE : LIS2HH12_IG_YLIE;
			if (!strcmp(axis, "Z") || !strcmp(axis, "ANY")) cfg |= high ? LIS2HH12_IG_ZHIE : LIS2HH12_IG_ZLIE;
			uint8_t ths = lis2hh12_ig_ths_from_mg(thresh_mg, fs_mg);
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_THS_X1, ths);
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_THS_Y1, ths);
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_THS_Z1, ths);
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_DUR1, (uint8_t)(dur & 0x7F));
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_CFG1, cfg);
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL3, LIS2HH12_CTRL3_INT1_IG1);
			int_armed = 1;
			exti_rearm1();
			printf("OK,INT,ON,%s,%s,%d,%d\n", axis, dir, thresh_mg, dur);
		}
		else if (sub && !strcmp(sub, "OFF"))
		{
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_CFG1, 0);
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL3, 0);
			int_armed = 0;
			printf("OK,INT,OFF\n");
		}
		else if (sub && !strcmp(sub, "STATUS"))
		{
			uint8_t src = 0;
			lis2hh12_read_reg(accel_addr, LIS2HH12_IG_SRC1, &src);
			printf("INTSTATUS,ia=%d,xh=%d,xl=%d,yh=%d,yl=%d,zh=%d,zl=%d\n",
				(src & LIS2HH12_IG_SRC_IA) ? 1 : 0, (src & LIS2HH12_IG_SRC_XH) ? 1 : 0,
				(src & LIS2HH12_IG_SRC_XL) ? 1 : 0, (src & LIS2HH12_IG_SRC_YH) ? 1 : 0,
				(src & LIS2HH12_IG_SRC_YL) ? 1 : 0, (src & LIS2HH12_IG_SRC_ZH) ? 1 : 0,
				(src & LIS2HH12_IG_SRC_ZL) ? 1 : 0);
		}
		else if (sub && !strcmp(sub, "PINSTATUS"))
		{
			printf("INTPINSTATUS,level=%d,edges=%lu,tripped=%d\n",
				int_pin_level, (unsigned long)int_pin_edges, int_storm_tripped);
		}
		else printf("ERR,usage: INT ON|OFF|STATUS|PINSTATUS\n");
	}
	else if (!strcmp(cmd, "WAKE"))
	{
		char *sub = next_token(&rest);
		if (sub && !strcmp(sub, "ON"))
		{
			char *thstr = next_token(&rest);
			char *durstr = next_token(&rest);
			int thresh_mg = thstr ? parse_int(thstr) : 100;
			int dur = durstr ? parse_int(durstr) : 0;
			uint8_t ths = lis2hh12_act_ths_from_mg(thresh_mg, fs_mg);
			lis2hh12_write_reg(accel_addr, LIS2HH12_ACT_THS, ths);
			lis2hh12_write_reg(accel_addr, LIS2HH12_ACT_DUR, (uint8_t)(dur & 0x7F));
			// Route only Activity/Inactivity to INT1 - clears any generic
			// IG1 routing a prior "INT ON" left there, since they'd
			// otherwise share (and contend for) the same physical pin.
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL3, LIS2HH12_CTRL3_INT1_INACT);
			int_armed = 0;
			exti_rearm1();
			printf("OK,WAKE,ON,%d,%d\n", thresh_mg, dur);
		}
		else if (sub && !strcmp(sub, "OFF"))
		{
			lis2hh12_write_reg(accel_addr, LIS2HH12_ACT_THS, 0); // 0 disables the feature (datasheet)
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL3, 0);
			printf("OK,WAKE,OFF\n");
		}
		else printf("ERR,usage: WAKE ON [mg] [dur] | WAKE OFF "
			"(no I2C status register for this - use INT PINSTATUS, the real INT1 pin)\n");
	}
	else if (!strcmp(cmd, "IMPACT"))
	{
		char *sub = next_token(&rest);
		if (sub && !strcmp(sub, "ON"))
		{
			char *thstr = next_token(&rest);
			char *durstr = next_token(&rest);
			int thresh_mg = thstr ? parse_int(thstr) : 2000;
			int dur = durstr ? parse_int(durstr) : 0;
			uint8_t ths = lis2hh12_ig_ths_from_mg(thresh_mg, fs_mg);
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_THS2, ths);
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_DUR2, (uint8_t)(dur & 0x7F));
			// Any axis, any strong positive-direction hit - a collision
			// can come from any direction.
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_CFG2,
				LIS2HH12_IG_XHIE | LIS2HH12_IG_YHIE | LIS2HH12_IG_ZHIE);
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL6, LIS2HH12_CTRL6_INT2_IG2);
			impact_armed = 1;
			exti_rearm2();
			printf("OK,IMPACT,ON,%d,%d\n", thresh_mg, dur);
		}
		else if (sub && !strcmp(sub, "OFF"))
		{
			lis2hh12_write_reg(accel_addr, LIS2HH12_IG_CFG2, 0);
			lis2hh12_write_reg(accel_addr, LIS2HH12_CTRL6, 0);
			impact_armed = 0;
			printf("OK,IMPACT,OFF\n");
		}
		else if (sub && !strcmp(sub, "STATUS"))
		{
			uint8_t src = 0;
			lis2hh12_read_reg(accel_addr, LIS2HH12_IG_SRC2, &src);
			printf("IMPACTSTATUS,ia=%d,xh=%d,xl=%d,yh=%d,yl=%d,zh=%d,zl=%d\n",
				(src & LIS2HH12_IG_SRC_IA) ? 1 : 0, (src & LIS2HH12_IG_SRC_XH) ? 1 : 0,
				(src & LIS2HH12_IG_SRC_XL) ? 1 : 0, (src & LIS2HH12_IG_SRC_YH) ? 1 : 0,
				(src & LIS2HH12_IG_SRC_YL) ? 1 : 0, (src & LIS2HH12_IG_SRC_ZH) ? 1 : 0,
				(src & LIS2HH12_IG_SRC_ZL) ? 1 : 0);
		}
		else if (sub && !strcmp(sub, "PINSTATUS"))
		{
			printf("IMPACTPINSTATUS,level=%d,edges=%lu,tripped=%d\n",
				int2_pin_level, (unsigned long)int2_pin_edges, int2_storm_tripped);
		}
		else printf("ERR,usage: IMPACT ON|OFF|STATUS|PINSTATUS\n");
	}
	else if (!strcmp(cmd, "FS"))
	{
		char *arg = next_token(&rest);
		if (!arg)
		{
			printf("FS,%ld\n", (long)(fs_mg / 1000));
			return;
		}
		int g = parse_int(arg);
		int32_t new_scale = lis2hh12_set_scale(accel_addr, g);
		if (!new_scale) { printf("ERR,usage: FS [2|4|8]\n"); return; }
		mg_per_lsb = new_scale;
		fs_mg = g * 1000;
		printf("OK,FS,%d\n", g);
	}
	else if (!strcmp(cmd, "PEAK"))
	{
		char *sub = next_token(&rest);
		if (sub && !strcmp(sub, "RESET"))
		{
			peak_mg = 0;
			printf("OK,PEAK,RESET\n");
		}
		else
		{
			printf("PEAK,%lu\n", (unsigned long)peak_mg);
		}
	}
	else if (!strcmp(cmd, "HELP"))
	{
		printf("Commands: STREAM ON|OFF | ODR 0-6 | SELFTEST | "
			"FIFO MODE <m> [th] | FIFO STATUS | FIFO READ | "
			"INT ON <axis> <dir> [mg] [dur] | INT OFF | INT STATUS | INT PINSTATUS | "
			"WAKE ON [mg] [dur] | WAKE OFF | "
			"IMPACT ON [mg] [dur] | IMPACT OFF | IMPACT STATUS | IMPACT PINSTATUS | "
			"FS [2|4|8] | PEAK | PEAK RESET\n");
	}
	else
	{
		printf("ERR,unknown command %s\n", cmd);
	}
}

int main()
{
	SystemInit();
	funGpioInitAll();
	systick_init();

	// SetupUART() (called from SystemInit with FUNCONF_USE_UARTPRINTF) only
	// wires up TX - it's meant for one-way debug logging. Turn on the
	// receiver too so we can read commands back on PD6.
	funPinMode(PD6, GPIO_CFGLR_IN_PUPD);
	funDigitalWrite(PD6, FUN_HIGH); // pull-up
	USART1->CTLR1 |= USART_CTLR1_RE | USART_CTLR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);

	// I2C1 pins as alternate-function open-drain (required for I2C)
	funPinMode(PC1, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SDA
	funPinMode(PC2, GPIO_CFGLR_OUT_10Mhz_AF_OD); // SCL

	exti_init(); // PD4 <- sensor INT1, PD3 <- sensor INT2

	i2c_init(I2C1, FUNCONF_SYSTEM_CORE_CLOCK, 100000);
	Delay_Ms(100);

	if (!lis2hh12_find(&accel_addr))
	{
		printf("LIS2HH12 not found - check wiring / address\n");
		while (1)
			Delay_Ms(1000);
	}
	printf("LIS2HH12 found at 0x%02X\n", accel_addr);
	lis2hh12_init(accel_addr);
	lis2hh12_set_scale(accel_addr, 8); // matches the mg_per_lsb/fs_mg defaults above
	printf("Ready. FS=+-8g. Send HELP for commands.\n");

	char cmdbuf[48];
	uint8_t cmdlen = 0;
	uint32_t last_cmd_byte_ms = 0;
	uint32_t last_stream_ms = 0;
	uint32_t last_reported_pin_edges = 0;
	uint32_t last_reported_pin2_edges = 0;
	const uint32_t stream_period_ms = 50;
	const uint32_t cmd_idle_timeout_ms = 500;

	while (1)
	{
		// Proof the main loop is still running: reset both storm
		// counters every pass. Only a sustained edge rate that never
		// lets the loop come back around keeps climbing past the
		// threshold in the ISR.
		int_storm_count = 0;
		int2_storm_count = 0;

		int c;
		while ((c = uart_getchar()) >= 0)
		{
			last_cmd_byte_ms = systick_millis;
			if (c == '\n' || c == '\r')
			{
				if (cmdlen > 0)
				{
					cmdbuf[cmdlen] = 0;
					handle_line(cmdbuf);
					cmdlen = 0;
				}
			}
			else if (cmdlen < sizeof(cmdbuf) - 1)
			{
				cmdbuf[cmdlen++] = (char)c;
			}
			else
			{
				// Line too long / desynced - drop it rather than feed
				// a truncated command to the parser.
				cmdlen = 0;
			}
		}

		// A dropped byte (no flow control on this link) can otherwise
		// wedge the parser with a partial line forever. Self-heal.
		if (cmdlen > 0 && TimeElapsed32u(systick_millis, last_cmd_byte_ms) > cmd_idle_timeout_ms)
			cmdlen = 0;

		if (int_armed)
		{
			uint8_t src = 0;
			if (!lis2hh12_read_reg(accel_addr, LIS2HH12_IG_SRC1, &src) && (src & LIS2HH12_IG_SRC_IA))
				printf("INTEVENT,0x%02X\n", src);
		}

		if (impact_armed)
		{
			uint8_t src = 0;
			if (!lis2hh12_read_reg(accel_addr, LIS2HH12_IG_SRC2, &src) && (src & LIS2HH12_IG_SRC_IA))
				printf("IMPACTEVENT,0x%02X\n", src);
		}

		// Real electrical pins: report spontaneously whenever the ISR has
		// seen a new edge, independent of the I2C polling above.
		uint32_t edges_now = int_pin_edges;
		if (edges_now != last_reported_pin_edges)
		{
			last_reported_pin_edges = edges_now;
			printf("INTPIN,level=%d,edges=%lu,tripped=%d\n",
				int_pin_level, (unsigned long)edges_now, int_storm_tripped);
			if (int_storm_tripped)
				printf("ERR,INT1 (PD4/EXTI4) edge storm - disabled to protect the board. "
					"Check the wiring, then re-arm (INT ON / WAKE ON) to retry.\n");
		}
		uint32_t edges2_now = int2_pin_edges;
		if (edges2_now != last_reported_pin2_edges)
		{
			last_reported_pin2_edges = edges2_now;
			printf("IMPACTPIN,level=%d,edges=%lu,tripped=%d\n",
				int2_pin_level, (unsigned long)edges2_now, int2_storm_tripped);
			if (int2_storm_tripped)
				printf("ERR,INT2 (PD3/EXTI3) edge storm - disabled to protect the board. "
					"Check the wiring, then re-arm (IMPACT ON) to retry.\n");
		}

		uint32_t now = systick_millis;
		if (TimeElapsed32u(now, last_stream_ms) >= stream_period_ms)
		{
			last_stream_ms = now;
			int16_t x, y, z;
			if (lis2hh12_read_xyz(accel_addr, &x, &y, &z))
			{
				int32_t mgx = lis2hh12_to_mg(x, mg_per_lsb);
				int32_t mgy = lis2hh12_to_mg(y, mg_per_lsb);
				int32_t mgz = lis2hh12_to_mg(z, mg_per_lsb);

				// Peak-hold tracks every sample, independent of STREAM ON/OFF.
				uint32_t mag_sq = (uint32_t)(mgx * mgx) + (uint32_t)(mgy * mgy) + (uint32_t)(mgz * mgz);
				uint32_t mag = isqrt32(mag_sq);
				if (mag > peak_mg)
					peak_mg = mag;

				if (streaming)
				{
					int16_t traw = 0;
					int32_t temp_mc = 0;
					if (lis2hh12_read_temp_raw(accel_addr, &traw))
						temp_mc = lis2hh12_temp_to_mc(traw);
					printf("ACC,%ld,%ld,%ld,%ld\n", (long)mgx, (long)mgy, (long)mgz, (long)temp_mc);
				}
			}
		}
	}
}
