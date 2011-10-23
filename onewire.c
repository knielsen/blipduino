/* One-wire, temperature sensor. */

#include <util/delay.h>
#include <avr/sleep.h>
#include <arduino/timer1.h>

static void serial_puts(const char *str);

#define OW_PIN 9

static uint8_t ev_timer= 0;

static void
ow_delay_us(uint16_t usec)
{
  timer1_count_set(0);
  /* d8 with 16 MHz -> 0.5 usec resolution. */
  /* Thus max. wait 65535*0.5usec = 32.768 msec */
  timer1_compare_a_set(2*usec);
  timer1_clock_d8();
}

static void
ow_delay_ms(uint16_t msec)
{
  /* For now, assume the timer is already stopped. */
  timer1_count_set(0);
  /* d1024 with 16 MHz -> 64 usec resolution. */
  /* Thus max. wait 65535*64usec = 4.194 sec */
  timer1_compare_a_set(((uint32_t)msec*1000 + 63)/64);
  timer1_clock_d1024();
}

timer1_interrupt_a()
{
  timer1_clock_off();
  ev_timer = 1;
}

static void
ow_timer_init(void)
{
  timer1_mode_normal();
  timer1_clock_off();
  timer1_count_set(0);
  timer1_compare_a_set(0xffff);
  timer1_interrupt_a_enable();
}

static char * sprint_uint16_b10(char *p, uint16_t n);
static int8_t next_onewire(void);

static void
handle_timer(void)
{
  ev_timer = 0;
  next_onewire();
}

static void
ow_release(void)
{
  pin_high(OW_PIN);
  /* Enable internal pullup. */
  pin_mode_input(OW_PIN);
}

static void
ow_low(void)
{
  pin_mode_output(OW_PIN);
  pin_low(OW_PIN);
}

static uint8_t
ow_read(void)
{
  return !!pin_is_high(OW_PIN);
}

struct onewire_cmd {
  int8_t (*cmd)(uint8_t);
  uint8_t count;
};
static const struct onewire_cmd *cmd_list;
static uint8_t cmd_index, cmd_count;

/* Run next step; return 0 if still running, 1 when done, -1 if error. */
static int8_t
next_onewire(void)
{
  int8_t err;
  err= (*cmd_list[cmd_index].cmd)(cmd_count);
  if (err)
    return -1;
  cmd_count++;
  if (cmd_count >= cmd_list[cmd_index].count)
  {
    cmd_count= 0;
    cmd_index++;
    if (!cmd_list[cmd_index].cmd)
      return 1;
  }
  return 0;
}

static void
reset_cmds(const struct onewire_cmd *cmds)
{
  cmd_list= cmds;
  cmd_index= 0;
  cmd_count= 0;
}

/* Start a list of commands; return 0 on ok, -1 on error. */
static int8_t
start_cmds(const struct onewire_cmd *cmds)
{
  reset_cmds(cmds);
  return next_onewire();
}

static uint8_t ow_presence= 0xff;
#define OW_INIT {ow_init, 3}
static int8_t
ow_init(uint8_t i)
{
  if (i == 0)
  {
    /* Reset pulse: low for >= 480 usec. */
    ow_low();
    ow_delay_us(480);
  }
  else if (i == 1)
  {
    /* Presence detect 60 usec <= T <= 240 usec. */
    ow_release();
    ow_delay_us(60);
  }
  else
  {
    /* Total presence pulse 480 usec. */
    ow_presence= !ow_read();
    ow_delay_us(480-60);
  }
  return 0;
}

static int8_t
ow_write_bit(uint8_t bit)
{
  ow_release();
  /* Min. 1 usec recovery between slots. */
  _delay_us(1);
  if (bit)
  {
    /* Write 1: release bus within 1 usec <= T <= 15 usec. */
    /* Let's make that 2 usec just to be a bit on the safe side. */
    cli();
    ow_low();
    _delay_us(2);
    ow_release();
    sei();
    /* Total write pulse >= 60 usec. */
    ow_delay_us(60 - 2);
  }
  else
  {
    /* Write 0: pull low for 60 usec <= T <= 120 usec. */
    ow_low();
    ow_delay_us(60);
  }
  return 0;
}

#define OW_SKIP_ROM {ow_skip_rom, 8}
static int8_t
ow_skip_rom(uint8_t i)
{
  return ow_write_bit(0xcc & (1 << i));
}

#define OW_READ_SCRATCH {ow_read_scratch, 8}
static int8_t
ow_read_scratch(uint8_t i)
{
  return ow_write_bit(0xbe & (1 << i));
}

#define OW_CONVERT_T {ow_convert_t, 9}
static int8_t
ow_convert_t(uint8_t i)
{
  if (i < 8)
    return ow_write_bit(0x44 & (1 << i));
  else
  {
    /* Temperature conversion takes max 750 msec. */
    ow_release();
    ow_delay_ms(750);
    return 0;
  }
}

#define OW_READ_N(n) {ow_read_bit, (n)*8}
uint8_t ow_read_buf[9];
static int8_t
ow_read_bit(uint8_t i)
{
  uint8_t bit;
  /* >= 1usec recovery time. */
  ow_release();
  _delay_us(1);
  /* >= 1 usec pull low to start read slot. */
  /* The read slot is time critical to a few usec, so disable interrupts. */
  cli();
  ow_low();
  _delay_us(1);
  ow_release();
  /*
    We must read the bus within at most 15 usec from pulling the bus low.
    The later we read, the more margin. But let's keep a couple usec to account
    for delays outside of _delay_us().
  */
  _delay_us(12);
  bit= ow_read();
  sei();
  if ((i % 8) == 0)
      ow_read_buf[i / 8] = 0;
  if (bit)
    ow_read_buf[i / 8] |= (1 << (i % 8));
  /* Total read slot >= 60 usec. */
  ow_delay_us(60-1-12);
  return 0;
}

/* -XXX.XXXX\0 */
static char last_temp_buf[10] = { 0 };

#define OW_RECORD_TEMP {ow_record_temp, 1}
static int8_t ow_record_temp(uint8_t dummy __attribute__((unused)));

static const struct onewire_cmd
ow_cmds_read_temp_simple[] =
{
  OW_INIT,
  OW_SKIP_ROM,
  OW_CONVERT_T,
  OW_INIT,
  OW_SKIP_ROM,
  OW_READ_SCRATCH,
  OW_READ_N(9),
  OW_RECORD_TEMP,
  {0,0}
};

static int8_t
ow_record_temp(uint8_t dummy __attribute__((unused)))
{
  char *p, *q;
  uint16_t tu;
  int16_t t = (int16_t)((uint16_t)ow_read_buf[0] | ((uint16_t)(ow_read_buf[1]) << 8)) / 2;
  int8_t count_remain = ow_read_buf[6];
  int16_t t_16 = 16*t - count_remain + (16 - 4);
  p= last_temp_buf;
  if (t_16 < 0)
  {
    *p++ = '-';
    tu= (uint16_t)0 - (uint16_t)t_16;
  }
  else
    tu= (uint16_t)t_16;
  p= sprint_uint16_b10(p, tu/16);
  *p++ = '.';
  q= sprint_uint16_b10(p, tu%16*(10000/16));
  p+= 4;
  while (q < p)
    *q++ = '0';
  *p= '\0';

  /* Schedule a new temperature measurement in a bit. */
  reset_cmds(ow_cmds_read_temp_simple);
  ow_delay_ms(250);

  /*
    Hack: return error; this way we avoid updating the cmd counter after
    return, which would cause us to skip the first step of the second round.
  */
  return -1;
}

static void
start_temp_measure(void)
{
  start_cmds(ow_cmds_read_temp_simple);
}
