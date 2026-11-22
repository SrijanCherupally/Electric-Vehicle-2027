#include "motor/encoder.h"

#include <hardware/gpio.h>
#include <string.h>

// ---------------------------------------------------------------------------
// PIO program (pico-examples quadrature_encoder.pio, built at runtime with the
// SDK encoders so no pioasm step is needed in the PlatformIO build).
//
// The first 16 instructions are a jump table indexed by (old_AB << 2) | new_AB.
// Y holds the count; every loop pushes Y to the RX FIFO (non-blocking), so
// the newest FIFO entry is always the current count.
// ---------------------------------------------------------------------------

namespace {

constexpr uint ADDR_DECREMENT = 14;
constexpr uint ADDR_UPDATE = 15;  // also .wrap_target
constexpr uint ADDR_INCREMENT = 21;
constexpr uint ADDR_INCREMENT_CONT = 23;  // also .wrap
constexpr uint PROGRAM_LENGTH = 24;

uint16_t g_instructions[PROGRAM_LENGTH];
pio_program_t g_program;
bool g_programBuilt = false;
PIO g_loadedPio = nullptr;

void buildProgram() {
  if (g_programBuilt) return;
  const uint U = ADDR_UPDATE, D = ADDR_DECREMENT, I = ADDR_INCREMENT;
  // Jump table: index = old(2 bits) << 2 | new(2 bits)
  const uint table[14] = {
      U, D, I, U,  // old 00: new 00, 01, 10, 11
      I, U, U, D,  // old 01
      D, U, U, I,  // old 10
      U, I,        // old 11: new 00, 01 (10 / 11 fall through below)
  };
  for (uint i = 0; i < 14; ++i) {
    g_instructions[i] = (uint16_t)pio_encode_jmp(table[i]);
  }
  g_instructions[14] = (uint16_t)pio_encode_jmp_y_dec(ADDR_UPDATE);   // decrement: Y--
  g_instructions[15] = (uint16_t)pio_encode_mov(pio_isr, pio_y);      // update: ISR = Y
  g_instructions[16] = (uint16_t)pio_encode_push(false, false);       // push noblock
  g_instructions[17] = (uint16_t)pio_encode_out(pio_isr, 2);          // ISR <- old AB
  g_instructions[18] = (uint16_t)pio_encode_in(pio_pins, 2);          // ISR <- new AB
  g_instructions[19] = (uint16_t)pio_encode_mov(pio_osr, pio_isr);    // remember state
  g_instructions[20] = (uint16_t)pio_encode_mov(pio_pc, pio_isr);     // computed jump
  g_instructions[21] = (uint16_t)pio_encode_mov_not(pio_y, pio_y);    // increment: Y = ~Y
  g_instructions[22] = (uint16_t)pio_encode_jmp_y_dec(ADDR_INCREMENT_CONT);
  g_instructions[23] = (uint16_t)pio_encode_mov_not(pio_y, pio_y);    // Y = ~(~Y - 1)

  memset(&g_program, 0, sizeof(g_program));
  g_program.instructions = g_instructions;
  g_program.length = PROGRAM_LENGTH;
  g_program.origin = 0;  // computed jumps are absolute
  g_programBuilt = true;
}

// Load the program once, into the first PIO block with offset 0 free.
PIO loadProgram() {
  if (g_loadedPio != nullptr) return g_loadedPio;
  buildProgram();
  PIO candidates[] = {pio0, pio1,
#if NUM_PIOS > 2
                      pio2
#endif
  };
  for (PIO p : candidates) {
    if (pio_can_add_program_at_offset(p, &g_program, 0)) {
      pio_add_program_at_offset(p, &g_program, 0);
      g_loadedPio = p;
      return p;
    }
  }
  return nullptr;
}

}  // namespace

bool QuadratureEncoder::begin() {
  pio_ = loadProgram();
  if (pio_ == nullptr) return false;
  sm_ = pio_claim_unused_sm(pio_, false);
  if (sm_ < 0) return false;

  for (uint i = 0; i < 2; ++i) {
    pio_gpio_init(pio_, pinBase_ + i);
    gpio_pull_up(pinBase_ + i);
  }
  pio_sm_set_consecutive_pindirs(pio_, sm_, pinBase_, 2, false);

  pio_sm_config cfg = pio_get_default_sm_config();
  sm_config_set_wrap(&cfg, ADDR_UPDATE, ADDR_INCREMENT_CONT);
  sm_config_set_in_pins(&cfg, pinBase_);
  sm_config_set_jmp_pin(&cfg, pinBase_);
  sm_config_set_in_shift(&cfg, false, false, 32);  // shift left, no autopush
  sm_config_set_out_shift(&cfg, true, false, 32);  // shift right, no autopull
  sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_NONE);
  sm_config_set_clkdiv(&cfg, 1.0f);  // ~10 clocks per loop: >10 M steps/s

  pio_sm_init(pio_, sm_, ADDR_UPDATE, &cfg);
  pio_sm_set_enabled(pio_, sm_, true);
  return true;
}

int32_t QuadratureEncoder::read() {
  if (sm_ < 0) return 0;
  // Drain everything queued plus one fresh entry, so the value is never stale.
  uint32_t value = 0;
  int n = (int)pio_sm_get_rx_fifo_level(pio_, sm_) + 1;
  while (n-- > 0) {
    value = pio_sm_get_blocking(pio_, sm_);
  }
  return (int32_t)value;
}
