/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * RC pulse capture for RP2350 using ChibiOS PAL GPIO edge callbacks.
 */

#include "SoftSigReaderRP2350.h"

#if defined(HAL_RCIN_IS_GPIO)

using namespace ChibiOS;

/*
  ISR-context callback invoked by ChibiOS PAL on every GPIO edge.
  CH_CFG_ST_FREQUENCY == 1000000 on RP2350, so chVTGetSystemTimeX() ticks == µs.
*/
void SoftSigReaderRP2350::_irq_handler(void *ctx)
{
    SoftSigReaderRP2350 *reader = static_cast<SoftSigReaderRP2350 *>(ctx);
    reader->_edge();
}

void SoftSigReaderRP2350::_edge(void)
{
    const uint32_t now = (uint32_t)chVTGetSystemTimeX();

    if (!_got_first) {
        _last_tick = now;
        _got_first = true;
        return;
    }

    const uint32_t delta = now - _last_tick;
    _last_tick = now;

    if (!_pending_valid) {
        _pending_w0 = delta;
        _pending_valid = true;
    } else {
        pulse_t p;
        p.w0 = _pending_w0;
        p.w1 = delta;
        sigbuf.push(p);
        _pending_valid = false;
    }
}

void SoftSigReaderRP2350::init(ioline_t line)
{
    _line = line;
    _got_first = false;
    _pending_valid = false;
    _last_tick = 0;
    _pending_w0 = 0;

    chSysLock();
    palSetLineCallbackI(_line, _irq_handler, this);
    palEnableLineEventI(_line, PAL_EVENT_MODE_BOTH_EDGES);
    chSysUnlock();
}

bool SoftSigReaderRP2350::read(uint32_t &widths0, uint32_t &widths1)
{
    pulse_t p;
    if (!sigbuf.pop(p)) {
        return false;
    }
    widths0 = p.w0;
    widths1 = p.w1;
    return true;
}

void SoftSigReaderRP2350::disable(void)
{
    palDisableLineEvent(_line);
}

#endif // HAL_RCIN_IS_GPIO
