use heapless::Vec;
use stm32f1xx_hal::pac::{self};
use rtt_target::rprintln;
use rtic_monotonics::fugit::ExtU32;


const SYSCLK_HZ: u32 = 72_000_000;
const BAUD: u32 = 9_600;
const BIT_CYCLES: u32 = SYSCLK_HZ / BAUD;       // ≈ 7500 cycles per bit (72MHz/9600)
const BYTE_TIME_CYCLES: u32 = BIT_CYCLES * 10;  // 1 start + 8 data + 1 stop ≈ 75_000 cycles ~1.04 ms
const BYTE_CYCLES: u32 = BIT_CYCLES * 10;        // start+8+stop ~75k cycles ~1.04ms


// TMC recommend send delay default (SENDDELAY) is 8 bit-times; add processing margin.
// We'll wait for 8 bits + 2 extra byte-times of processing (~3 ms total).
const SWITCH_TO_RX_CYCLES: u32 = BYTE_TIME_CYCLES + (BIT_CYCLES * 8) + (BYTE_TIME_CYCLES * 2); 
// rough ~75k + 60k + 150k = ~285k cycles -> ~4ms (safe)

fn cycles_to_us(cycles: u32) -> u32 {
    // use u64 intermediates for safety and rounding
    let us = ((cycles as u64) * 1_000_000u64 + (SYSCLK_HZ as u64 / 2)) / (SYSCLK_HZ as u64);
    us as u32
}
pub struct HalfDuplexSerial {
    timeout: u32,        
    delay: stm32f1xx_hal::timer::DelayUs<pac::TIM4>,

}

impl HalfDuplexSerial {
    // Don't take Tx/Rx - we'll manage the peripheral directly
    pub fn new(timeout: u32, delay: stm32f1xx_hal::timer::DelayUs<pac::TIM4>) -> Self {
        let uart3 = unsafe { &*pac::USART3::ptr() };

        // Disable hardware half-duplex. We'll use full-duplex and flip TE/RE manually.
        uart3.cr3().modify(|_, w| w.hdsel().clear_bit());

        // Ensure USART enabled. Start with TX disabled, RX enabled so we don't drive bus unintentionally.
        uart3.cr1().modify(|_, w| {
            w.ue().set_bit()
            .te().clear_bit()
            .re().set_bit()
        });

        rprintln!("Configured (SW half-duplex) - CR1: 0x{:08X}, CR3: 0x{:08X}",
                uart3.cr1().read().bits(),
                uart3.cr3().read().bits());

        Self { timeout, delay }
    }

    pub fn send_and_get(&mut self, buffer: &[u8]) -> Result<heapless::Vec<u8, 64>, &'static str> {
        let uart3 = unsafe { &*pac::USART3::ptr() };

        rprintln!("=== Send {} bytes ===", buffer.len());

        uart3.cr1().modify(|_, w| w.te().set_bit().re().clear_bit());

        
        // Clear any stale RX data and flags
        while uart3.sr().read().rxne().bit_is_set() {
            let _ = uart3.dr().read().bits();
        }
        let _ = uart3.sr().read(); // Clear flags
        
        // Send all bytes
        for (idx, &byte) in buffer.iter().enumerate() {
            // Wait for TX register empty
            let mut count = 0;
            while uart3.sr().read().txe().bit_is_clear() {
                count += 1;
                if count > self.timeout {
                    return Err("TXE timeout");
                }
            }

            rprintln!("BRR=0x{:X}", uart3.brr().read().bits());
            rprintln!("CR1=0x{:08X}, CR3=0x{:08X}, SR=0x{:08X}",
                    uart3.cr1().read().bits(),
                    uart3.cr3().read().bits(),
                    uart3.sr().read().bits());
            
            // Write byte
            uart3.dr().write(|w| unsafe { w.dr().bits(byte as u16) });
            rprintln!("TX[{}]: 0x{:02X}", idx, byte);
        }

                // Wait for transmission complete
                // Wait for transmission complete
        let mut count = 0u32;
        while uart3.sr().read().tc().bit_is_clear() {
            count = count.saturating_add(1);
            if count > self.timeout { rprintln!("TC timeout"); break; }
        }
        rprintln!("TC after {} loops, SR=0x{:08X}", count, uart3.sr().read().bits());

        // Atomically clear TE (stop driving) and keep RE set so RX can listen
        uart3.cr1().modify(|_, w| w.te().clear_bit().re().set_bit());
        rprintln!("CR1 after stop-driving: 0x{:08X}", uart3.cr1().read().bits());

        // Wait SENDDELAY in bit-times + margin
        const SENDDELAY_BITS: u32 = 8;
        const SENDDELAY_MARGIN: u32 = 6; // bigger margin due to wiring
        let wait_cycles = (SENDDELAY_BITS + SENDDELAY_MARGIN).saturating_mul(BIT_CYCLES);
        self.delay.delay(cycles_to_us(wait_cycles).micros());

        // Drain any immediate echoes that arrived before SENDDELAY
        // let mut drained = 0u32;
        // let drain_limit = 128u32;
        // while uart3.sr().read().rxne().bit_is_set() && drained < drain_limit {
        //     let _ = uart3.dr().read().bits();
        //     drained = drained.saturating_add(1);
        // }
        // if drained != 0 { rprintln!("Drained {} echo bytes", drained); }

        // Now search for the sync byte 0x05 within timeout
        let mut reply: Vec<u8, 64> = Vec::new();
        let mut timeout_loops = 0u32;
        while timeout_loops < self.timeout {
            if uart3.sr().read().rxne().bit_is_set() {
                let b = (uart3.dr().read().bits() & 0xFF) as u8;
                if b == 0x05 {
                    reply.push(b).map_err(|_| "buffer full")?;
                    break;
                } else {
                    rprintln!("Ignored byte before sync: 0x{:02X}", b);
                }
            } else {
                timeout_loops = timeout_loops.saturating_add(1);
                self.delay.delay(cycles_to_us(100).micros());
            }
        }
        if reply.is_empty() {
            rprintln!("No sync byte seen -> treating as no-reply (write or dead device)");
            // Restore TX mode before returning
            uart3.cr1().modify(|_, w| w.te().set_bit().re().clear_bit());
            return Ok(reply);
        }

        // Read remaining expected bytes for full reply (TMC usually 8 bytes)
        let expected = 8usize;
        while reply.len() < expected {
            let mut inner = 0u32;
            while uart3.sr().read().rxne().bit_is_clear() {
                inner = inner.saturating_add(1);
                if inner > self.timeout {
                    rprintln!("Timeout waiting reply byte {}", reply.len());
                    uart3.cr1().modify(|_, w| w.te().set_bit().re().clear_bit());
                    return Ok(reply);
                }
                self.delay.delay(cycles_to_us(50).micros());
            }
            let b = (uart3.dr().read().bits() & 0xFF) as u8;
            reply.push(b).map_err(|_| "buffer full")?;
        }
        rprintln!("got {:?}", reply);

        // Validate CRC here (implement/check CRC8 LSB-first). If CRC bad -> rprintln and return Err or empty vector.

        // Restore TX (stop listening)
        uart3.cr1().modify(|_, w| w.te().set_bit().re().clear_bit());
        Ok(reply)

    }
}