use rtt_target::rprintln;

pub struct DCF77Decoder {
    current_bits: u64,
    last_bits: Option<u64>,
    pub bit_pos: usize,
    start_detected: bool,
}

impl DCF77Decoder {
    pub fn new() -> Self {
        Self {
            current_bits: 0,
            last_bits: None,
            bit_pos: 0,
            start_detected: false,
        }
    }

    pub fn reset_last_bits(&mut self) {
        self.last_bits.take();
    }

    pub fn last_bits(&self) -> Option<u64> {
        self.last_bits
    }

    pub fn add_minute(&mut self) {
        self.last_bits.replace(self.current_bits);
        rprintln!(
            "Minute mark: {:059b}, bits: {}",
            self.current_bits,
            self.bit_pos
        );
        self.current_bits = 0;
        self.bit_pos = 0;
        self.start_detected = true;
    }

    pub fn add_second(&mut self, bit: bool) {
        rprintln!("Second add: {}", bit);
        if self.bit_pos >= 59 {
            return;
        }
        if bit {
            self.current_bits |= 1 << self.bit_pos;
        } else {
            self.current_bits &= !(1 << self.bit_pos);
        }
        if self.bit_pos == 59 {
            rprintln!("Overrun!");
        }
        self.bit_pos += 1;
    }
}
