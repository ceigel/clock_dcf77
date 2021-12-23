use crate::SegmentDisplay;
use adafruit_7segment::{AsciiChar, Index, SevenSegment};

pub struct SegmentDisplayAdapter {
    display: SegmentDisplay,
}

impl SegmentDisplayAdapter {
    pub fn new(display: SegmentDisplay) -> Self {
        Self { display }
    }

    pub fn display_time(&mut self, hours: u8, minutes: u8, dots: u8) {
        let d1 = (hours / 10) as u8;
        let d2 = (hours % 10) as u8;
        let d3 = (minutes / 10) as u8;
        let d4 = (minutes % 10) as u8;
        self.display.update_buffer_with_digit(Index::One, d1);
        self.display.update_buffer_with_digit(Index::Two, d2);
        self.display.update_buffer_with_digit(Index::Three, d3);
        self.display.update_buffer_with_digit(Index::Four, d4);
        self.show_points(dots);
        self.display.write_display_buffer().expect("Could not write 7-segment display");
    }

    pub fn display_error(&mut self, dots: u8) {
        self.display.update_buffer_with_char(Index::One, AsciiChar::Minus).expect("display minus");
        self.display.update_buffer_with_char(Index::Two, AsciiChar::Minus).expect("display minus");
        self.display
            .update_buffer_with_char(Index::Three, AsciiChar::Minus)
            .expect("display minus");
        self.display
            .update_buffer_with_char(Index::Four, AsciiChar::Minus)
            .expect("display minus");
        self.show_points(dots);
        self.display.update_buffer_with_colon(false);
        self.display.write_display_buffer().expect("Could not write 7-segment display");
    }

    pub fn blink_second(&mut self, on_state: bool) {
        self.display.update_buffer_with_colon(on_state);
        self.display.write_display_buffer().expect("Could not write 7-segment display");
    }
    fn show_points(&mut self, dots: u8) {
        self.display.update_buffer_with_dot(Index::Four, dots & 1 != 0);
        self.display.update_buffer_with_dot(Index::Three, dots & 2 != 0);
        self.display.update_buffer_with_dot(Index::Two, dots & 4 != 0);
        self.display.update_buffer_with_dot(Index::One, dots & 8 != 0);
    }
}
