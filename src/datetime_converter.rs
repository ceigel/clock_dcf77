use chrono::naive::{NaiveDate, NaiveDateTime};
use core::fmt::Debug;
use rtt_target::rprintln;
#[derive(Debug)]
pub enum DateTimeErr {
    TimeWrong,
    DateWrong,
    WrongStart,
}

/// Decode DCF77 binary, and output as chrono::naive::NaiveDateTime
/// Sample data:
/// * 23:14 15.Sep.2021 Deutschland (CEST)
/// * `let test_data = 0b00000000000000001000_1_0010100_0_110001_1_101010_110_10010_10000100_1_0;`
/// * Zero-padded 64bit intgeger: `000000000000000010001001010001100011101010110100101000010010`
/// ```
/// // 23:14 15.Sep.2021 Deutschland (CEST)
/// // Write me!
/// ```
pub struct DCF77DateTimeConverter {
    encoded_data: u64,
    bcd: [u32; 8],
}

const THREE_BITS: u64 = (1 << 3) - 1;
const FIVE_BITS: u64 = (1 << 5) - 1;
const SIX_BITS: u64 = (1 << 6) - 1;
const SEVEN_BITS: u64 = (1 << 7) - 1;
const EIGHT_BITS: u64 = (1 << 8) - 1;
const TWENTYTWO_BITS: u64 = (1 << 22) - 1;

impl DCF77DateTimeConverter {
    pub fn new(dcf77_data: u64) -> Self {
        DCF77DateTimeConverter {
            encoded_data: dcf77_data,
            bcd: [1, 2, 4, 8, 10, 20, 40, 80],
        }
    }

    pub fn dcf77_decoder(&self) -> Result<NaiveDateTime, DateTimeErr> {
        if ((self.encoded_data >> 20) & 1) != 1 || (self.encoded_data & 1) != 0 {
            return Err(DateTimeErr::WrongStart);
        }
        let year = ((self.encoded_data >> 50) & EIGHT_BITS) as u32;
        let month = ((self.encoded_data >> 45) & FIVE_BITS) as u32;
        let _weekday = ((self.encoded_data >> 42) & THREE_BITS) as u32;
        let day = ((self.encoded_data >> 36) & SIX_BITS) as u32;
        let datetime_frame = ((self.encoded_data >> 36) & TWENTYTWO_BITS) as u32;
        let hours = ((self.encoded_data >> 29) & SIX_BITS) as u32;
        let minutes = ((self.encoded_data >> 21) & SEVEN_BITS) as u32;

        let check_datetime_parity: bool = DCF77DateTimeConverter::check_parity(datetime_frame)
            == ((self.encoded_data >> 58) & 1) as u32;
        let check_hours_parity: bool =
            DCF77DateTimeConverter::check_parity(hours) == ((self.encoded_data >> 35) & 1) as u32;
        let check_minutes_parity: bool =
            DCF77DateTimeConverter::check_parity(minutes) == ((self.encoded_data >> 28) & 1) as u32;

        let year = DCF77DateTimeConverter::naive_year(self, year) as i32;
        let month = DCF77DateTimeConverter::naive_month(self, month);
        let day = DCF77DateTimeConverter::naive_day_or_hours(self, day);
        rprintln!("Naive date: {}-{}-{}", day, month, year);

        if !check_datetime_parity {
            return Err(DateTimeErr::DateWrong);
        }

        let hours = DCF77DateTimeConverter::naive_day_or_hours(self, hours);
        let minutes = DCF77DateTimeConverter::naive_minutes(self, minutes);
        rprintln!("Naive time: {}:{}", hours, minutes);
        if !check_hours_parity {
            return Err(DateTimeErr::TimeWrong);
        }
        if !check_minutes_parity {
            return Err(DateTimeErr::TimeWrong);
        }

        let date_time = NaiveDate::from_ymd_opt(year, month, day);
        if let Some(ymd) = date_time {
            if let Some(date_time) = ymd.and_hms_opt(hours, minutes, 0) {
                Ok(date_time)
            } else {
                Err(DateTimeErr::TimeWrong)
            }
        } else {
            Err(DateTimeErr::DateWrong)
        }
    }

    fn naive_year(&self, year_dcf77: u32) -> u32 {
        let mut naive_year = 2000;
        for bit in 0..8 {
            naive_year += self.bcd[bit] * ((year_dcf77 >> bit) & 1)
        }
        naive_year
    }
    fn naive_month(&self, month_dcf77: u32) -> u32 {
        let mut naive_month = 0;
        for bit in 0..5 {
            naive_month += self.bcd[bit] * ((month_dcf77 >> bit) & 1)
        }
        naive_month
    }
    fn naive_day_or_hours(&self, day_dcf77: u32) -> u32 {
        let mut naive_day = 0;
        for bit in 0..6 {
            naive_day += self.bcd[bit] * ((day_dcf77 >> bit) & 1)
        }
        naive_day
    }

    fn naive_minutes(&self, minutes_dcf77: u32) -> u32 {
        let mut naive_minutes = 0;
        for bit in 0..7 {
            naive_minutes += self.bcd[bit] * ((minutes_dcf77 >> bit) & 1)
        }
        naive_minutes
    }

    /// Check bit-parity of a u64 integer
    /// ```
    /// assert_eq!(check_parity(13), 1)
    /// assert_eq!(check_parity(2806404), 1)
    /// ```
    fn check_parity(i: u32) -> u32 {
        let count = i.count_ones();
        return count % 2;
    }
}
