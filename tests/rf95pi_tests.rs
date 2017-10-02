extern crate rf95pi;

use rf95pi::{LoraRegister, Channel};

#[cfg(test)]
mod tests {
    use super::LoraRegister::*;

    #[test]
    fn register_as_u8() {
        let correct_answer = 0x42;
        assert!(correct_answer == RegVersion.as_u8());
    }

    use super::Channel::*;
    #[test]
    fn channel_msb() {
        let ca = 0xD8;
        assert!(ca == Ch13.msb());
    }

    #[test]
    fn channel_mid() {
        let ca = 0x73;
        assert!(ca == Ch12.mid());
    }

    #[test]
    fn channel_lsb() {
        let ca = 0xCC;
        assert!(ca == Ch10.lsb());
    }
}
