extern crate rf95pi;

#[cfg(test)]
mod tests {
    use super::rf95pi::{LoraRegister, Channel, SpreadingFactor, Bandwidth, CodingRate};

    #[test]
    fn register_as_u8() {
        let correct_answer = 0x42;
        assert!(correct_answer == LoraRegister::RegVersion.as_u8());
    }

    #[test]
    fn channel_msb() {
        let ca = 0xD8;
        assert!(ca == Channel::Ch13.msb());
    }

    #[test]
    fn channel_mid() {
        let ca = 0x73;
        assert!(ca == Channel::Ch12.mid());
    }

    #[test]
    fn channel_lsb() {
        let ca = 0xCC;
        assert!(ca == Channel::Ch10.lsb());
    }

    #[test]
    fn init_test() {
		if let Ok(radio) = super::rf95pi::RF95::new(Bandwidth::Bw250, CodingRate::Cr8, SpreadingFactor::Sf10) {

		} else {
			panic!("Cannot create lora radio object");
		}
	}
}
