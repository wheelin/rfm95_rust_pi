extern crate spidev;
extern crate sysfs_gpio;

use sysfs_gpio::{Direction, Edge, Pin};
use spidev::{Spidev, SpidevOptions, SpidevTransfer, SPI_MODE_0};
use std::io;
use std::thread;
use std::time::Duration;
use std::io::{Read, Write, Error, ErrorKind};

const RST_BCM_PIN : u64 = 17;
const DIO_BCM_PIN : u64 = 4;
const CS_BCM_PIN : u64 = 25;

#[derive(Copy, Clone)]
pub enum LoraRegister {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    // reserved : 0x02 - 0x05
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegOcp = 0x0B,
    RegLna = 0x0C,
    RegFifoAddrPtr = 0x0D,
    RegFifoTxBaseAddr = 0x0E,
    RegFifoRxBaseAddr = 0x0F,
    RegFifoRxCurrentAddr = 0x10,
    RegIrqFlagsMask = 0x11,
    RegIrqFlags = 0x12,
    RegRxNbBytes = 0x13,
    RegRxHeaderCntValueMsb = 0x14,
    RegRxHeaderCntValueLsb = 0x15,
    RegRxPacketCntValueMsb = 0x16,
    RegRxPacketCntValueLsb = 0x17,
    RegModemStat = 0x18,
    RegPktSnrValue = 0x19,
    RegPktRssiValue = 0x1A,
    RegRssiValue = 0x1B,
    RegHopChannel = 0x1C,
    RegModemConfig1 = 0x1D,
    RegModemConfig2 = 0x1E,
    RegSymbTimeoutLsb = 0x1F,
    RegPreambleMsb = 0x20,
    RegPreambleLsb = 0x21,
    RegPayloadLength = 0x22,
    RegMaxPayloadLength = 0x23,
    RegHopPeriod = 0x24,
    RegFifoRxByteAddr = 0x25,
    RegModemConfig3 = 0x26,
    // reserved : 0x27-0x3F
    RegDioMapping1 = 0x40,
    RegDioMapping2 = 0x41,
    RegVersion = 0x42,
    // unused 0x44
    RegTcxo = 0x4B,
    RegPaDac = 0x4D,
    RegFormerTemp = 0x5B,
    RegAgcRef = 0x61,
    RegAgcThresh1 = 0x62,
    RegAgcThresh2 = 0x63,
    RegAgcThresh3 = 0x64,
}

impl LoraRegister {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
pub enum FskOokRegister {
    RegFifo = 0x00,
    RegOpMode = 0x01,
    RegBitRateMsb = 0x02,
    RegBitRateLsb = 0x03,
    RegFdevMsb = 0x04,
    RegFdevLsb = 0x05,
    RegFrfMsb = 0x06,
    RegFrfMid = 0x07,
    RegFrfLsb = 0x08,
    RegPaConfig = 0x09,
    RegPaRamp = 0x0A,
    RegOcp = 0x0B,
    RegLna = 0x0C,
    RegRxConfig = 0x0D,
    RegRssiConfig = 0x0E,
    RegRssiCollision = 0x0F,
    RegRssiThresh = 0x10,
    RegRssiValue = 0x11,
    RegRxBw = 0x12,
    RegAfcBw = 0x13,
    RegOokPeak = 0x14,
    RegOokFix = 0x15,
    RegOokAvg = 0x16,
    RegAfcFei = 0x1A,
    RegAfcMsb = 0x1B,
    RegAfcLsb = 0x1C,
    RegFeiMsb = 0x1D,
    RegFeiLsb = 0x1E,
    RegPreambleDetect = 0x1F,
    RegRxTimeout1 = 0x20,
    RegRxTimeout2 = 0x21,
    RegRxTimeout3 = 0x22,
    RegRxDelay = 0x23,
    RegOsc = 0x24,
    RegPreambleMsb = 0x25,
    RegPreambleLsb = 0x26,
    RegSyncConfig = 0x27,
    RegSyncValue1 = 0x28,
    RegSyncValue2 = 0x29,
    RegSyncValue3 = 0x2A,
    RegSyncValue4 = 0x2B,
    RegSyncValue5 = 0x2C,
    RegSyncValue6 = 0x2D,
    RegSyncValue7 = 0x2E,
    RegSyncValue8 = 0x2F,
    RegPacketconfig1 = 0x30,
    RegPacketConfig2 = 0x31,
    RegPayloadLength = 0x32,
    RegNodeAdrs = 0x33,
    RegBroadcastAdrs = 0x34,
    RegFifoThresh = 0x35,
    RegSeqConfig1 = 0x36,
    RegSeqConfig2 = 0x37,
    RegTimerResol = 0x38,
    RegTimer1Coef = 0x39,
    RegTimer2Coef = 0x3A,
    RegImageCal = 0x3B,
    RegTemp = 0x3C,
    RegLowBat = 0x3D,
    RegIrqFlags1 = 0x3E,
    RegIrqFlags2 = 0x3F,
    RegDioMapping1 = 0x40,
    RegDioMapping2 = 0x41,
    RegVersion = 0x42,
    RegPllHop = 0x44,
    RegTxco = 0x4B,
    RegPaDac = 0x4D,
    RegFormerTemp = 0x5B,
    RegBitRateFrac = 0x5D,
    RegAgcRef = 0x61,
    RegAgcThresh1 = 0x62,
    RegAgcThresh2 = 0x63,
    RegAgcThresh3 = 0x64,
}

impl FskOokRegister {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
#[allow(non_snake_case)]
pub enum Channel {
    Ch10 = 0xD84CCC,
    Ch11 = 0xD86000,
    Ch12 = 0xD87333,
    Ch13 = 0xD88666,
    Ch14 = 0xD89999,
    Ch15 = 0xD8ACCC,
    Ch16 = 0xD8C000,
    Ch17 = 0xD90000,
}

impl Channel {
    pub fn msb(&self) -> u8 {
        let chan_as_u8 = *self as u32;
        ((chan_as_u8 >> 16) & 0x000000FF) as u8
    }

    pub fn mid(&self) -> u8 {
        let chan_as_u8 = *self as u32;
        ((chan_as_u8 >> 8) & 0x000000FF) as u8
    }

    pub fn lsb(&self) -> u8 {
        let chan_as_u8 = *self as u32;
        (chan_as_u8 & 0x000000FF) as u8
    }
}

#[derive(Copy, Clone)]
enum Bandwidth {
    Bw125 = 0x07,
    Bw250 = 0x08,
    Bw500 = 0x09,
}

impl Bandwidth {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
enum CodingRate {
    Cr5 = 0x01,
    Cr6 = 0x02,
    Cr7 = 0x03,
    Cr8 = 0x04,
}

impl CodingRate {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
enum SpreadingFactor {
    Sf7 = 0x07,
    Sf8 = 0x08,
    Sf9 = 0x09,
    Sf10 = 0x0A,
    Sf11 = 0x0B,
    Sf12 = 0x0C,
}

impl SpreadingFactor {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
enum LoraMode {
    Sleep = 0x80,
    Standby = 0x81,
    Tx = 0x83,
    Rx = 0x85,
    RxSingle = 0x86,
    CadDetection = 0x87,
    StandbyOokFsk = 0xC1,
}

impl LoraMode {
    pub fn as_u8(&self) -> u8 {
        *self as u8
    }
}

#[derive(Copy, Clone)]
enum DioFunction {
    RxDone = 0x00,
    TxDone = 0x01,
}

impl DioFunction {
    pub fn as_u8(&self) -> u8 { *self as u8 }
}

struct RF95 {
    ch : Channel,
    bw : Bandwidth,
    cr : CodingRate,
    sf : SpreadingFactor,
    crc_check_enabled : bool,
    implicit_header_enabled : bool,
    pwr_db : u8,
    continuous_receiving_enabled : bool,

    sent_callback : Option<Box<FnMut()>>,
    recv_callback : Option<Box<FnMut()>>,
    err_callback  : Option<Box<FnMut()>>,

    rst_pin : Pin,
    int_pin : Pin,
    cs_pin  : Pin,

    dev : Spidev,
}

impl RF95 {
    pub fn new(bw : Bandwidth, cr : CodingRate, sf : SpreadingFactor) -> io::Result<RF95> {
        let mut tmpspi = Spidev::open("/dev/spidev0.0")?;
        let spi_options = SpidevOptions::new()
            .bits_per_word(8)
            .max_speed_hz(5_000_000)
            .mode(SPI_MODE_0)
            .build();
        tmpspi.configure(&spi_options)?;

        let mut tmp_rst_pin = Pin::new(RST_BCM_PIN);
        tmp_rst_pin.set_direction(Direction::Low);
        thread::sleep(Duration::from_millis(10));
        tmp_rst_pin.set_value(1);


        let mut tmp_dio_pin = Pin::new(DIO_BCM_PIN);
        tmp_dio_pin.set_direction(Direction::In);

        let mut tmp_cs_pin  = Pin::new(CS_BCM_PIN);
        tmp_cs_pin.set_direction(Direction::High);

        Ok(
            RF95 {
                ch            : Channel::Ch10,
                bw,
                cr,
                sf,
                crc_check_enabled : false,
                implicit_header_enabled : false,
                pwr_db        : 0,
                continuous_receiving_enabled : false,
                sent_callback : None,
                recv_callback : None,
                err_callback  : None,
                rst_pin       : tmp_rst_pin,
                int_pin       : tmp_dio_pin,
                cs_pin        : tmp_cs_pin,
                dev           : tmpspi,
            }
        )
    }

    pub fn set_main_parameters(&mut self,
                               ch : Channel,
                               bw : Bandwidth,
                               cr : CodingRate,
                               sf : SpreadingFactor) -> io::Result<()> {
        self.set_channel(ch)?;
        self.set_bandwidth(bw)?;
        self.set_coding_rate(cr)?;
        self.set_spreading_factor(sf)
    }

    pub fn set_channel(&mut self, ch : Channel) -> io::Result<()> {
        self.set_mode(LoraMode::Sleep)?;
        self.ch = ch;
        self.write_register(LoraRegister::RegFrfMsb, ch.msb())?;
        self.write_register(LoraRegister::RegFrfMid, ch.mid())?;
        self.write_register(LoraRegister::RegFrfLsb, ch.lsb())
    }

    pub fn set_spreading_factor(&mut self, sf : SpreadingFactor) -> io::Result<()> {
        self.set_mode(LoraMode::Sleep)?;
        self.sf = sf;
        let mut tmp = self.read_register(LoraRegister::RegModemConfig2)?;
        tmp &= 0x0F;
        tmp |= sf.as_u8() << 4;
        self.write_register(LoraRegister::RegModemConfig2, tmp)
    }

    pub fn set_bandwidth(&mut self, bw : Bandwidth) -> io::Result<()> {
        self.set_mode(LoraMode::Sleep)?;
        self.bw = bw;
        let mut tmp = self.read_register(LoraRegister::RegModemConfig1)?;
        tmp &= 0x0F;
        tmp |= bw.as_u8() << 4;
        self.write_register(LoraRegister::RegModemConfig1, tmp)?;
        Ok(())
    }

    pub fn set_coding_rate(&mut self, cr : CodingRate) -> io::Result<()> {
        self.set_mode(LoraMode::Sleep)?;
        self.cr = cr;
        let mut tmp = self.read_register(LoraRegister::RegModemConfig1)?;
        tmp &= 0xF1;
        tmp |= cr.as_u8() << 1;
        self.write_register(LoraRegister::RegModemConfig1, tmp)
    }

    pub fn enable_crc_check(&mut self, en : bool) -> io::Result<()> {
        let mut tmp = self.read_register(LoraRegister::RegModemConfig2)?;
        if en {
            tmp |= 1 << 2;
        } else {
            tmp &= !(1 << 2);
        }
        self.crc_check_enabled = en;
        self.write_register(LoraRegister::RegModemConfig2, tmp)
    }

    pub fn enable_implicit_header(&mut self, en : bool) -> io::Result<()> {
        let mut tmp = self.read_register(LoraRegister::RegModemConfig1)?;
        if en {
            tmp |= 0x01;
        } else {
            tmp &= !0x01;
        }
        self.implicit_header_enabled = en;
        self.write_register(LoraRegister::RegModemConfig1, tmp)
    }

    pub fn set_output_power(&mut self, pwr : u8) -> io::Result<()> {
        let mut pwr = pwr;
        if pwr > 20 {
            pwr = 20;
        }
        self.pwr_db = pwr;
        let out = (pwr - 2) & 0x0F;
        let mut tmp = self.read_register(LoraRegister::RegPaConfig)?;
        tmp |= 0x80;
        tmp &= 0xF0;
        tmp |= out & 0x0F;
        self.write_register(LoraRegister::RegPaConfig, tmp)
    }

    pub fn set_mode(&mut self, m : LoraMode) -> io::Result<()> {
        self.write_register(LoraRegister::RegOpMode, m.as_u8())
    }

    pub fn listen_timed(&mut self, timeout : u32) -> io::Result<()> {
        
        Ok(())
    }

    pub fn disable_timer(&mut self) -> io::Result<()> {
        Ok(())
    }

    pub fn listen_continuous(&mut self) -> io::Result<()> {
        Ok(())
    }

    pub fn stop_listening(&mut self) -> io::Result<()> {
        Ok(())
    }

    pub fn get_snr(&mut self) -> io::Result<i16> {
        Ok((self.read_register(LoraRegister::RegPktSnrValue)? as i16) / 4)
    }

    pub fn get_packet_rssi(&mut self) -> io::Result<i16> {
        Ok((self.read_register(LoraRegister::RegRssiValue)? as i16) - 137)
    }

    pub fn get_rssi(&mut self) -> io::Result<i16> {
        Ok((self.read_register(LoraRegister::RegRssiValue)? as i16) - 137)
    }

    pub fn reset(&mut self) -> io::Result<()> {
        match self.rst_pin.set_direction(Direction::Low) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        std::thread::sleep(Duration::new(0, 10000000));
        match self.rst_pin.set_value(1) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        Ok(())
    }

    pub fn set_dio_mapping(&mut self, df : DioFunction) -> io::Result<()> {
        let prev_mode = self.read_register(LoraRegister::RegOpMode)?;
        self.set_mode(LoraMode::StandbyOokFsk)?;
        let mut tmp = self.read_register(LoraRegister::RegDioMapping1)?;
        tmp &= 0x3F;
        if df.as_u8() == DioFunction::TxDone.as_u8() {
            tmp |= 0x40;
        }
        self.write_register(LoraRegister::RegDioMapping1, tmp)?;
        self.write_register(LoraRegister::RegOpMode, prev_mode)
    }

    fn write_register(&mut self, reg : LoraRegister, data : u8) -> io::Result<()> {
        match self.dev.write(&[reg.as_u8(), data]) {
            Ok(_) => Ok(()),
            Err(_) => Err(io::Error::new(ErrorKind::Other, "Problem while writing to device"))
        }
    }

    fn write_buffer(&mut self, reg : LoraRegister, buffer : Vec<u8>) -> io::Result<()> {
        let mut v2 = buffer;
        v2.insert(0, reg.as_u8());
        match self.cs_pin.set_value(0) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        self.dev.write(&v2)?;
        match self.cs_pin.set_value(1) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        }
    }

    fn read_register(&mut self, reg : LoraRegister) -> io::Result<u8> {
        let mut ret: [u8; 1] = [0; 1];
        match self.cs_pin.set_value(0) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        self.dev.write(&[reg.as_u8()])?;
        let val = self.dev.read(&mut ret)?;
        match self.cs_pin.set_value(1) {
            Ok(_) => (),
            Err(_) => return Err(io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        Ok(ret[0])
    }

    fn read_buffer(&mut self, reg : LoraRegister, buffer : &mut Vec<u8>, length : u8) -> io::Result<()> {
        let mut tmp : Vec<u8> = Vec::with_capacity(length as usize);
        match self.cs_pin.set_value(0) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        self.dev.write(&[reg.as_u8()])?;
        self.dev.read(tmp.as_mut_slice())?;
        buffer.clear();
        buffer.write(&tmp)?;
        match self.cs_pin.set_value(1) {
            Ok(_) => Ok(()),
            Err(_) => Err(std::io::Error::new(ErrorKind::Other, "Problem setting value on gpio")),
        };
        Ok(())
    }
}
