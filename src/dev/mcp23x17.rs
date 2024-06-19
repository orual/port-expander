//! Support for the `MCP23017` and `MCP23S17` "16-Bit I/O Expander with Serial Interface"
//!
//! Datasheet: https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
//!
//! The MCP23x17 offers two eight-bit GPIO ports.  It has three
//! address pins, so eight devices can coexist on an I2C bus.
//!
//! Each port has an interrupt, which can be configured to work
//! together or independently.
//!
//! When passing 16-bit values to this driver, the upper byte corresponds to port
//! B (pins 7..0) and the lower byte corresponds to port A (pins 7..0).
use core::{convert::Infallible, task::Waker};

use embassy_sync::waitqueue::MultiWakerRegistration;
use embedded_hal::digital::ErrorType;
use embedded_hal_async::digital::Wait;

use crate::{pin::PinError, I2cExt, Interrupt, InterruptType, PortDriverInterrupts, PortDriverIrqState, PortDriverExtI};

/// `MCP23x17` "16-Bit I/O Expander with Serial Interface" with I2C or SPI interface
pub struct Mcp23x17<M>(M);

impl<I2C> Mcp23x17<core::cell::RefCell<Driver<Mcp23017Bus<I2C>>>>
where
    I2C: crate::I2cBus,
{
    /// Create a new instance of the MCP23017 with I2C interface
    pub fn new_mcp23017(bus: I2C, a0: bool, a1: bool, a2: bool) -> Self {
        Self::with_mutex(Mcp23017Bus(bus), a0, a1, a2)
    }
}

impl<SPI> Mcp23x17<core::cell::RefCell<Driver<Mcp23S17Bus<SPI>>>>
where
    SPI: crate::SpiBus,
{
    /// Create a new instance of the MCP23S17 with SPI interface
    pub fn new_mcp23s17(bus: SPI) -> Self {
        Self::with_mutex(Mcp23S17Bus(bus), false, false, false)
    }
}

impl<B, M> Mcp23x17<M>
where
    B: Mcp23x17Bus,
    M: crate::PortMutex<Port = Driver<B>>,
{
    pub fn with_mutex(bus: B, a0: bool, a1: bool, a2: bool) -> Self {
        Self(crate::PortMutex::create(Driver::new(bus, a0, a1, a2)))
    }

    pub fn split<'a>(&'a mut self) -> Parts<'a, B, M> {
        Parts {
            gpa0: crate::Pin::new(0, &self.0),
            gpa1: crate::Pin::new(1, &self.0),
            gpa2: crate::Pin::new(2, &self.0),
            gpa3: crate::Pin::new(3, &self.0),
            gpa4: crate::Pin::new(4, &self.0),
            gpa5: crate::Pin::new(5, &self.0),
            gpa6: crate::Pin::new(6, &self.0),
            gpa7: crate::Pin::new(7, &self.0),
            gpb0: crate::Pin::new(8, &self.0),
            gpb1: crate::Pin::new(9, &self.0),
            gpb2: crate::Pin::new(10, &self.0),
            gpb3: crate::Pin::new(11, &self.0),
            gpb4: crate::Pin::new(12, &self.0),
            gpb5: crate::Pin::new(13, &self.0),
            gpb6: crate::Pin::new(14, &self.0),
            gpb7: crate::Pin::new(15, &self.0),
        }
    }
}

pub struct Parts<'a, B, M = core::cell::RefCell<Driver<B>>>
where
    B: Mcp23x17Bus,
    M: crate::PortMutex<Port = Driver<B>>,

    
{
    pub gpa0: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa1: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa2: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa3: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa4: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa5: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa6: crate::Pin<'a, crate::mode::Input, M>,
    pub gpa7: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb0: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb1: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb2: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb3: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb4: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb5: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb6: crate::Pin<'a, crate::mode::Input, M>,
    pub gpb7: crate::Pin<'a, crate::mode::Input, M>,
}

/// `MCP23017` 16-Bit I/O Expander wrapped with external interrupt pins
pub struct Mcp23x17ExtI<M, IA, IB> {
    bus: M,
    pub exti_a: IA,
    pub exti_b: IB,
}

impl<I2C, IA, IB> Mcp23x17ExtI<
    core::cell::RefCell<Driver<Mcp23017Bus<I2C>>>, 
    core::cell::RefCell<MCPExtI<IA, 8>>, 
    core::cell::RefCell<MCPExtI<IB, 8>>, 
    >
where
    I2C: crate::I2cBus,
    IA: crate::common::ExtIPin,
    IB: crate::common::ExtIPin
{
    /// Create a new instance of the MCP23017 with I2C interface
    pub fn new_mcp23017(bus: I2C, exti_a: IA, exti_b: IB, a0: bool, a1: bool, a2: bool) -> Self {
        Self::with_mutex(Mcp23017Bus(bus), MCPExtI::new(exti_a), MCPExtI::new(exti_b), a0, a1, a2)
    }
}

impl<SPI, IA, IB> Mcp23x17ExtI<
    core::cell::RefCell<Driver<Mcp23S17Bus<SPI>>>,
    core::cell::RefCell<MCPExtI<IA, 8>>, 
    core::cell::RefCell<MCPExtI<IB, 8>>, 
    >
where
    SPI: crate::SpiBus,
    IA: crate::common::ExtIPin,
    IB: crate::common::ExtIPin
{
    /// Create a new instance of the MCP23S17 with SPI interface
    pub fn new_mcp23s17(bus: SPI, exti_a: IA, exti_b: IB) -> Self {
        Self::with_mutex(Mcp23S17Bus(bus), MCPExtI::new(exti_a), MCPExtI::new(exti_b), false, false, false)
    }
}


pub struct MCPExtI<T, const N: usize> {
    pin: T,
    wakers: heapless::Vec<Waker, N>,
}

impl <T, const N: usize> MCPExtI<T, {N}> {
    pub fn new(pin: T) -> Self {
        Self {
            pin,
            wakers: heapless::Vec::new()
        }
    }

    pub fn register(&mut self, w: &Waker, index: usize) {
        // If we already have some waker that wakes the same task as `w`, do nothing.
        // This avoids cloning wakers, and avoids unnecessary mass-wakes.
        for w2 in &self.wakers {
            if w.will_wake(w2) {
                return;
            }
        }

        self.wakers[index] = w.clone();
    }

    /// Wake all registered wakers. This clears the buffer
    pub fn wake_all(&mut self) {
        // heapless::Vec has no `drain()`, do it unsafely ourselves...

        // First set length to 0, without dropping the contents.
        // This is necessary for soundness: if wake() panics and we're using panic=unwind.
        // Setting len=0 upfront ensures other code can't observe the vec in an inconsistent state.
        // (it'll leak wakers, but that's not UB)
        let len = self.wakers.len();
        unsafe { self.wakers.set_len(0) }

        for i in 0..len {
            // Move a waker out of the vec.
            let waker = unsafe { self.wakers.as_mut_ptr().add(i).read() };
            // Wake it by value, which consumes (drops) it.
            waker.wake();
        }
    }
    
}

impl<T, const N: usize> ErrorType for MCPExtI<T, { N }>
where
    T: ErrorType,
{
    type Error = PinError<T::Error>;
}

impl <T, E, const N: usize> Wait for MCPExtI<T, { N }>
where 
    T: Wait<Error = E>,
{
    async fn wait_for_high(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_high().await?;
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_low().await?;
        Ok(())
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_rising_edge().await?;
        Ok(())
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_falling_edge().await?;
        Ok(())
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Self::Error> {
        self.pin.wait_for_any_edge().await?;
        Ok(())
    }
}


impl<B, M, IA, IB, IAM, IBM> Mcp23x17ExtI<M, IAM, IBM>
where
    B: Mcp23x17Bus,
    M: crate::PortMutex<Port = Driver<B>>,
    IA: crate::common::ExtIPin,
    IB: crate::common::ExtIPin,
    IAM: crate::PortMutex<Port = MCPExtI<IA, 8>>,
    IBM: crate::PortMutex<Port = MCPExtI<IB, 8>>,

{
    pub fn with_mutex(bus: B, exti_a: MCPExtI<IA, 8>, exti_b: MCPExtI<IB, 8>, a0: bool, a1: bool, a2: bool) -> Self {
        Self {
            bus: crate::PortMutex::create(Driver::new(bus, a0, a1, a2)),
            exti_a: crate::PortMutex::create(exti_a),
            exti_b: crate::PortMutex::create(exti_b)
        }
    }

    pub fn split<'a>(&'a mut self) -> IntParts<'a, B, IA, IB, IAM, IBM, M> {
        IntParts {
            gpa0: crate::IntPin::new(0, &self.bus, &self.exti_a),
            gpa1: crate::IntPin::new(1, &self.bus, &self.exti_a),
            gpa2: crate::IntPin::new(2, &self.bus, &self.exti_a),
            gpa3: crate::IntPin::new(3, &self.bus, &self.exti_a),
            gpa4: crate::IntPin::new(4, &self.bus, &self.exti_a),
            gpa5: crate::IntPin::new(5, &self.bus, &self.exti_a),
            gpa6: crate::IntPin::new(6, &self.bus, &self.exti_a),
            gpa7: crate::IntPin::new(7, &self.bus, &self.exti_a),
            gpb0: crate::IntPin::new(8, &self.bus, &self.exti_b),
            gpb1: crate::IntPin::new(9, &self.bus, &self.exti_b),
            gpb2: crate::IntPin::new(10, &self.bus, &self.exti_b),
            gpb3: crate::IntPin::new(11, &self.bus, &self.exti_b),
            gpb4: crate::IntPin::new(12, &self.bus, &self.exti_b),
            gpb5: crate::IntPin::new(13, &self.bus, &self.exti_b),
            gpb6: crate::IntPin::new(14, &self.bus, &self.exti_b),
            gpb7: crate::IntPin::new(15, &self.bus, &self.exti_b),
        }
    }
}

pub struct IntParts<'a, B, IA, IB, IAM, IBM, M = core::cell::RefCell<Driver<B>>>
where
    B: Mcp23x17Bus,
    M: crate::PortMutex<Port = Driver<B>>,
    IA: crate::common::ExtIPin,
    IB: crate::common::ExtIPin,
    IAM: crate::PortMutex<Port = MCPExtI<IA, 8>>,
    IBM: crate::PortMutex<Port = MCPExtI<IB, 8>>,
    
{
    pub gpa0: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa1: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa2: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa3: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa4: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa5: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa6: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpa7: crate::IntPin<'a, crate::mode::Input, M, IAM>,
    pub gpb0: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb1: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb2: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb3: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb4: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb5: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb6: crate::IntPin<'a, crate::mode::Input, M, IBM>,
    pub gpb7: crate::IntPin<'a, crate::mode::Input, M, IBM>,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// N.B.: These values are for BANK=0, which is the reset state of
/// the chip (and this driver does not change).
///
/// For all registers, the reset value is 0x00, except for
/// IODIR{A,B} which are 0xFF (making all pins inputs) at reset.
enum Regs {
    /// IODIR: input/output direction: 0=output; 1=input
    IODIRA = 0x00,
    /// IPOL: input polarity: 0=register values match input pins; 1=opposite
    IPOLA = 0x02,
    /// GPINTEN: interrupt-on-change: 0=disable; 1=enable
    GPINTENA = 0x04,
    /// DEFVAL: default values for interrupt-on-change
    DEFVALA = 0x06,
    /// INTCON: interrupt-on-change config: 0=compare to previous pin value;
    ///   1=compare to corresponding bit in DEFVAL
    INTCONA = 0x08,
    /// IOCON: configuration register
    /// - Pin 7: BANK (which driver assumes stays 0)
    /// - Pin 6: MIRROR: if enabled, INT{A,B} are logically ORed; an interrupt on either
    ///          port will cause both pins to activate
    /// - Pin 5: SEQOP: controls the incrementing function of the address pointer
    /// - Pin 4: DISSLW: disables slew rate control on SDA
    /// - Pin 3: HAEN: no effect on MCP23017, enables address pins on MCP23S17
    /// - Pin 2: ODR: interrupt pins are 0=active-driver outputs (INTPOL sets polarity)
    ///          or 1=open-drain outputs (overrides INTPOL)
    /// - Pin 1: INTPOL: interrupt pin is 0=active-low or 1=active-high
    /// - Pin 0: unused
    IOCONA = 0x0a,
    /// GPPU: GPIO pull-ups: enables weak internal pull-ups on each pin (when configured
    ///   as an input)
    GPPUA = 0x0c,
    /// INTF: interrupt flags: 0=no interrupt pending; 1=corresponding pin caused interrupt
    INTFA = 0x0e,
    /// INTCAP: interrupt captured value: reflects value of each pin at the time that they
    ///   caused an interrupt
    INTCAPA = 0x10,
    /// GPIO: reflects logic level on pins
    GPIOA = 0x12,
    /// OLAT: output latches: sets state for pins configured as outputs
    OLATA = 0x14,
    /// IODIR: input/output direction: 0=output; 1=input
    IODIRB = 0x01,
    /// IPOL: input polarity: 0=register values match input pins; 1=opposite
    IPOLB = 0x03,
    /// GPINTEN: interrupt-on-change: 0=disable; 1=enable
    GPINTENB = 0x05,
    /// DEFVAL: default values for interrupt-on-change
    DEFVALB = 0x07,
    /// INTCON: interrupt-on-change config: 0=compare to previous pin value;
    ///   1=compare to corresponding bit in DEFVAL
    INTCONB = 0x09,
    /// IOCON: configuration register
    /// - Pin 7: BANK (which driver assumes stays 0)
    /// - Pin 6: MIRROR: if enabled, INT{A,B} are logically ORed; an interrupt on either
    ///          port will cause both pins to activate
    /// - Pin 5: SEQOP: controls the incrementing function of the address pointer
    /// - Pin 4: DISSLW: disables slew rate control on SDA
    /// - Pin 3: HAEN: no effect on MCP23017, enables address pins on MCP23S17
    /// - Pin 2: ODR: interrupt pins are 0=active-driver outputs (INTPOL sets polarity)
    ///          or 1=open-drain outputs (overrides INTPOL)
    /// - Pin 1: INTPOL: interrupt pin is 0=active-low or 1=active-high
    /// - Pin 0: unused
    IOCONB = 0x0b,
    /// GPPU: GPIO pull-ups: enables weak internal pull-ups on each pin (when configured
    ///   as an input)
    GPPUB = 0x0d,
    /// INTF: interrupt flags: 0=no interrupt pending; 1=corresponding pin caused interrupt
    INTFB = 0x0f,
    /// INTCAP: interrupt captured value: reflects value of each pin at the time that they
    ///   caused an interrupt
    INTCAPB = 0x11,
    /// GPIO: reflects logic level on pins
    GPIOB = 0x13,
    /// OLAT: output latches: sets state for pins configured as outputs
    OLATB = 0x15,
}

impl From<Regs> for u8 {
    fn from(r: Regs) -> u8 {
        r as u8
    }
}

pub struct Driver<B> {
    bus: B,
    out: u16,
    addr: u8,
    // annoying amount of state to cache, but helps with interrupt handling
    int_changed: u16,
    int_default: u16,
    int_level: u16,
    interrupted: u16,
    changed: u16,
}

impl<B> Driver<B> {
    pub fn new(bus: B, a0: bool, a1: bool, a2: bool) -> Self {
        let addr = 0x20 | ((a2 as u8) << 2) | ((a1 as u8) << 1) | (a0 as u8);
        Self {
            bus,
            out: 0x0000,
            addr,
            int_changed: 0x0000,
            int_default: 0x0000,
            int_level: 0x0000,
            interrupted: 0x0000,
            changed: 0x0000,
        }
    }
}

impl<B: Mcp23x17Bus> crate::PortDriver for Driver<B> {
    type Error = B::BusError;

    fn set(&mut self, mask_high: u32, mask_low: u32) -> Result<(), Self::Error> {
        self.out |= mask_high as u16;
        self.out &= !mask_low as u16;
        if (mask_high | mask_low) & 0x00FF != 0 {
            self.bus
                .write_reg(self.addr, Regs::GPIOA, (self.out & 0xFF) as u8)?;
        }
        if (mask_high | mask_low) & 0xFF00 != 0 {
            self.bus
                .write_reg(self.addr, Regs::GPIOB, (self.out >> 8) as u8)?;
        }
        Ok(())
    }

    fn is_set(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error> {
        Ok(((self.out as u32) & mask_high) | (!(self.out as u32) & mask_low))
    }

    fn get(&mut self, mask_high: u32, mask_low: u32) -> Result<u32, Self::Error> {
        let io0 = if (mask_high | mask_low) & 0x00FF != 0 {
            self.bus.read_reg(self.addr, Regs::GPIOA)?
        } else {
            0
        };
        let io1 = if (mask_high | mask_low) & 0xFF00 != 0 {
            self.bus.read_reg(self.addr, Regs::GPIOB)?
        } else {
            0
        };
        let in_ = ((io1 as u32) << 8) | io0 as u32;
        Ok((in_ & mask_high) | (!in_ & mask_low))
    }
}

impl<B: Mcp23x17Bus> crate::PortDriverTotemPole for Driver<B> {
    fn set_direction(
        &mut self,
        mask: u32,
        dir: crate::Direction,
        _state: bool,
    ) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match dir {
            crate::Direction::Input => (mask as u16, 0),
            crate::Direction::Output => (0, mask as u16),
        };
        if mask & 0x00FF != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IODIRA,
                (mask_set & 0xFF) as u8,
                (mask_clear & 0xFF) as u8,
            )?;
        }
        if mask & 0xFF00 != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IODIRB,
                (mask_set >> 8) as u8,
                (mask_clear >> 8) as u8,
            )?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus> crate::PortDriverPullUp for Driver<B> {
    fn set_pull_up(&mut self, mask: u32, enable: bool) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match enable {
            true => (mask as u16, 0),
            false => (0, mask as u16),
        };
        if mask & 0x00FF != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::GPPUA,
                (mask_set & 0xFF) as u8,
                (mask_clear & 0xFF) as u8,
            )?;
        }
        if mask & 0xFF00 != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::GPPUB,
                (mask_set >> 8) as u8,
                (mask_clear >> 8) as u8,
            )?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus> crate::PortDriverPolarity for Driver<B> {
    fn set_polarity(&mut self, mask: u32, inverted: bool) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match inverted {
            true => (mask as u16, 0),
            false => (0, mask as u16),
        };
        if mask & 0x00FF != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IPOLA,
                (mask_set & 0xFF) as u8,
                (mask_clear & 0xFF) as u8,
            )?;
        }
        if mask & 0xFF00 != 0 {
            self.bus.update_reg(
                self.addr,
                Regs::IPOLB,
                (mask_set >> 8) as u8,
                (mask_clear >> 8) as u8,
            )?;
        }
        Ok(())
    }
}

impl<B: Mcp23x17Bus> crate::PortDriverIrqMask for Driver<B> {
    fn configure_interrupts(&mut self, interrupt: crate::Interrupt) -> Result<(), Self::Error> {
        let (mask_set, mask_clear) = match interrupt {
            crate::Interrupt::Falling(s, c) => {
                self.int_changed &= !c as u16;
                self.int_changed |= s as u16;
                self.int_default &= !s as u16;
                self.int_default |= c as u16;
                self.int_level &= !s as u16; // inverse of how the MCP23x17 works, more intuitive
                self.int_level |= c as u16; 
                (s as u16, c as u16)
            },
            crate::Interrupt::Rising(s, c) => {
                self.int_changed &= !c as u16;
                self.int_changed |= s as u16;
                self.int_default &= !s as u16;
                self.int_default |= c as u16;
                self.int_level &= !s as u16; // inverse of how the MCP23x17 works, more intuitive
                self.int_level |= c as u16; 
                (s as u16, c as u16)
            },
            crate::Interrupt::Both(s, c) => {
                self.int_changed &= !c as u16;
                self.int_changed |= s as u16;
                self.int_default &= !c as u16;
                self.int_default |= s as u16;
                (s as u16, c as u16)
            },
            crate::Interrupt::High(s, c) => {
                self.int_changed &= !s as u16;
                self.int_changed |= !c as u16;
                self.int_default &= !c as u16;
                self.int_default |= s as u16;
                self.int_level &= !s as u16; // inverse of how the MCP23x17 works, more intuitive
                self.int_level |= c as u16; 
                (s as u16, c as u16)
            },
            crate::Interrupt::Low(s, c) => {
                self.int_changed &= !s as u16;
                self.int_changed |= !c as u16;
                self.int_default &= !c as u16;
                self.int_default |= s as u16;
                self.int_level &= !s as u16; // inverse of how the MCP23x17 works, more intuitive
                self.int_level |= c as u16; 
                (s as u16, c as u16)
            },
        };
        if mask_set & 0x00FF != 0 || mask_clear & 0x00FF != 0 {
            // make sure the relevant pins are set as inputs
            self.bus.update_reg(
                self.addr,
                Regs::IODIRA,
                (mask_set & 0xFF) as u8,
                (0) as u8,
            )?;
            match interrupt {
                crate::Interrupt::Both(_, _) => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONA, 
                        (mask_clear & 0xFF) as u8, 
                        (mask_set & 0xFF) as u8
                    )?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALA, 
                        (mask_clear & 0xFF) as u8, 
                        (mask_set & 0xFF) as u8
                    )?;
                },
                crate::Interrupt::High(_, _) | crate::Interrupt::Rising(_, _) => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONA, 
                        (mask_set & 0xFF) as u8, 
                        (mask_clear & 0xFF) as u8
                    )?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALA, 
                        (mask_clear & 0xFF) as u8, 
                        (mask_set & 0xFF) as u8
                    )?;
                },
                crate::Interrupt::Low(_, _) | crate::Interrupt::Falling(_, _) => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONA, 
                        (mask_set & 0xFF) as u8, 
                        (mask_clear & 0xFF) as u8
                    )?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALA, 
                        (mask_set & 0xFF) as u8, 
                        (mask_clear & 0xFF) as u8
                    )?;
                }
            }
            self.bus.update_reg(
                self.addr, 
                Regs::GPINTENA, 
                (mask_set & 0xFF) as u8, 
                (mask_clear & 0xFF) as u8
            )?;
        }
        if mask_set & 0xFF00 != 0 || mask_clear & 0xFF00 != 0 {
            // make sure the relevant pins are set as inputs
            self.bus.update_reg(
                self.addr,
                Regs::IODIRB,
                (mask_set >> 8) as u8,
                0 as u8,
            )?;
            match interrupt {
                crate::Interrupt::Both(_, _) | Interrupt::Falling(_, _) | Interrupt::Rising(_, _)=> {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_clear >> 8) as u8, 
                        (mask_set >> 8) as u8
                    )?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_clear >> 8) as u8, 
                        (mask_set >> 8) as u8
                    )?;
                },
                crate::Interrupt::High(_, _) => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_set >> 8) as u8, 
                        (mask_clear >> 8) as u8
                    )?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_clear >> 8) as u8, 
                        (mask_set >> 8) as u8
                    )?;
                },
                crate::Interrupt::Low(_, _) => {
                    self.bus.update_reg(
                        self.addr, 
                        Regs::INTCONB, 
                        (mask_set >> 8) as u8, 
                        (mask_clear >> 8) as u8
                    )?;
                    self.bus.update_reg(
                        self.addr, 
                        Regs::DEFVALB, 
                        (mask_set >> 8) as u8, 
                        (mask_clear >> 8) as u8
                    )?;
                }
            }
            self.bus.update_reg(
                self.addr, 
                Regs::GPINTENB, 
                (mask_set >> 8) as u8, 
                (mask_clear >> 8) as u8
            )?;
        }
        Ok(())
    }
}

impl <B: Mcp23x17Bus> PortDriverInterrupts for Driver<B> {
    fn fetch_interrupt_state(&mut self) -> Result<(), Self::Error> {
        let inta = self.bus.read_reg(self.addr, Regs::INTCAPA)?;
        let intb = self.bus.read_reg(self.addr, Regs::INTCAPB)?;
        let state = ((intb as u16) << 8) | inta as u16;
        let high_ints = self.int_default & state & self.int_level;
        let low_ints = self.int_default & !state & !self.int_level;
        let changed = self.out ^ state;
        let changed_ints = self.int_changed & changed;
        self.interrupted = high_ints | low_ints | changed_ints;
        self.changed = changed;
        self.out = state;
        Ok(())
    }

    fn query_pin_change(&mut self, mask: u32) -> u32 {
        let changed = self.changed as u32 & mask;
        self.changed = 0;
        changed
    }
}

impl <B: Mcp23x17Bus> PortDriverIrqState for Driver<B> {
    fn query_interrupt_state(&mut self, mask: u32, int_type: InterruptType) -> crate::Interrupt {        
        let interrupt = match int_type {
            InterruptType::Falling => {
                let falling_int = self.int_changed & self.changed & !self.out & mask as u16;
                crate::Interrupt::Falling(falling_int as u32, (self.out & falling_int) as u32)
            },
            InterruptType::Rising => {
                let rising_int = self.int_changed & self.changed & self.out & mask as u16;
                crate::Interrupt::Rising(rising_int as u32, (self.out & rising_int) as u32)
            },
            InterruptType::Both => {
                let both_ints = self.int_changed & self.changed & !self.int_default & mask as u16;
                crate::Interrupt::Both(both_ints as u32, (self.out & self.changed) as u32)
            },
            InterruptType::High => {
                let high_ints = self.int_default & self.interrupted & self.int_level & mask as u16;
                crate::Interrupt::High(high_ints as u32, (self.out & high_ints) as u32)
            },
            InterruptType::Low => {
                let low_ints = self.int_default & !self.interrupted & !self.int_level & mask as u16;
                crate::Interrupt::Low(low_ints as u32, (self.out & low_ints) as u32)
            },
        };
        self.changed = 0;
        self.interrupted = 0;
        interrupt
    }
}

impl <B: Mcp23x17Bus> PortDriverExtI for Driver<B> {
    fn configure_int_pin(&mut self, mask: u8, drive: crate::common::OutputDrive) -> Result<(), Self::Error> {
        let int_a = mask & 0b01 > 0;
        let int_b = mask & 0b10 > 0;
        let mask = match drive {
            crate::common::OutputDrive::ActiveLow => 0b000,
            crate::common::OutputDrive::ActiveHigh => 0b010,
            crate::common::OutputDrive::OpenDrain => 0b100,
        };
        if int_b {
            self.bus.update_reg(self.addr, Regs::IOCONB, mask, 0b000)?;
        }
        if int_a {
            self.bus.update_reg(self.addr, Regs::IOCONA, mask, 0b000)?;
        }
        Ok(())
    }
}


// We need these newtype wrappers since we can't implement `Mcp23x17Bus` for both `I2cBus` and `SpiBus`
// at the same time
pub struct Mcp23017Bus<I2C>(I2C);
pub struct Mcp23S17Bus<SPI>(SPI);

/// Special -Bus trait for the Mcp23x17 since the SPI version is a bit special/weird in terms of writing
/// SPI registers, which can't necessarily be generialized for other devices.
pub trait Mcp23x17Bus {
    type BusError;

    fn write_reg<R: Into<u8>>(&mut self, addr: u8, reg: R, value: u8)
        -> Result<(), Self::BusError>;
    fn read_reg<R: Into<u8>>(&mut self, addr: u8, reg: R) -> Result<u8, Self::BusError>;

    fn update_reg<R: Into<u8>>(
        &mut self,
        addr: u8,
        reg: R,
        mask_set: u8,
        mask_clear: u8,
    ) -> Result<(), Self::BusError> {
        let reg = reg.into();
        let mut val = self.read_reg(addr, reg)?;
        val |= mask_set;
        val &= !mask_clear;
        self.write_reg(addr, reg, val)?;
        Ok(())
    }
}

impl<SPI: crate::SpiBus> Mcp23x17Bus for Mcp23S17Bus<SPI> {
    type BusError = SPI::BusError;

    fn write_reg<R: Into<u8>>(
        &mut self,
        addr: u8,
        reg: R,
        value: u8,
    ) -> Result<(), Self::BusError> {
        self.0.write(&[0x40 | addr << 1, reg.into(), value])?;

        Ok(())
    }

    fn read_reg<R: Into<u8>>(&mut self, addr: u8, reg: R) -> Result<u8, Self::BusError> {
        let mut val = [0; 1];
        let write = [0x40 | addr << 1 | 0x1, reg.into()];
        let mut tx = [
            embedded_hal::spi::Operation::Write(&write),
            embedded_hal::spi::Operation::Read(&mut val),
        ];
        self.0.transaction(&mut tx)?;

        Ok(val[0])
    }
}

impl<I2C: crate::I2cBus> Mcp23x17Bus for Mcp23017Bus<I2C> {
    type BusError = I2C::BusError;

    fn write_reg<R: Into<u8>>(
        &mut self,
        addr: u8,
        reg: R,
        value: u8,
    ) -> Result<(), Self::BusError> {
        self.0.write_reg(addr, reg, value)
    }

    fn read_reg<R: Into<u8>>(&mut self, addr: u8, reg: R) -> Result<u8, Self::BusError> {
        self.0.read_reg(addr, reg)
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal_mock::eh1::{i2c as mock_i2c, spi as mock_spi};

    #[test]
    fn mcp23017() {
        let expectations = [
            // pin setup gpa0
            mock_i2c::Transaction::write_read(0x22, vec![0x00], vec![0xff]),
            mock_i2c::Transaction::write(0x22, vec![0x00, 0xfe]),
            // pin setup gpa7
            mock_i2c::Transaction::write_read(0x22, vec![0x00], vec![0xfe]),
            mock_i2c::Transaction::write(0x22, vec![0x00, 0x7e]),
            mock_i2c::Transaction::write_read(0x22, vec![0x00], vec![0x7e]),
            mock_i2c::Transaction::write(0x22, vec![0x00, 0xfe]),
            // pin setup gpb0
            mock_i2c::Transaction::write_read(0x22, vec![0x01], vec![0xff]),
            mock_i2c::Transaction::write(0x22, vec![0x01, 0xfe]),
            // pin setup gpb7
            mock_i2c::Transaction::write_read(0x22, vec![0x01], vec![0xfe]),
            mock_i2c::Transaction::write(0x22, vec![0x01, 0x7e]),
            mock_i2c::Transaction::write_read(0x22, vec![0x01], vec![0x7e]),
            mock_i2c::Transaction::write(0x22, vec![0x01, 0xfe]),
            // output gpa0, gpb0
            mock_i2c::Transaction::write(0x22, vec![0x12, 0x01]),
            mock_i2c::Transaction::write(0x22, vec![0x12, 0x00]),
            mock_i2c::Transaction::write(0x22, vec![0x13, 0x01]),
            mock_i2c::Transaction::write(0x22, vec![0x13, 0x00]),
            // input gpa7, gpb7
            mock_i2c::Transaction::write_read(0x22, vec![0x12], vec![0x80]),
            mock_i2c::Transaction::write_read(0x22, vec![0x12], vec![0x7f]),
            mock_i2c::Transaction::write_read(0x22, vec![0x13], vec![0x80]),
            mock_i2c::Transaction::write_read(0x22, vec![0x13], vec![0x7f]),
        ];
        let mut bus = mock_i2c::Mock::new(&expectations);

        let mut pca = super::Mcp23x17::new_mcp23017(bus.clone(), false, true, false);
        let pca_pins = pca.split();

        let mut gpa0 = pca_pins.gpa0.into_output().unwrap();
        let gpa7 = pca_pins.gpa7.into_output().unwrap();
        let gpa7 = gpa7.into_input().unwrap();

        let mut gpb0 = pca_pins.gpb0.into_output().unwrap();
        let gpb7 = pca_pins.gpb7.into_output().unwrap();
        let gpb7 = gpb7.into_input().unwrap();

        // output high and low
        gpa0.set_high().unwrap();
        gpa0.set_low().unwrap();
        gpb0.set_high().unwrap();
        gpb0.set_low().unwrap();

        // input high and low
        assert!(gpa7.is_high().unwrap());
        assert!(gpa7.is_low().unwrap());
        assert!(gpb7.is_high().unwrap());
        assert!(gpb7.is_low().unwrap());

        bus.done();
    }

    #[test]
    fn mcp23s17() {
        let expectations = [
            // pin setup gpa0
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x00]),
            mock_spi::Transaction::read(0xff),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x00, 0xfe]),
            mock_spi::Transaction::transaction_end(),
            // pin setup gpa7
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x00]),
            mock_spi::Transaction::read(0xfe),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x00, 0x7e]),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x00]),
            mock_spi::Transaction::read(0x7e),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x00, 0xfe]),
            mock_spi::Transaction::transaction_end(),
            // pin setup gpb0
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x01]),
            mock_spi::Transaction::read(0xff),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x01, 0xfe]),
            mock_spi::Transaction::transaction_end(), // pin setup gpb7
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x01]),
            mock_spi::Transaction::read(0xfe),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x01, 0x7e]),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x01]),
            mock_spi::Transaction::read(0x7e),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x01, 0xfe]),
            mock_spi::Transaction::transaction_end(),
            // output gpa0, gpb0
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x12, 0x01]),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x12, 0x00]),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x13, 0x01]),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x40, 0x13, 0x00]),
            mock_spi::Transaction::transaction_end(),
            // input gpa7, gpb7
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x12]),
            mock_spi::Transaction::read(0x80),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x12]),
            mock_spi::Transaction::read(0x7f),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x13]),
            mock_spi::Transaction::read(0x80),
            mock_spi::Transaction::transaction_end(),
            mock_spi::Transaction::transaction_start(),
            mock_spi::Transaction::write_vec(vec![0x41, 0x13]),
            mock_spi::Transaction::read(0x7f),
            mock_spi::Transaction::transaction_end(),
        ];
        let mut bus = mock_spi::Mock::new(&expectations);

        let mut pca = super::Mcp23x17::new_mcp23s17(bus.clone());
        let pca_pins = pca.split();

        let mut gpa0 = pca_pins.gpa0.into_output().unwrap();
        let gpa7 = pca_pins.gpa7.into_output().unwrap();
        let gpa7 = gpa7.into_input().unwrap();

        let mut gpb0 = pca_pins.gpb0.into_output().unwrap();
        let gpb7 = pca_pins.gpb7.into_output().unwrap();
        let gpb7 = gpb7.into_input().unwrap();

        // output high and low
        gpa0.set_high().unwrap();
        gpa0.set_low().unwrap();
        gpb0.set_high().unwrap();
        gpb0.set_low().unwrap();

        // input high and low
        assert!(gpa7.is_high().unwrap());
        assert!(gpa7.is_low().unwrap());
        assert!(gpb7.is_high().unwrap());
        assert!(gpb7.is_low().unwrap());

        bus.done();
    }
}
