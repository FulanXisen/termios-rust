use std::{os::fd::RawFd, io, ops::{Deref, DerefMut}};
use libc::{c_int,pid_t};
extern crate libc;
pub use crate::os::target::{cc_t,speed_t,tcflag_t}; 
pub mod ffi;
pub mod os;

/// A thin Wrapper, whoes impl OS-specific.
pub struct Termios {
    termios: crate::os::target::termios
}

impl Termios {
    /// Creates a `Termios` structure based on the current settings of a file descriptor.
    ///
    /// `fd` must be an open file descriptor for a terminal device.
    pub fn from_fd(fd: RawFd) -> io::Result<Self> {
        let mut termios = unsafe {std::mem::MaybeUninit::<Termios>::uninit().assume_init()};
        match tcgetattr(fd, &mut termios){
            Ok(_) => Ok(termios),
            Err(err) => Err(err),
        }
    }

    fn termios(&self) -> &crate::os::target::termios {
        &self.termios
    }

    fn termios_mut(&mut self)-> &mut crate::os::target::termios {
        &mut self.termios
    }
}

impl Deref for Termios {
    type Target = crate::os::target::termios;

    fn deref(&self) -> &crate::os::target::termios {
        self.termios()
    }
}

impl DerefMut for Termios {
    fn deref_mut(&mut self) -> &mut crate::os::target::termios {
        self.termios_mut()
    }
}

/// Gets the input baud rate stored in a `Termios` structure.
///
/// # Examples
///
/// ```
/// # use std::mem;
/// # use termios::{Termios,B9600,cfsetispeed,cfgetispeed};
/// # let mut termios = unsafe { mem::uninitialized() };
/// cfsetispeed(&mut termios, B9600).unwrap();
/// assert_eq!(cfgetispeed(&termios), B9600);
/// ```
pub fn cfgetispeed(termios: &Termios) -> speed_t {
    unsafe { ffi::cfgetispeed(termios.termios()) }
}

/// Gets the output baud rate stored in a `Termios` structure.
///
/// # Examples
///
/// ```
/// # use std::mem;
/// # use termios::{Termios,B9600,cfsetospeed,cfgetospeed};
/// # let mut termios = unsafe { mem::uninitialized() };
/// cfsetospeed(&mut termios, B9600).unwrap();
/// assert_eq!(cfgetospeed(&termios), B9600);
/// ```
pub fn cfgetospeed(termios: &Termios) -> speed_t {
    unsafe { ffi::cfgetospeed(termios.termios()) }
}

/// Sets the input baud rate.
///
/// This function only sets the necessary values on the given `Termios` structure. The settings are
/// applied by a subsequent call to [`tcsetattr()`](fn.tcsetattr.html).
///
/// # Parameters
///
/// * `termios` should be a mutable reference to a `Termios` structure.
/// * `speed` should be one of the baud rate constants:
///   - `B0`
///   - `B50`
///   - `B75`
///   - `B110`
///   - `B134`
///   - `B150`
///   - `B200`
///   - `B300`
///   - `B600`
///   - `B1200`
///   - `B1800`
///   - `B2400`
///   - `B4800`
///   - `B9600`
///   - `B19200`
///   - `B38400`
///   - any OS-specific baud rate defined in [`termios::os`](os/index.html).
///
/// A value of `B0` for `speed` sets the input baud rate to be the same as the output baud rate.
///
/// # Examples
///
/// ```
/// # use std::mem;
/// # use termios::{Termios,B9600,cfsetispeed,cfgetispeed};
/// # let mut termios = unsafe { mem::uninitialized() };
/// cfsetispeed(&mut termios, B9600).unwrap();
/// assert_eq!(cfgetispeed(&termios), B9600);
/// ```
pub fn cfsetispeed(termios: &mut Termios, speed: speed_t) -> io::Result<()> {
    {
        let result = unsafe { ffi::cfsetispeed(termios.termios_mut(), speed) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Sets the output baud rate.
///
/// This function only sets the necessary values on the given `Termios` structure. The settings are
/// applied on a successful call to [`tcsetattr()`](fn.tcsetattr.html).
///
/// # Parameters
///
/// * `termios` should be a mutable reference to a `Termios` structure.
/// * `speed` should be one of the baud rate constants:
///   - `B0` (hang up)
///   - `B50`
///   - `B75`
///   - `B110`
///   - `B134`
///   - `B150`
///   - `B200`
///   - `B300`
///   - `B600`
///   - `B1200`
///   - `B1800`
///   - `B2400`
///   - `B4800`
///   - `B9600`
///   - `B19200`
///   - `B38400`
///   - any OS-specific baud rate defined in [`termios::os`](os/index.html).
///
/// A value of `B0` for `speed` deasserts the modem control lines when applied with
/// [`tcsetattr()`](fn.tcsetattr.html).  This normally has the effect of disconnecting the line.
///
/// # Examples
///
/// ```
/// # use std::mem;
/// # use termios::{Termios,B9600,cfsetospeed,cfgetospeed};
/// # let mut termios = unsafe { mem::uninitialized() };
/// cfsetospeed(&mut termios, B9600).unwrap();
/// assert_eq!(cfgetospeed(&termios), B9600);
/// ```
pub fn cfsetospeed(termios: &mut Termios, speed: speed_t) -> io::Result<()> {
    {
        let result = unsafe { ffi::cfsetospeed(termios.termios_mut(), speed) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Sets input and output baud rates.
///
/// This function only sets the necessary values on the given `Termios` structure. The settings are
/// applied on a successful call to [`tcsetattr()`](fn.tcsetattr.html).
///
/// # Parameters
///
/// * `termios` should be a mutable reference to a `Termios` structure.
/// * `speed` should be one of the baud rate constants:
///   - `B0`
///   - `B50`
///   - `B75`
///   - `B110`
///   - `B134`
///   - `B150`
///   - `B200`
///   - `B300`
///   - `B600`
///   - `B1200`
///   - `B1800`
///   - `B2400`
///   - `B4800`
///   - `B9600`
///   - `B19200`
///   - `B38400`
///   - any OS-specific baud rate defined in [`termios::os`](os/index.html).
///
/// # Examples
///
/// ```
/// # use std::mem;
/// # use termios::{Termios,B9600,cfsetspeed,cfgetispeed,cfgetospeed};
/// # let mut termios = unsafe { mem::uninitialized() };
/// cfsetspeed(&mut termios, B9600).unwrap();
/// assert_eq!(cfgetispeed(&termios), B9600);
/// assert_eq!(cfgetospeed(&termios), B9600);
/// ```
///
/// # Portability
///
/// This function is not part of the IEEE Std 1003.1 ("POSIX.1") specification, but it is available
/// on Linux, BSD, and OS X.
pub fn cfsetspeed(termios: &mut Termios, speed: speed_t) -> io::Result<()> {
    {
        let result = unsafe { ffi::cfsetspeed(termios.termios_mut(), speed) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Sets flags to disable all input and output processing.
///
/// This function only sets the necessary values on the given `Termios` structure. The settings are
/// applied on a successful call to [`tcsetattr()`](fn.tcsetattr.html).
///
/// # Portability
///
/// This function is not part of the IEEE Std 1003.1 ("POSIX.1") specification, but it is available
/// on Linux, BSD, and OS X.
pub fn cfmakeraw(termios: &mut Termios) {
    unsafe { ffi::cfmakeraw(termios.termios_mut()) };
}

/// Blocks until all output written to the file descriptor is transmitted.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a terminal.
pub fn tcdrain(fd: RawFd) -> io::Result<()> {
    {
        let result = unsafe { ffi::tcdrain(fd) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Suspends or restarts transmission or reception of data.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a terminal.
/// * `action` should be one of the following constants:
///   - `TCOOFF` suspends output.
///   - `TCOON` restarts output.
///   - `TCIOFF` transmits a STOP character, intended to cause the remote device to stop
///     transmitting.
///   - `TCION` transmits a START character, intended to cause the remote device to resume
///     transmitting.
pub fn tcflow(fd: RawFd, action: c_int) -> io::Result<()> {
    {
        let result = unsafe { ffi::tcflow(fd, action) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Discards data waiting in the terminal device's buffers.
///
/// `tcflush()` discards data that has been written to the device by an application but has not yet
/// been transmitted by the hardware or data that has been received by the hardware but has not yet
/// been read by an application.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a terminal.
/// * `queue_selector` should be one of:
///   - `TCIFLUSH` to discard data received but not read.
///   - `TCOFLUSH` to discard data written but not transmitted.
///   - `TCIOFLUSH` to discard both data received but not read and data written but not
///     transmitted.
pub fn tcflush(fd: RawFd, queue_selector: c_int) -> io::Result<()> {
    {
        let result = unsafe { ffi::tcflush(fd, queue_selector) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Populates a `Termios` structure with parameters associated with a terminal.
///
/// Upon successful completion, the `Termios` structure referred to by the `termios` parameter will
/// contain the parameters associated with the terminal device referred to by `fd`.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a terminal.
/// * `termios` should be a mutable reference to the `Termios` structure that will hold the
///   terminal device's parameters.
pub fn tcgetattr(fd: RawFd, termios: &mut Termios) -> io::Result<()> {
    {
        let result = unsafe { ffi::tcgetattr(fd, termios.termios_mut()) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
    }
}

/// Sets a terminal device's parameters.
///
/// `tcsetattr()` returns successfully if it was able to perform any of the requested actions, even
/// if other requested actions could not be performed. It will set all attributes that the
/// implementation supports and leave others unchanged. The `Termios` structure will not be updated
/// to reflect the changes that were applied.
///
/// In order to determine which parameters were applied to the terminal device, an application
/// should use [`tcgetattr()`](fn.tcgetattr.html) to obtain the latest state of the terminal
/// device. In particular, when attempting to change baud rates, [`tcgetattr()`](fn.tcgetattr.html)
/// can be used to determine which baud rates were actually selected.
///
/// If none of the requested actions could be performed, then `tcsetattr()` returns an error.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a terminal.
/// * `action` should be one of the constants:
///   - `TCSANOW` applies the change immediately.
///   - `TCSADRAIN` applies the change after all output previously written to `fd` is transmitted.
///     This mode should be used when changing parameters that affect output.
///   - `TCSAFLUSH` applies the change after all output previously written to `fd` is transmitted.
///     All data received but not read is discarded before applying the change.
/// * `termios` should be a mutable reference to a `Termios` structure containing the parameters to
///   apply to the terminal device.
pub fn tcsetattr(fd: RawFd, action: c_int, termios: &Termios) -> io::Result<()> {
        let result = unsafe { ffi::tcsetattr(fd, action, termios.termios()) };
        match result {
            0 => Ok(()),
            _ => Err(io::Error::last_os_error())
        }
}

/// Transmits data to generate a break condition.
///
/// If the terminal device is using asynchronous data transmission, `tcsendbreak()` transmits a
/// continuous stream of zero bits for a specific duration.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a terminal.
/// * `duration` controls the duration of the transmitted zero bits. A value of 0 causes a
///   transmission between 0.25 and 0.5 seconds. A value other than 0 causes a transmission for an
///   implementation-defined period of time.
pub fn tcsendbreak(fd: RawFd, duration: c_int) -> io::Result<()> {
    match unsafe { ffi::tcsendbreak(fd, duration) } {
        0 => Ok(()),
        _ => Err(io::Error::last_os_error())
    }
}

/// Returns the process group ID of the controlling terminal's session.
///
/// # Parameters
///
/// * `fd` should be an open file descriptor associated with a controlling terminal.
pub fn tcgetsid(fd: RawFd) -> pid_t {
    unsafe { ffi::tcgetsid(fd) }
}

#[inline]
fn io_result(result: c_int) -> io::Result<()> {
    match result {
        0 => Ok(()),
        _ => Err(io::Error::last_os_error())
    }
}