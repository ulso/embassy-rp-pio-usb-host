//! Transport-independent protocol helpers used by the BleuIO example.
//!
//! This module deliberately knows nothing about USB or CDC-ACM. Callers feed
//! arbitrary byte slices received from a byte stream into the accumulators,
//! so USB packet boundaries do not affect protocol parsing.
//!
//! The implementation uses only `core` and fixed-size storage, making it
//! suitable for the crate's `no_std` firmware target.

use embedded_io_async::{Read, Write};

/// Basic command used to verify that a BleuIO device is responsive.
pub const ATTENTION_COMMAND: &[u8] = b"AT\r\n";

/// Command used to select the BLE central role.
pub const CENTRAL_ROLE_COMMAND: &[u8] = b"AT+CENTRAL\r\n";

/// One-second default-mode GAP scan command.
pub const GAP_SCAN_COMMAND: &[u8] = b"AT+GAPSCAN=1\r\n";

/// Bounded storage for a basic BleuIO AT command response.
pub const MAX_AT_RESPONSE_BYTES: usize = 64;

/// Maximum buffered length of one default-mode BleuIO scan output line.
pub const MAX_SCAN_LINE_BYTES: usize = 128;

/// Capacity of the persistent receive buffer owned by [`BleuIo`].
///
/// Parsers consume bytes as they inspect them. This buffer therefore normally
/// contains only bytes received after the terminal line of the preceding
/// response.
#[allow(dead_code)] // Public example API and fixed storage used by BleuIo.
pub const BLEUIO_RX_BUFFER_BYTES: usize = 256;

/// Progress made by a streaming response parser.
#[allow(dead_code)] // Public example API; also exercised by host tests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParseProgress<T> {
    /// More bytes are required.
    Pending {
        /// Number of bytes consumed from the supplied input.
        consumed: usize,
    },
    /// The response is complete.
    Complete {
        /// Number of bytes consumed through the response's terminal marker.
        consumed: usize,
        /// Parsed response value.
        output: T,
    },
}

#[allow(dead_code)]
impl<T> ParseProgress<T> {
    const fn consumed(&self) -> usize {
        match self {
            Self::Pending { consumed } | Self::Complete { consumed, .. } => *consumed,
        }
    }
}

/// Incremental parser accepted by [`BleuIo::read_response`].
///
/// A parser may stop before the end of `input` when it reaches a terminal
/// marker. [`BleuIo`] retains the unconsumed suffix for the following command.
/// When returning [`ParseProgress::Pending`] for non-empty input, the parser
/// must consume at least one byte.
#[allow(dead_code)] // Public example API; also exercised by host tests.
pub trait ResponseParser {
    /// Value produced when the response is complete.
    type Output;
    /// Protocol-specific parse error.
    type Error;

    /// Consume as much of `input` as belongs to this response.
    fn parse(&mut self, input: &[u8]) -> Result<ParseProgress<Self::Output>, Self::Error>;
}

/// Stream-level failure while communicating with a BleuIO device.
#[allow(dead_code)] // Public example API; also exercised by host tests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BleuIoTransportError<E> {
    /// Error returned by the underlying byte stream.
    Io(E),
    /// The stream returned zero from a non-empty write.
    WriteZero,
    /// The stream reached end-of-file before the response completed.
    UnexpectedEof,
    /// A stream or parser returned an impossible byte count.
    ContractViolation,
}

/// Failure from a BleuIO command exchange.
#[allow(dead_code)] // Public example API; also exercised by host tests.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BleuIoError<E, P> {
    /// Transport-level failure.
    Transport(BleuIoTransportError<E>),
    /// Protocol-specific parse failure.
    Protocol(P),
    /// The device emitted an exact `ERROR` terminal line.
    DeviceError,
}

/// Transport-independent asynchronous BleuIO command client.
///
/// `S` may be any `embedded-io-async` byte stream, including a CDC-ACM host
/// class instance. The client owns both the stream and a persistent receive
/// buffer so bytes following one terminal response line remain available to
/// the next command.
#[allow(dead_code)] // Public example API; exercised by host tests during the migration.
pub struct BleuIo<S> {
    stream: S,
    rx: [u8; BLEUIO_RX_BUFFER_BYTES],
    rx_start: usize,
    rx_end: usize,
}

#[allow(dead_code)]
impl<S> BleuIo<S> {
    /// Wrap a byte stream in a BleuIO command client.
    pub const fn new(stream: S) -> Self {
        Self {
            stream,
            rx: [0; BLEUIO_RX_BUFFER_BYTES],
            rx_start: 0,
            rx_end: 0,
        }
    }

    /// Borrow the underlying byte stream.
    pub const fn get_ref(&self) -> &S {
        &self.stream
    }

    /// Unwrap the underlying byte stream.
    pub fn into_inner(self) -> S {
        self.stream
    }

    /// Return the number of bytes retained for a subsequent response.
    pub const fn buffered_len(&self) -> usize {
        self.rx_end - self.rx_start
    }

    fn consume_buffered(&mut self, count: usize) {
        self.rx_start += count;
        if self.rx_start == self.rx_end {
            self.rx_start = 0;
            self.rx_end = 0;
        }
    }
}

#[allow(dead_code)]
impl<S> BleuIo<S>
where
    S: Read + Write,
{
    /// Write and flush one complete command, handling partial writes.
    pub async fn write_command(
        &mut self,
        command: &[u8],
    ) -> Result<(), BleuIoTransportError<S::Error>> {
        let mut written = 0;
        while written < command.len() {
            let remaining = &command[written..];
            let count = self
                .stream
                .write(remaining)
                .await
                .map_err(BleuIoTransportError::Io)?;
            if count == 0 {
                return Err(BleuIoTransportError::WriteZero);
            }
            if count > remaining.len() {
                return Err(BleuIoTransportError::ContractViolation);
            }
            written += count;
        }
        self.stream.flush().await.map_err(BleuIoTransportError::Io)
    }

    /// Read until `parser` completes, retaining bytes beyond its terminal
    /// marker for the next response.
    pub async fn read_response<P>(
        &mut self,
        parser: &mut P,
    ) -> Result<P::Output, BleuIoError<S::Error, P::Error>>
    where
        P: ResponseParser,
    {
        loop {
            if self.rx_start != self.rx_end {
                let available = self.rx_end - self.rx_start;
                let progress = parser
                    .parse(&self.rx[self.rx_start..self.rx_end])
                    .map_err(BleuIoError::Protocol)?;
                let consumed = progress.consumed();
                if consumed > available
                    || (consumed == 0 && matches!(&progress, ParseProgress::Pending { .. }))
                {
                    return Err(BleuIoError::Transport(
                        BleuIoTransportError::ContractViolation,
                    ));
                }
                self.consume_buffered(consumed);

                match progress {
                    ParseProgress::Pending { .. } => continue,
                    ParseProgress::Complete { output, .. } => return Ok(output),
                }
            }

            let count = self
                .stream
                .read(&mut self.rx[self.rx_end..])
                .await
                .map_err(|error| BleuIoError::Transport(BleuIoTransportError::Io(error)))?;
            if count == 0 {
                return Err(BleuIoError::Transport(BleuIoTransportError::UnexpectedEof));
            }
            if count > self.rx.len() - self.rx_end {
                return Err(BleuIoError::Transport(
                    BleuIoTransportError::ContractViolation,
                ));
            }
            self.rx_end += count;
        }
    }

    /// Write `command` and parse its response with an arbitrary incremental
    /// parser.
    pub async fn command<P>(
        &mut self,
        command: &[u8],
        parser: &mut P,
    ) -> Result<P::Output, BleuIoError<S::Error, P::Error>>
    where
        P: ResponseParser,
    {
        self.write_command(command)
            .await
            .map_err(BleuIoError::Transport)?;
        self.read_response(parser).await
    }

    async fn basic_command(
        &mut self,
        command: &[u8],
    ) -> Result<(), BleuIoError<S::Error, AtResponseError>> {
        let mut parser = AtResponseAccumulator::new();
        match self.command(command, &mut parser).await? {
            AtResponseStatus::Ok => Ok(()),
            AtResponseStatus::Error => Err(BleuIoError::DeviceError),
            AtResponseStatus::Pending => Err(BleuIoError::Transport(
                BleuIoTransportError::ContractViolation,
            )),
        }
    }

    /// Send `AT` and require an exact `OK` terminal response.
    pub async fn attention(&mut self) -> Result<(), BleuIoError<S::Error, AtResponseError>> {
        self.basic_command(ATTENTION_COMMAND).await
    }

    /// Select the BLE central role and require an exact `OK` response.
    pub async fn set_central(&mut self) -> Result<(), BleuIoError<S::Error, AtResponseError>> {
        self.basic_command(CENTRAL_ROLE_COMMAND).await
    }

    /// Run the one-second default-mode GAP scan and return its first device.
    ///
    /// The command itself supplies the scan duration. Applications that also
    /// need a transport deadline can wrap this future in their executor's
    /// timeout primitive.
    pub async fn gap_scan(&mut self) -> Result<ScanResult, BleuIoError<S::Error, ScanError>> {
        let mut parser = ScanAccumulator::new();
        self.command(GAP_SCAN_COMMAND, &mut parser).await
    }
}

/// State of the response to a basic BleuIO AT command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AtResponseStatus {
    /// No terminal response line has been received yet.
    Pending,
    /// An exact `OK\r\n` terminal line was received.
    Ok,
    /// An exact `ERROR\r\n` terminal line was received.
    Error,
}

/// Error while accumulating a bounded BleuIO AT response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AtResponseError {
    /// The response did not fit in [`MAX_AT_RESPONSE_BYTES`].
    Overflow,
}

/// Packet-boundary-independent accumulator for a CRLF-delimited AT response.
///
/// Echoed command lines and empty lines are retained but ignored. Completion
/// requires an exact `OK` line; `NOTOK`, `OKAY`, and lowercase `ok` do not
/// match. An exact `ERROR` line is reported separately.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AtResponseAccumulator {
    bytes: [u8; MAX_AT_RESPONSE_BYTES],
    len: usize,
    scan_index: usize,
    line_start: usize,
    status: AtResponseStatus,
}

impl AtResponseAccumulator {
    /// Construct an empty response accumulator.
    pub const fn new() -> Self {
        Self {
            bytes: [0; MAX_AT_RESPONSE_BYTES],
            len: 0,
            scan_index: 0,
            line_start: 0,
            status: AtResponseStatus::Pending,
        }
    }

    /// Append arbitrary stream bytes and scan complete CRLF-delimited lines.
    ///
    /// Only bytes through a terminal `OK` or `ERROR` line are retained. Bytes
    /// following the terminal line in the same input slice are not part of
    /// this response and are left outside the accumulator's scope.
    #[allow(dead_code)] // Kept as a packet-oriented parser/test convenience.
    pub fn push(&mut self, payload: &[u8]) -> Result<AtResponseStatus, AtResponseError> {
        self.push_with_consumed(payload).map(|(status, _)| status)
    }

    fn push_with_consumed(
        &mut self,
        payload: &[u8],
    ) -> Result<(AtResponseStatus, usize), AtResponseError> {
        if self.status != AtResponseStatus::Pending {
            return Ok((self.status, 0));
        }
        let original = self.clone();

        for (payload_index, &byte) in payload.iter().enumerate() {
            if self.len == self.bytes.len() {
                *self = original;
                return Err(AtResponseError::Overflow);
            }
            self.bytes[self.len] = byte;
            self.len += 1;

            while self.scan_index < self.len {
                if self.bytes[self.scan_index] == b'\n' {
                    let has_cr = self.scan_index > self.line_start
                        && self.bytes[self.scan_index - 1] == b'\r';
                    let line_end = if has_cr {
                        self.scan_index - 1
                    } else {
                        self.scan_index
                    };
                    let line = &self.bytes[self.line_start..line_end];
                    let status = if has_cr && line == b"OK" {
                        AtResponseStatus::Ok
                    } else if has_cr && line == b"ERROR" {
                        AtResponseStatus::Error
                    } else {
                        AtResponseStatus::Pending
                    };
                    self.line_start = self.scan_index + 1;
                    if status != AtResponseStatus::Pending {
                        self.status = status;
                        return Ok((status, payload_index + 1));
                    }
                }
                self.scan_index += 1;
            }
        }

        Ok((AtResponseStatus::Pending, payload.len()))
    }

    /// Return the number of retained bytes.
    #[allow(dead_code)] // Useful to parser tests and diagnostics.
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Return whether no response bytes have been retained.
    #[allow(dead_code)] // Useful to parser tests and diagnostics.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl Default for AtResponseAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl ResponseParser for AtResponseAccumulator {
    type Output = AtResponseStatus;
    type Error = AtResponseError;

    fn parse(&mut self, input: &[u8]) -> Result<ParseProgress<Self::Output>, Self::Error> {
        let (status, consumed) = self.push_with_consumed(input)?;
        Ok(match status {
            AtResponseStatus::Pending => ParseProgress::Pending { consumed },
            AtResponseStatus::Ok | AtResponseStatus::Error => ParseProgress::Complete {
                consumed,
                output: status,
            },
        })
    }
}

/// First structured device found by a default-mode BleuIO GAP scan.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ScanResult {
    /// Result index assigned by the BleuIO firmware.
    pub index: u8,
    /// BLE address type reported by BleuIO (`0` public or `1` random).
    pub address_type: u8,
    /// Bluetooth device address in the order printed by BleuIO.
    pub address: [u8; 6],
    /// Received signal strength in dBm.
    pub rssi: i8,
}

/// Progress of a packet-boundary-independent BleuIO GAP scan.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScanStatus {
    /// The scan transcript has not reached its terminal line.
    Pending,
    /// `SCAN COMPLETE` was received after at least one valid result.
    Complete,
}

/// Error while consuming default-mode BleuIO GAP scan output.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScanError {
    /// The device emitted an exact `ERROR` line.
    DeviceError,
    /// One response line exceeded [`MAX_SCAN_LINE_BYTES`].
    LineOverflow,
    /// A line shaped like a device result contained invalid fields.
    MalformedDeviceLine,
    /// A device result arrived before `SCANNING...`.
    UnexpectedDevice,
    /// `SCAN COMPLETE` arrived before `SCANNING...`.
    UnexpectedComplete,
    /// The scan completed without a valid device result.
    NoDevice,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ScanLine {
    Ignore,
    Scanning,
    Device(ScanResult),
    Complete,
    Error,
}

/// Streaming parser for a timed default-mode `AT+GAPSCAN` response.
///
/// Only one CRLF-delimited line is retained at a time, so an arbitrary number
/// of scan results can pass through fixed storage. The first syntactically
/// valid result is preserved for the caller.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ScanAccumulator {
    line: [u8; MAX_SCAN_LINE_BYTES],
    line_len: usize,
    saw_scanning: bool,
    first_result: Option<ScanResult>,
    complete: bool,
}

impl ScanAccumulator {
    /// Construct an empty scan accumulator.
    pub const fn new() -> Self {
        Self {
            line: [0; MAX_SCAN_LINE_BYTES],
            line_len: 0,
            saw_scanning: false,
            first_result: None,
            complete: false,
        }
    }

    /// Append arbitrary stream bytes and consume complete CRLF lines.
    #[allow(dead_code)] // Kept as a packet-oriented parser/test convenience.
    pub fn push(&mut self, payload: &[u8]) -> Result<ScanStatus, ScanError> {
        self.push_with_consumed(payload).map(|(status, _)| status)
    }

    fn push_with_consumed(&mut self, payload: &[u8]) -> Result<(ScanStatus, usize), ScanError> {
        if self.complete {
            return Ok((ScanStatus::Complete, 0));
        }

        for (payload_index, &byte) in payload.iter().enumerate() {
            if byte == b'\n' && self.line_len != 0 && self.line[self.line_len - 1] == b'\r' {
                let line = classify_scan_line(&self.line[..self.line_len - 1])?;
                self.line_len = 0;

                match line {
                    ScanLine::Ignore => {}
                    ScanLine::Scanning => self.saw_scanning = true,
                    ScanLine::Device(result) => {
                        if !self.saw_scanning {
                            return Err(ScanError::UnexpectedDevice);
                        }
                        if self.first_result.is_none() {
                            self.first_result = Some(result);
                        }
                    }
                    ScanLine::Complete => {
                        if !self.saw_scanning {
                            return Err(ScanError::UnexpectedComplete);
                        }
                        if self.first_result.is_none() {
                            return Err(ScanError::NoDevice);
                        }
                        self.complete = true;
                        return Ok((ScanStatus::Complete, payload_index + 1));
                    }
                    ScanLine::Error => return Err(ScanError::DeviceError),
                }
                continue;
            }

            if self.line_len == self.line.len() {
                return Err(ScanError::LineOverflow);
            }
            self.line[self.line_len] = byte;
            self.line_len += 1;
        }

        Ok((ScanStatus::Pending, payload.len()))
    }

    /// Return the first valid device result, if one has been received.
    pub const fn first_result(&self) -> Option<ScanResult> {
        self.first_result
    }
}

impl Default for ScanAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl ResponseParser for ScanAccumulator {
    type Output = ScanResult;
    type Error = ScanError;

    fn parse(&mut self, input: &[u8]) -> Result<ParseProgress<Self::Output>, Self::Error> {
        let (status, consumed) = self.push_with_consumed(input)?;
        match status {
            ScanStatus::Pending => Ok(ParseProgress::Pending { consumed }),
            ScanStatus::Complete => self
                .first_result()
                .map(|output| ParseProgress::Complete { consumed, output })
                .ok_or(ScanError::NoDevice),
        }
    }
}

fn classify_scan_line(line: &[u8]) -> Result<ScanLine, ScanError> {
    if line == b"SCANNING..." {
        return Ok(ScanLine::Scanning);
    }
    if line == b"SCAN COMPLETE" {
        return Ok(ScanLine::Complete);
    }
    if line == b"ERROR" {
        return Ok(ScanLine::Error);
    }

    if line.first() != Some(&b'[') {
        return Ok(ScanLine::Ignore);
    }
    let Some(index_end) = line.iter().position(|&byte| byte == b']') else {
        return Ok(ScanLine::Ignore);
    };
    let after_index = &line[index_end + 1..];
    let Some(rest) = after_index.strip_prefix(b" Device: ") else {
        return Ok(ScanLine::Ignore);
    };

    parse_device_line(&line[1..index_end], rest)
        .map(ScanLine::Device)
        .ok_or(ScanError::MalformedDeviceLine)
}

fn parse_device_line(index: &[u8], line: &[u8]) -> Option<ScanResult> {
    if !(1..=3).contains(&index.len()) {
        return None;
    }
    let index = parse_decimal_u8(index)?;
    if line.first() != Some(&b'[') {
        return None;
    }
    let address_type_end = line.iter().position(|&byte| byte == b']')?;
    if address_type_end != 2 {
        return None;
    }
    let address_type = parse_decimal_u8(&line[1..address_type_end])?;
    if address_type > 1 {
        return None;
    }

    let mut cursor = address_type_end + 1;
    let mut address = [0_u8; 6];
    let mut octet = 0;
    while octet < address.len() {
        let high = hex_nibble(*line.get(cursor)?)?;
        let low = hex_nibble(*line.get(cursor + 1)?)?;
        address[octet] = (high << 4) | low;
        cursor += 2;
        if octet + 1 != address.len() {
            if line.get(cursor) != Some(&b':') {
                return None;
            }
            cursor += 1;
        }
        octet += 1;
    }

    let remaining = line.get(cursor..)?;
    if remaining.first() != Some(&b' ') {
        return None;
    }
    let remaining = trim_ascii_spaces_start(remaining);
    let remaining = remaining.strip_prefix(b"RSSI:")?;
    if remaining.first() != Some(&b' ') {
        return None;
    }
    let remaining = trim_ascii_spaces_start(remaining);
    let (rssi, consumed) = parse_leading_i8(remaining)?;
    let suffix = &remaining[consumed..];
    if let Some(first) = suffix.first()
        && !matches!(first, b' ' | b'(' | b'[')
    {
        return None;
    }

    Some(ScanResult {
        index,
        address_type,
        address,
        rssi,
    })
}

fn parse_decimal_u8(bytes: &[u8]) -> Option<u8> {
    if bytes.is_empty() {
        return None;
    }
    let mut value = 0_u8;
    for &byte in bytes {
        if !byte.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?.checked_add(byte - b'0')?;
    }
    Some(value)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn trim_ascii_spaces_start(mut bytes: &[u8]) -> &[u8] {
    while bytes.first() == Some(&b' ') {
        bytes = &bytes[1..];
    }
    bytes
}

fn parse_leading_i8(bytes: &[u8]) -> Option<(i8, usize)> {
    let (negative, digits) = match bytes.first()? {
        b'-' => (true, &bytes[1..]),
        b'+' => (false, &bytes[1..]),
        _ => (false, bytes),
    };

    let mut value = 0_i16;
    let mut digit_count = 0;
    for &byte in digits {
        if !byte.is_ascii_digit() {
            break;
        }
        value = value.checked_mul(10)?.checked_add(i16::from(byte - b'0'))?;
        digit_count += 1;
    }
    if digit_count == 0 {
        return None;
    }
    if negative {
        value = -value;
    }
    let consumed = digit_count + usize::from(matches!(bytes.first(), Some(b'-' | b'+')));
    Some((i8::try_from(value).ok()?, consumed))
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::convert::Infallible;
    use core::future::Future;
    use core::task::{Context, Poll};
    use embedded_io_async::ErrorType;
    use std::collections::VecDeque;
    use std::task::Waker;
    use std::vec::Vec;

    extern crate std;

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut context = Context::from_waker(Waker::noop());
        let mut future = core::pin::pin!(future);
        loop {
            match future.as_mut().poll(&mut context) {
                Poll::Ready(output) => return output,
                Poll::Pending => std::thread::yield_now(),
            }
        }
    }

    struct FakeStream {
        reads: VecDeque<Vec<u8>>,
        writes: Vec<u8>,
        max_write: usize,
        read_calls: usize,
        write_calls: usize,
        flushes: usize,
    }

    impl FakeStream {
        fn new(reads: &[&[u8]], max_write: usize) -> Self {
            Self {
                reads: reads.iter().map(|chunk| chunk.to_vec()).collect(),
                writes: Vec::new(),
                max_write,
                read_calls: 0,
                write_calls: 0,
                flushes: 0,
            }
        }
    }

    impl ErrorType for FakeStream {
        type Error = Infallible;
    }

    impl Read for FakeStream {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            self.read_calls += 1;
            while self.reads.front().is_some_and(Vec::is_empty) {
                self.reads.pop_front();
            }
            let Some(chunk) = self.reads.front_mut() else {
                return Ok(0);
            };
            let count = chunk.len().min(buf.len());
            buf[..count].copy_from_slice(&chunk[..count]);
            chunk.drain(..count);
            Ok(count)
        }
    }

    impl Write for FakeStream {
        async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.write_calls += 1;
            let count = buf.len().min(self.max_write);
            self.writes.extend_from_slice(&buf[..count]);
            Ok(count)
        }

        async fn flush(&mut self) -> Result<(), Self::Error> {
            self.flushes += 1;
            Ok(())
        }
    }

    #[test]
    fn bleuio_attention_handles_fragmented_reads() {
        let reads: &[&[u8]] = &[b"A", b"T\r", b"\n", b"O", b"K\r", b"\n"];
        let mut bleuio = BleuIo::new(FakeStream::new(reads, usize::MAX));

        block_on(bleuio.attention()).unwrap();

        let stream = bleuio.into_inner();
        assert_eq!(stream.writes, ATTENTION_COMMAND);
        assert_eq!(stream.read_calls, reads.len());
        assert_eq!(stream.flushes, 1);
    }

    #[test]
    fn bleuio_retries_partial_writes_until_the_command_is_complete() {
        let reads: &[&[u8]] = &[b"AT+CENTRAL\r\nOK\r\n"];
        let mut bleuio = BleuIo::new(FakeStream::new(reads, 2));

        block_on(bleuio.set_central()).unwrap();

        let stream = bleuio.into_inner();
        assert_eq!(stream.writes, CENTRAL_ROLE_COMMAND);
        assert_eq!(stream.write_calls, CENTRAL_ROLE_COMMAND.len().div_ceil(2));
        assert_eq!(stream.flushes, 1);
    }

    #[test]
    fn bleuio_preserves_trailing_bytes_for_the_next_command() {
        let second_response = b"AT+CENTRAL\r\nOK\r\n";
        let reads: &[&[u8]] = &[b"AT\r\nOK\r\nAT+CENTRAL\r\nOK\r\n"];
        let mut bleuio = BleuIo::new(FakeStream::new(reads, 3));

        block_on(bleuio.attention()).unwrap();
        assert_eq!(bleuio.buffered_len(), second_response.len());
        let read_calls_after_first = bleuio.get_ref().read_calls;

        block_on(bleuio.set_central()).unwrap();
        assert_eq!(bleuio.buffered_len(), 0);
        assert_eq!(bleuio.get_ref().read_calls, read_calls_after_first);

        let stream = bleuio.into_inner();
        let mut expected_writes = Vec::new();
        expected_writes.extend_from_slice(ATTENTION_COMMAND);
        expected_writes.extend_from_slice(CENTRAL_ROLE_COMMAND);
        assert_eq!(stream.writes, expected_writes);
    }

    #[test]
    fn bleuio_gap_scan_uses_the_streaming_scan_parser() {
        let reads: &[&[u8]] = &[
            b"AT+GAP",
            b"SCAN=1\r\nSCANN",
            b"ING...\r\n[01] Device: [1]30:63:C5:D0:B1:DE RSSI: ",
            b"-38\r\nSCAN COMPLETE\r\n",
        ];
        let mut bleuio = BleuIo::new(FakeStream::new(reads, 4));

        let result = block_on(bleuio.gap_scan()).unwrap();

        assert_eq!(
            result,
            ScanResult {
                index: 1,
                address_type: 1,
                address: [0x30, 0x63, 0xc5, 0xd0, 0xb1, 0xde],
                rssi: -38,
            }
        );
        assert_eq!(bleuio.into_inner().writes, GAP_SCAN_COMMAND);
    }

    #[test]
    fn command_constants_include_required_crlf() {
        assert_eq!(ATTENTION_COMMAND, b"AT\r\n");
        assert_eq!(CENTRAL_ROLE_COMMAND, b"AT+CENTRAL\r\n");
        assert_eq!(GAP_SCAN_COMMAND, b"AT+GAPSCAN=1\r\n");
    }

    #[test]
    fn at_response_accumulates_bytewise_across_packet_boundaries() {
        let expected = b"AT\r\nOK\r\n";
        let mut response = AtResponseAccumulator::new();

        for (index, byte) in expected.iter().enumerate() {
            let status = response.push(core::slice::from_ref(byte)).unwrap();
            let expected_status = if index + 1 == expected.len() {
                AtResponseStatus::Ok
            } else {
                AtResponseStatus::Pending
            };
            assert_eq!(status, expected_status);
        }

        assert_eq!(&response.bytes[..response.len], expected);
        assert_eq!(response.len(), expected.len());
        assert!(!response.is_empty());
    }

    #[test]
    fn at_response_accepts_every_split_and_only_exact_lines() {
        let expected = b"AT\r\nOK\r\n";
        for split in 0..=expected.len() {
            let mut response = AtResponseAccumulator::new();
            let first_status = response.push(&expected[..split]).unwrap();
            if split == expected.len() {
                assert_eq!(first_status, AtResponseStatus::Ok);
            } else {
                assert_eq!(first_status, AtResponseStatus::Pending);
                assert_eq!(response.push(&expected[split..]), Ok(AtResponseStatus::Ok));
            }
        }

        for non_match in [
            &b"NOTOK\r\n"[..],
            &b"OKAY\r\n"[..],
            &b"ok\r\n"[..],
            &b"AT\r\n\r\n"[..],
        ] {
            let mut response = AtResponseAccumulator::new();
            assert_eq!(response.push(non_match), Ok(AtResponseStatus::Pending));
        }

        let mut no_echo = AtResponseAccumulator::new();
        assert_eq!(no_echo.push(b"\r\nOK\r\n"), Ok(AtResponseStatus::Ok));

        let mut error = AtResponseAccumulator::new();
        assert_eq!(error.push(b"AT\r\nERROR\r\n"), Ok(AtResponseStatus::Error));
    }

    #[test]
    fn at_response_capacity_is_exact_and_overflow_is_atomic() {
        let mut prefix = [0_u8; MAX_AT_RESPONSE_BYTES - 4];
        for pair in prefix.chunks_exact_mut(2) {
            pair.copy_from_slice(b"\r\n");
        }
        let mut exact = AtResponseAccumulator::new();
        assert_eq!(exact.push(&prefix), Ok(AtResponseStatus::Pending));
        assert_eq!(exact.push(b"OK\r\n"), Ok(AtResponseStatus::Ok));
        assert_eq!(exact.len(), MAX_AT_RESPONSE_BYTES);

        let mut terminal_before_trailing = AtResponseAccumulator::new();
        assert_eq!(
            terminal_before_trailing.push(&prefix),
            Ok(AtResponseStatus::Pending)
        );
        assert_eq!(
            terminal_before_trailing.push(b"OK\r\nJUNK"),
            Ok(AtResponseStatus::Ok)
        );
        assert_eq!(terminal_before_trailing.len(), MAX_AT_RESPONSE_BYTES);
        assert!(
            terminal_before_trailing.bytes[..terminal_before_trailing.len].ends_with(b"OK\r\n")
        );

        let full = [b'X'; MAX_AT_RESPONSE_BYTES];
        let mut overflow = AtResponseAccumulator::new();
        assert_eq!(overflow.push(&full), Ok(AtResponseStatus::Pending));
        let before_overflow = overflow.clone();
        assert_eq!(overflow.push(b"!"), Err(AtResponseError::Overflow));
        assert_eq!(overflow, before_overflow);
    }

    #[test]
    fn scan_parser_reassembles_every_packet_split() {
        let transcript = b"AT+GAPSCAN=1\r\n\
            SCANNING...\r\n\
            [01] Device: [1]30:63:C5:D0:B1:DE RSSI: -38\r\n\
            SCAN COMPLETE\r\n";
        let expected = ScanResult {
            index: 1,
            address_type: 1,
            address: [0x30, 0x63, 0xc5, 0xd0, 0xb1, 0xde],
            rssi: -38,
        };

        for split in 0..=transcript.len() {
            let mut scan = ScanAccumulator::new();
            let first = scan.push(&transcript[..split]).unwrap();
            if split == transcript.len() {
                assert_eq!(first, ScanStatus::Complete);
            } else {
                assert_eq!(first, ScanStatus::Pending);
                assert_eq!(scan.push(&transcript[split..]), Ok(ScanStatus::Complete));
            }
            assert_eq!(scan.first_result(), Some(expected));
        }

        let mut bytewise = ScanAccumulator::new();
        for (index, byte) in transcript.iter().enumerate() {
            let status = bytewise.push(core::slice::from_ref(byte)).unwrap();
            assert_eq!(
                status,
                if index + 1 == transcript.len() {
                    ScanStatus::Complete
                } else {
                    ScanStatus::Pending
                }
            );
        }
        assert_eq!(bytewise.first_result(), Some(expected));
    }

    #[test]
    fn scan_parser_keeps_first_result_and_accepts_suffixes() {
        let transcript = b"\r\nSCANNING...\r\n\
            [7] Device: [0]aa:bb:cc:dd:ee:ff  RSSI: -75(closebeacon.com)\r\n\
            ignored asynchronous line\r\n\
            [08] Device: [1]01:02:03:04:05:06 RSSI: +12 [MFSID: 004C]\r\n\
            SCAN COMPLETE\r\ntrailing data";
        let mut scan = ScanAccumulator::new();

        assert_eq!(scan.push(transcript), Ok(ScanStatus::Complete));
        assert_eq!(
            scan.first_result(),
            Some(ScanResult {
                index: 7,
                address_type: 0,
                address: [0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff],
                rssi: -75,
            })
        );
        assert_eq!(
            scan.push(b"ignored after completion"),
            Ok(ScanStatus::Complete)
        );
    }

    #[test]
    fn scan_parser_rejects_terminal_and_sequence_errors() {
        let mut device_error = ScanAccumulator::new();
        assert_eq!(device_error.push(b"ERROR\r\n"), Err(ScanError::DeviceError));

        let mut unexpected_device = ScanAccumulator::new();
        assert_eq!(
            unexpected_device.push(b"[01] Device: [0]01:02:03:04:05:06 RSSI: -1\r\n"),
            Err(ScanError::UnexpectedDevice)
        );

        let mut unexpected_complete = ScanAccumulator::new();
        assert_eq!(
            unexpected_complete.push(b"SCAN COMPLETE\r\n"),
            Err(ScanError::UnexpectedComplete)
        );

        let mut no_device = ScanAccumulator::new();
        assert_eq!(
            no_device.push(b"SCANNING...\r\nSCAN COMPLETE\r\n"),
            Err(ScanError::NoDevice)
        );
    }

    #[test]
    fn scan_parser_rejects_malformed_device_lines_and_overflow() {
        for malformed in [
            &b"SCANNING...\r\n[01] Device: [2]01:02:03:04:05:06 RSSI: -1\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [0]01:02:03:04:05:GG RSSI: -1\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [0]01:02:03:04:05:06 RSSI: -129\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [0]01:02:03:04:05:06 RSSI: nope\r\n"[..],
            &b"SCANNING...\r\n[0001] Device: [0]01:02:03:04:05:06 RSSI: -1\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [00]01:02:03:04:05:06 RSSI: -1\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [0]01:02:03:04:05:06RSSI: -1\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [0]01:02:03:04:05:06 RSSI:-1\r\n"[..],
            &b"SCANNING...\r\n[01] Device: [0]01:02:03:04:05:06 RSSI: -38xyz\r\n"[..],
        ] {
            let mut scan = ScanAccumulator::new();
            assert_eq!(scan.push(malformed), Err(ScanError::MalformedDeviceLine));
        }

        let full = [b'X'; MAX_SCAN_LINE_BYTES];
        let mut overflow = ScanAccumulator::new();
        assert_eq!(overflow.push(&full), Ok(ScanStatus::Pending));
        assert_eq!(overflow.push(b"!"), Err(ScanError::LineOverflow));

        let mut exact = ScanAccumulator::new();
        let mut exact_line = [b'X'; MAX_SCAN_LINE_BYTES + 1];
        exact_line[MAX_SCAN_LINE_BYTES - 1] = b'\r';
        exact_line[MAX_SCAN_LINE_BYTES] = b'\n';
        assert_eq!(exact.push(&exact_line), Ok(ScanStatus::Pending));
    }
}
