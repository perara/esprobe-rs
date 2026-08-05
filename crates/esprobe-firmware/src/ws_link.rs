//! One socket, held open, carrying the trackpad.
//!
//! The wire format and every decision about it live in
//! [`esprobe_firmware::link`], which is tested on the host. This is the part
//! that cannot be: the socket, the push thread, and the two things a persistent
//! connection makes possible that request-per-command never could.
//!
//! # A closed socket stops the motor
//!
//! Over HTTP there is no such thing as a client going away — there is only a
//! client that has not sent anything recently, which is why the jog has a
//! watchdog. Here the transport reports the disconnect, so closing the tab,
//! walking out of Wi-Fi range or killing the browser stops the motor at once
//! instead of up to [`stepper::JOG_TIMEOUT_US`] later. The watchdog stays for
//! the case the socket cannot detect: a client that is still connected and has
//! stopped thinking.
//!
//! # Nothing is sent when nothing changes
//!
//! The page used to ask for status twice a second forever. Status is now pushed
//! only when it differs from what was last pushed, so a station nobody is
//! touching puts nothing on the air at all.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result};
use embedded_svc::ws::FrameType;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::http::server::ws::EspHttpWsDetachedSender;

use esprobe_firmware::link::{self, Command, Status};

/// Where the page connects.
pub const PATH: &str = esprobe_firmware::ws_link_path();

/// How often the pusher looks for a change.
///
/// This paces the *display*, not the motor: commands are acted on the moment
/// they arrive, and nothing waits for this. Forty a second is smoother than a
/// screen can show and costs twelve bytes each while moving, nothing at rest.
const PUSH_PERIOD_MS: u64 = 25;

/// Big enough for any frame the protocol defines, with room to recognise one
/// that is too long and disconnect rather than wedge.
///
/// The receive API leaves an oversized frame unconsumed, so a client sending
/// one would have it re-offered forever. Reading it and then judging it is the
/// only way to stay out of that loop.
const RECV_BUF: usize = 64;

/// Tear down a connection whose stream can no longer be trusted.
///
/// This exists because of a loop that took the whole server down. When the
/// receive fails, the frame it failed on stays unread, and returning an error
/// from the handler has the server offer it again immediately — the same
/// bytes, the same failure, thousands of times a millisecond, with the CPU
/// spent on nothing and every other request starved. It was reached by a
/// single desynchronised frame, and it outlived the client that caused it,
/// because nothing in the loop depended on that client still being there.
///
/// Shutting the socket down makes the next read return end-of-file, which is
/// the one thing the server does act on. A disconnected client reconnects; a
/// wedged station has to be power-cycled.
fn abandon(fd: i32) {
    // Read direction only, and deliberately.
    //
    // The first attempt shut down both directions, which did not end the loop:
    // the server kept the session and kept offering the same frame. The second
    // asked the server to close the session outright, from inside that
    // session's own handler, and the board stopped answering shortly after —
    // which is what asking a server to free the thing it is currently standing
    // on tends to look like.
    //
    // Half-closing the read side needs neither. The next read returns
    // end-of-file, which is an ordinary client disconnect and the one thing the
    // server already knows how to unwind: it closes the session itself, in its
    // own time, on its own thread, and the close handler runs and stops the
    // motor. The write direction is left alone because the server may still be
    // using it.
    //
    // Safety: shutting down a descriptor is defined whether or not it is still
    // valid.
    unsafe {
        esp_idf_svc::sys::lwip_shutdown(fd, esp_idf_svc::sys::SHUT_RD as i32);
    }
}

/// The most pages that may hold the link at once.
///
/// Deliberately well under the server's socket budget, so that the link can
/// never starve the REST routes — including `/health`, which is how anyone
/// would find out that it had. Past this the oldest is dropped, because the
/// usual way to reach the limit is tabs nobody is looking at, and the client
/// that just connected is the one someone is actually using.
const MAX_CLIENTS: usize = 3;

/// Clients currently listening, oldest first, each with the descriptor needed
/// to hang up on it.
type Subscribers = Arc<Mutex<Vec<(i32, EspHttpWsDetachedSender)>>>;

/// Bound how long a write may block, and turn off Nagle's algorithm.
///
/// This is the single largest latency item on the whole path. Nagle holds a
/// small write back until the previous segment is acknowledged, to avoid
/// flooding a network with tiny packets — which is exactly what this protocol
/// does deliberately, and the interaction with delayed acknowledgement can add
/// tens of milliseconds to a three-byte command. Every saving from shrinking
/// the frame would be given back and then some.
fn tune_socket(fd: i32) {
    // A send that blocks here blocks everything.
    //
    // The server is a single task servicing every socket, so a write that waits
    // on a congested client is not slow for that client — it is slow for the
    // whole board, REST routes included. That is what a health check timing out
    // during a busy jog turned out to be. With a deadline the write fails
    // instead, the reply is dropped, and the next one goes out cleanly; the
    // only thing lost is one round-trip reading.
    let timeout = esp_idf_svc::sys::timeval {
        tv_sec: 0,
        tv_usec: 200_000,
    };
    // Safety: the descriptor belongs to the connection being handled and the
    // option value outlives the call.
    let rc = unsafe {
        esp_idf_svc::sys::lwip_setsockopt(
            fd,
            esp_idf_svc::sys::SOL_SOCKET as i32,
            esp_idf_svc::sys::SO_SNDTIMEO as i32,
            core::ptr::addr_of!(timeout).cast(),
            core::mem::size_of::<esp_idf_svc::sys::timeval>() as u32,
        )
    };
    if rc != 0 {
        log::warn!("no send deadline on fd {fd}; a slow client can stall the server");
    }

    let on: u32 = 1;
    // Safety: `fd` comes from the connection we are handling, and the option
    // value outlives the call. A failure here is not worth refusing the
    // connection over — it costs latency, not correctness — so the result is
    // logged rather than propagated.
    let rc = unsafe {
        esp_idf_svc::sys::lwip_setsockopt(
            fd,
            esp_idf_svc::sys::IPPROTO_TCP as i32,
            esp_idf_svc::sys::TCP_NODELAY as i32,
            core::ptr::addr_of!(on).cast(),
            core::mem::size_of::<u32>() as u32,
        )
    };
    if rc != 0 {
        log::warn!("could not disable Nagle on fd {fd}; commands will be slower");
    }
}

/// Read the planner into something sendable.
fn snapshot(planner: &esprobe_firmware::stepper::Planner) -> Status {
    Status {
        // Saturating, not wrapping: the count is 64-bit and the wire is 32, and
        // a display that wraps to a large negative number after a very long
        // session would look like a fault rather than like a big number.
        position: planner.position().clamp(i32::MIN as i64, i32::MAX as i64) as i32,
        rate: planner.rate().clamp(i16::MIN as i32, i16::MAX as i32) as i16,
        moving: planner.is_moving(),
        energised: planner.is_energised(),
        remaining: planner.remaining(),
    }
}

fn now_us() -> u64 {
    // Safety: reading the monotonic microsecond counter has no preconditions.
    unsafe { esp_idf_svc::sys::esp_timer_get_time() as u64 }
}

/// Register the socket and start the pusher.
pub fn register(
    server: &mut EspHttpServer<'static>,
    stepper: crate::stepper_hw::Shared,
) -> Result<()> {
    let subscribers: Subscribers = Arc::new(Mutex::new(Vec::new()));

    // The pusher owns no hardware and does no work when nothing moves, so it
    // gets a small stack and the lowest priority that still keeps up.
    let planner = stepper.clone();
    let listeners = subscribers.clone();
    std::thread::Builder::new()
        .name("ws-push".into())
        // Not 4 KiB. This thread writes to sockets and formats log messages,
        // both of which reach well down a stack, and an overflow here presents
        // as the whole board going quiet — which is what happened once during
        // bring-up, cause unproven. Space is the cheaper side of that bet.
        .stack_size(8192)
        .spawn(move || {
            let mut last: Option<Status> = None;
            loop {
                std::thread::sleep(Duration::from_millis(PUSH_PERIOD_MS));

                // Taken out of the shared list, sent to, and put back. The
                // sends are blocking, and on a congested link one of them can
                // take a long time; holding the lock across that stalls the
                // handler thread trying to register a new client, which is the
                // whole server. Nothing else may hold this lock across I/O for
                // the same reason.
                let mut list = match listeners.lock() {
                    Ok(mut clients) => core::mem::take(&mut *clients),
                    Err(_) => continue,
                };
                // Reap. A client that went away leaves a sender behind, and
                // sending to it every cycle forever is the sort of thing that
                // works fine on a bench and fills a log in a fortnight.
                list.retain(|(_, tx)| !tx.is_closed());

                let status = if list.is_empty() {
                    last = None;
                    None
                } else {
                    match planner.lock() {
                        // Unchanged means unsent. That is the whole point.
                        Ok(planner) => {
                            let status = snapshot(&planner);
                            (last != Some(status)).then(|| {
                                last = Some(status);
                                status
                            })
                        }
                        Err(_) => None,
                    }
                };
                if let Some(status) = status {
                    let frame = link::encode_status(&status);
                    for (_, tx) in list.iter_mut() {
                        let _ = tx.send(FrameType::Binary(false), &frame);
                    }
                }
                if let Ok(mut clients) = listeners.lock() {
                    // Anything registered while the list was out joins the end.
                    list.append(&mut clients);
                    *clients = list;
                }
            }
        })
        .context("starting the status pusher")?;

    let state = stepper.clone();
    let listeners = subscribers.clone();
    server
        .ws_handler(PATH, None, move |conn| {
            if conn.is_closed() {
                // A closing connection stops the motor only when it was the
                // last one. Stopping on *any* close was wrong the moment more
                // than one client could exist: a stale tab going away, or one
                // evicted to make room for somebody else, would stop a motor
                // that a different client was actively driving. It presented as
                // a jog that died after a fraction of a second for no reason
                // the planner could explain, and it took a packet dump of the
                // status pushes to see that the motor had been told to stop
                // rather than having timed out.
                let fd = conn.session();
                let left = match listeners.lock() {
                    Ok(mut clients) => {
                        clients.retain(|(other, tx)| *other != fd && !tx.is_closed());
                        clients.len()
                    }
                    // Unknowable, so assume the worst and stop.
                    Err(_) => 0,
                };
                if left == 0
                    && let Ok(mut planner) = state.lock()
                {
                    planner.stop(now_us());
                }
                return Ok::<(), anyhow::Error>(());
            }

            if conn.is_new() {
                tune_socket(conn.session());
                match conn.create_detached_sender() {
                    Ok(tx) => {
                        // Who to hang up on is decided under the lock; the
                        // hanging up happens after it is released. Doing it
                        // inside deadlocks: closing a socket has the server run
                        // the close handler, and that handler takes this same
                        // lock to work out whether the last client just left.
                        // A `std::sync::Mutex` is not reentrant, so the HTTP
                        // thread stops there and the board answers nothing on
                        // any route.
                        let mut evicted = Vec::new();
                        if let Ok(mut clients) = listeners.lock() {
                            clients.retain(|(_, tx)| !tx.is_closed());
                            while clients.len() >= MAX_CLIENTS {
                                evicted.push(clients.remove(0).0);
                            }
                            clients.push((conn.session(), tx));
                        }
                        for old in evicted {
                            log::info!("stepper link full; hanging up on {old}");
                            abandon(old);
                        }
                    }
                    Err(err) => log::warn!("no status push for this client: {err}"),
                }
                return Ok(());
            }

            let mut buf = [0u8; RECV_BUF];
            let fd = conn.session();
            let (frame_type, len) = match conn.recv(&mut buf) {
                Ok(frame) => frame,
                Err(err) => {
                    // Never propagated. See `abandon`: returning an error here
                    // is what spun the server, because the unread frame comes
                    // straight back.
                    log::warn!("stepper link {fd} failed a read ({err}); closing it");
                    // No stop here. Half-closing makes the server unwind the
                    // session, which runs the close handler above, which stops
                    // the motor if this was the last client and leaves it alone
                    // if it was not. Stopping here as well would take the motor
                    // away from whoever else is still holding the pad.
                    abandon(fd);
                    return Ok(());
                }
            };
            match frame_type {
                FrameType::Binary(false) => {}
                FrameType::Close | FrameType::SocketClose => {
                    // Let the close handler decide, for the same reason: this
                    // is one client leaving, which is only a reason to stop if
                    // it was the only one.
                    abandon(fd);
                    return Ok(());
                }
                // Fragmented or text frames are not something this protocol
                // emits, so they are not something to guess at.
                _ => return Ok(()),
            }
            if len > buf.len() {
                // Left unread by the receive API, so it would be re-offered
                // forever exactly like a failed read. Same answer.
                log::warn!("stepper link {fd} sent {len} bytes; closing it");
                abandon(fd);
                return Ok(());
            }
            let Some(command) = link::decode(&buf[..len]) else {
                // Not fatal: a stray frame is dropped, and the link stays up so
                // the stop that follows still has somewhere to arrive.
                log::warn!("undecodable frame of {len} bytes");
                return Ok(());
            };

            let now = now_us();
            let mut planner = state
                .lock()
                .map_err(|_| anyhow::anyhow!("stepper lock poisoned"))?;
            match command {
                // No sequence numbers on this path. One TCP connection delivers
                // in the order it was written, so a jog cannot overtake the stop
                // that followed it — the race the REST routes have to stamp
                // against does not exist here.
                Command::Jog(rate) => {
                    planner.jog_for(rate, now, esprobe_firmware::stepper::LINK_JOG_TIMEOUT_US);
                }
                Command::Stop => planner.stop(now),
                Command::Release => planner.release(now),
                Command::Move { steps, steps_per_s } => {
                    planner.move_by(steps, steps_per_s, now);
                }
                Command::Accel(accel) => planner.set_accel(accel),
                Command::Ping => {
                    // The keepalive for the jog watchdog, and the client's
                    // round-trip probe, in one byte each way.
                    planner.keepalive_for(now, esprobe_firmware::stepper::LINK_JOG_TIMEOUT_US);
                    drop(planner);
                    // Best-effort. A failed reply costs this client its
                    // round-trip reading; propagating it would cost every
                    // client the server.
                    if let Err(err) = conn.send(FrameType::Binary(false), &[link::MSG_PONG]) {
                        log::warn!("stepper link {fd} would not take a pong: {err}");
                    }
                }
            }
            Ok(())
        })
        .context("registering the stepper socket")?;

    Ok(())
}
