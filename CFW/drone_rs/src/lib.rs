#![no_std]

// use core::fmt::Write;
// use cortex_m_semihosting::hio;
use panic_halt as _;
use pid::Pid;

#[repr(C)]
pub struct Gyro {
    ax: i16,
    ay: i16,
    az: i16,
    gx: i16,
    gy: i16,
    gz: i16,
    temp: i16,
}

#[repr(C)]
pub struct Controls {
    throttle: u8,
    yaw: u8,
    pitch: u8,
    roll: u8,
}

#[repr(C)]
pub struct Motors {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
}

struct ControlState {
    pid_roll: Pid<f32>,
    pid_pitch: Pid<f32>,
    pid_yaw: Pid<f32>,
    gyro_roll: f32,
    gyro_pitch: f32,
    gyro_yaw: f32,
}

static mut STATE: Option<ControlState> = None;

// fn print() -> Result<(), core::fmt::Error> {
//     let mut stdout = match hio::hstdout() {
//         Ok(fd) => fd,
//         Err(()) => return Err(core::fmt::Error),
//     };

//     let language = "Rust";
//     let ranking = 1;

//     write!(stdout, "{} on embedded is #{}!", language, ranking)?;

//     Ok(())
// }

#[no_mangle]
pub extern "C" fn control_loop(gyro: &Gyro, controls: &Controls, motors: &mut Motors) {
    unsafe {
        match STATE {
            None => {
                STATE = Some(ControlState {
                    pid_roll: Pid::new(1.0, 0.001, 0.2, 100.0, 5.0, 10.0, 0.0),
                    pid_pitch: Pid::new(1.0, 0.001, 0.2, 100.0, 5.0, 10.0, 0.0),
                    pid_yaw: Pid::new(1.0, 0.0, 0.0, 50.0, 3.0, 100.0, 0.0),
                    gyro_roll: 0.0,
                    gyro_pitch: 0.0,
                    gyro_yaw: 0.0,
                });
            }
            _ => {}
        }
    }

    let mut roll_out = 0.0;
    let mut pitch_out = 0.0;
    let mut yaw_out = 0.0;

    let roll_control = -(controls.roll as f32 - 100.0) * 1.4;
    let pitch_control = -(controls.pitch as f32 - 100.0) * 1.4;
    let yaw_control = -(controls.yaw as f32 - 100.0) * 1.3;

    unsafe {
        match STATE.as_mut() {
            Some(s) => {
                s.gyro_roll = (s.gyro_roll * 0.7) + ((gyro.gx as f32 / 65.5) * 0.3);
                s.gyro_pitch = (s.gyro_pitch * 0.7) + ((gyro.gy as f32 / 65.5) * 0.3);
                s.gyro_yaw = (s.gyro_yaw * 0.7) + ((gyro.gz as f32 / 65.5) * 0.3);

                roll_out = s.pid_roll.next_control_output(s.gyro_roll).output + roll_control;
                pitch_out = s.pid_pitch.next_control_output(s.gyro_pitch).output + pitch_control;
                yaw_out = s.pid_yaw.next_control_output(s.gyro_yaw).output + yaw_control;
            }
            None => {}
        }
    }

    let throttle = controls.throttle.saturating_sub(0x64) as f32;

    motors.a = ((throttle - roll_out - pitch_out - yaw_out) as u8).min(100);
    motors.b = ((throttle - roll_out + pitch_out + yaw_out) as u8).min(100);
    motors.c = ((throttle + roll_out + pitch_out - yaw_out) as u8).min(100);
    motors.d = ((throttle + roll_out - pitch_out + yaw_out) as u8).min(100);

    // motors.a = controls.throttle.saturating_sub(0x64);
}
