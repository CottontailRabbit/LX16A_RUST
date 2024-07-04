use serialport::{self, SerialPort};
use std::{sync::{Arc, Mutex}, time::Duration, io::{self, Write, Read}};
use std::thread;


use log::{debug, error, warn};


const SERVO_ID_ALL: u8 = 0xfe;
const SERVO_MOVE_TIME_WRITE: u8 = 1;
const SERVO_MOVE_TIME_READ: u8 = 2;
const SERVO_MOVE_TIME_WAIT_WRITE: u8 = 7;
const SERVO_MOVE_TIME_WAIT_READ: u8 = 8;
const SERVO_MOVE_START: u8 = 11;
const SERVO_MOVE_STOP: u8 = 12;
const SERVO_ID_WRITE: u8 = 13;
const SERVO_ID_READ: u8 = 14;
const SERVO_ANGLE_OFFSET_ADJUST: u8 = 17;
const SERVO_ANGLE_OFFSET_WRITE: u8 = 18;
const SERVO_ANGLE_OFFSET_READ: u8 = 19;
const SERVO_ANGLE_LIMIT_WRITE: u8 = 20;
const SERVO_ANGLE_LIMIT_READ: u8 = 21;
const SERVO_VIN_LIMIT_WRITE: u8 = 22;
const SERVO_VIN_LIMIT_READ: u8 = 23;
const SERVO_TEMP_MAX_LIMIT_WRITE: u8 = 24;
const SERVO_TEMP_MAX_LIMIT_READ: u8 = 25;
const SERVO_TEMP_READ: u8 = 26;
const SERVO_VIN_READ: u8 = 27;
const SERVO_POS_READ: u8 = 28;
const SERVO_OR_MOTOR_MODE_WRITE: u8 = 29;
const SERVO_OR_MOTOR_MODE_READ: u8 = 30;
const SERVO_LOAD_OR_UNLOAD_WRITE: u8 = 31;
const SERVO_LOAD_OR_UNLOAD_READ: u8 = 32;
const SERVO_LED_CTRL_WRITE: u8 = 33;
const SERVO_LED_CTRL_READ: u8 = 34;
const SERVO_LED_ERROR_WRITE: u8 = 35;
const SERVO_LED_ERROR_READ: u8 = 36;

const SERVO_ERROR_OVER_TEMPERATURE: u8 = 1;
const SERVO_ERROR_OVER_VOLTAGE: u8 = 2;
const SERVO_ERROR_LOCKED_ROTOR: u8 = 4;


// 유틸리티 함수
fn lower_byte(value: u16) -> u8 {
    (value & 0xFF) as u8
}

fn higher_byte(value: u16) -> u8 {
    ((value >> 8) & 0xFF) as u8
}

fn word(low: u8, high: u8) -> u16 {
    (low as u16) | ((high as u16) << 8)
}

fn clamp(value: i32, min: i32, max: i32) -> i32 {
    std::cmp::max(min, std::cmp::min(max, value))
}
#[derive(Debug)]
pub enum ControllerError {
    SerialPortError(serialport::Error),
    IoError(io::Error),
    Timeout,
}

impl From<serialport::Error> for ControllerError {
    fn from(err: serialport::Error) -> ControllerError {
        ControllerError::SerialPortError(err)
    }
}

impl From<io::Error> for ControllerError {
    fn from(err: io::Error) -> ControllerError {
        ControllerError::IoError(err)
    }
}

struct ServoController {
    serial: Arc<Mutex<Box<dyn SerialPort>>>,
    timeout: Duration,
    _lock: Mutex<()>,
}

impl ServoController
{
    pub fn new(port_name: &str, baud_rate: u32, timeout: Duration) -> Result<Self, ControllerError> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(timeout)
            .open()?;

        Ok(ServoController {
            serial: Arc::new(Mutex::new(port)),
            timeout,
            _lock: Mutex::new(()),

        })
    }

    fn command(&self, servo_id: u8, command: u8, params: &[u8]) -> Result<(), ControllerError>
    {
        let length = 3 + params.len() as u8;
        let divide_number:u16 = 256;
        let checksum: u8 = 255u8 - ((servo_id as u16 + length as u16 + command as u16 + params.iter().map(|&byte| byte as u16).sum::<u16>()) % divide_number as u16) as u8;

        let mut cmd_packet = vec![0x55, 0x55, servo_id, length, command];
        cmd_packet.extend_from_slice(params);
        cmd_packet.push(checksum);

        let mut serial = self.serial.lock().unwrap();
        serial.write_all(&cmd_packet)?;
        Ok(())
    }

    fn read_response(&self, servo_id: u8, command: u8) -> Result<Vec<u8>, ControllerError>
    {
        let mut read = |size: usize| -> Result<Vec<u8>, io::Error> {
            let mut buffer = vec![0; size];
            let mut serial = self.serial.lock().unwrap();
            serial.read_exact(&mut buffer)?;
            Ok(buffer)
        };

        loop
        {
            let mut data = read(1)?;
            if data[0] != 0x55 { continue; }
            data.extend(read(1)?);

            if data[1] != 0x55 { continue; }
            data.extend(read(3)?);

            let sid = data[2];
            let length = data[3] as usize;
            let cmd = data[4];

            if length > 7
            {
                error!("Invalid length for packet {:?}", data);
                continue;
            }

            if length > 3
            {
                data.extend(read(length - 3)?);
            }

            let params = &data[5..length + 2];
            return Ok(data);
        }
    }


    // Example method to demonstrate command sending and response reading
    pub fn move_servo(&self, servo_id: u8, position: u16, time: u16) -> Result<(), ControllerError>
    {
        let position_low = lower_byte(position);
        let position_high = higher_byte(position);
        let time_low = lower_byte(time);
        let time_high = higher_byte(time);

        self.command(servo_id, SERVO_MOVE_TIME_WRITE, &[position_low, position_high, time_low, time_high])?;

        Ok(())
    }

    pub fn move_prepare(&self, servo_id: u8, position: u16, time: u16) -> Result<(), ControllerError>
    {
        let position_low = lower_byte(position);
        let position_high = higher_byte(position);
        let time_low = lower_byte(time);
        let time_high = higher_byte(time);

        self.command(servo_id, SERVO_MOVE_TIME_WAIT_WRITE, &[position_low, position_high, time_low, time_high])?;
        // Optionally, read and process the response

        Ok(())
    }

    pub fn led_off(&self, servo_id: u8) -> Result<(),ControllerError>
    {
        let off = 0u8;
        let _= self.command(servo_id, 33, &[0u8])?;

        Ok(())
    }

    pub fn move_start(&self, servo_id: u8) -> Result<(),ControllerError>
    {
        self.command(servo_id, SERVO_MOVE_START ,&[])?;

        Ok(())
    }

    pub fn move_stop(&self, servo_id: u8) -> Result<(),ControllerError>
    {
        self.command(servo_id, SERVO_MOVE_STOP ,&[])?;

        Ok(())
    }

    pub fn set_motor_mode(&self, servo_id: u8, speed: i32) -> Result<(), ControllerError>
    {
        let mut calc_speed = clamp(-1000, 1000, speed) as u16; // i32에서 u16으로 캐스팅

        self.command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, &[1, 0, lower_byte(calc_speed), higher_byte(calc_speed)])?;
        Ok(())
    }

    pub fn set_servo_mode(&self, servo_id: u8) -> Result<(), ControllerError>
    {
        self.command(servo_id, SERVO_OR_MOTOR_MODE_WRITE, &[0, 0, 0, 0])?;
        Ok(())
    }

    pub fn get_position(&self, servo_id: u8, timeout: Option<Duration>) -> Result<i16, ControllerError>
    {
        let response = self._query(servo_id, SERVO_POS_READ, timeout)?;
        let position =  word(response[5],response[6]);
        let position = position as i16;

        Ok(position)
    }

    // _query 메서드 구현
    fn _query(&self, servo_id: u8, command: u8, timeout: Option<Duration>) -> Result<Vec<u8>, ControllerError>
    {
        let _guard = self._lock.lock().unwrap();
        self.command(servo_id, command,&[])?;

        self.read_response(1, command)
    }



}

fn main() {
    // 예시 사용법
    let controller = ServoController::new("COM4", 115200, Duration::from_secs(4));

    match controller
    {
        Ok(ctrl)
        =>
        {
            // ctrl.move_servo(1u8);
            let _ = ctrl.set_servo_mode(1u8);
            thread::sleep(Duration::from_secs(1));
            let result = ctrl.move_servo(1u8, 0u16, 1000u16);
            thread::sleep(Duration::from_secs(1));

            match result
            {
                Ok(angle) => println!("성공"),
                Err(e) => println!("오류 발생: {:?}", e), // 오류 타입에 따라 적절한 메시지로 대체하세요.
            }


            let angle_result = ctrl.get_position(1u8, Some(Duration::from_secs(5)));

            match angle_result {
                Ok(angle) => println!("각도: {}", angle),
                Err(e) => println!("오류 발생: {:?}", e), // 오류 타입에 따라 적절한 메시지로 대체하세요.
            }
        },


        Err(e) =>  println!("Failed to initialize ServoController: {:?}", e),
    }
}
