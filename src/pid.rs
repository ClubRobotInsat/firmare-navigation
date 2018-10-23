use core::fmt::Write;
use core::fmt::{Display, Formatter, Result};

use cortex_m_semihosting::hio::HStdout;

use qei::QeiManager;

use embedded_hal::digital::OutputPin;
use embedded_hal::{PwmPin, Qei};

pub struct Motor<MOT, DIR> {
    pwm: MOT,
    dir: DIR,
}

impl<MOT, DIR> Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    /// Crée une nouvelle structure de gestion moteur
    pub fn new(pwm: MOT, dir: DIR) -> Self {
        Motor { pwm, dir }
    }

    /// Applique la commande de direction et de vitesse aux moteurs
    pub fn apply_command(&mut self, cmd: Command) {
        match cmd {
            Command::Front(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_low();
            }
            Command::Back(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_high();
            }
        }
    }
}

/// La direction et la vitesse de la consigne
///
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Command {
    Front(u16),
    Back(u16),
}

impl Display for Command {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            Command::Front(val) => write!(f, "Forward : {}", val),
            Command::Back(val) => write!(f, "Backward {}", val),
        }
    }
}

/// Le PID du robot
pub struct Pid<L, R> {
    old_left_count: i64,
    old_right_count: i64,
    left_qei: QeiManager<L>,
    right_qei: QeiManager<R>,
    pos_kp: i64,
    pos_kd: i64,
    orient_kp: i64,
    orient_kd: i64,
    // Le multiplicateur interne pour augmenter la précision des calculs
    internal_multiplier: i64,
    // La valeur maximale de la commande en sortie
    cap: u16,
    // La consigne de position du robot exprimée en nombre de tick de roue codeuse
    position_consigne: i64,
    // La consigne d'angle exprimée en différence de tick de chaque roue codeuse
    orientation_consigne: i64,
}

impl<L, R> Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
    pub fn new(
        pos_kp: i64,
        pos_kd: i64,
        orient_kp: i64,
        orient_kd: i64,
        internal_multiplier: i64,
        cap: u16,
        left_qei: QeiManager<L>,
        right_qei: QeiManager<R>,
    ) -> Self {
        Pid {
            old_left_count: 0,
            old_right_count: 0,
            left_qei,
            right_qei,
            pos_kp,
            pos_kd,
            orient_kp,
            orient_kd,
            internal_multiplier,
            cap,
            position_consigne: 0,
            orientation_consigne: 0,
        }
    }

    /// Mets à jour la consigne en position
    pub fn set_position_goal(&mut self, pos: i64) {
        self.position_consigne = pos;
    }

    pub fn set_orientation_goal(&mut self, orientation: i64) {
        self.orientation_consigne = orientation;
    }

    /// Renvoie la nouvelle consigne à appliquer aux deux roues pour atteindre la commande en position
    fn update_position_command(
        &self,
        left_count: i64,
        right_count: i64,
        left_speed: i64,
        right_speed: i64,
    ) -> i64 {
        let dist = (left_count + right_count) / 2;
        let speed = (left_speed + right_speed) / 2;
        let diff = self.position_consigne as i64 - dist;
        let cmd = (diff * self.pos_kp) - self.pos_kd * speed;
        cmd
    }

    /// Renvoie les nouvelles consignes à appliquer aux deux roues pour atteindre la commande en orientation
    fn update_orientation_command(
        &self,
        left_count: i64,
        right_count: i64,
        left_speed: i64,
        right_speed: i64,
    ) -> (i64, i64) {
        let orientation = right_count - left_count;
        let speed = right_speed - left_speed;
        let diff = self.orientation_consigne - orientation;
        let cmd = (diff * self.orient_kp) - self.orient_kd * speed;
        (-cmd, cmd)
    }

    fn truncate(&self, val: i64) -> Command {
        if val.is_positive() {
            if val > self.cap as i64 {
                Command::Front(self.cap)
            } else {
                Command::Front(val as u16)
            }
        } else {
            if -val > self.cap as i64 {
                Command::Back(self.cap)
            } else {
                Command::Back((-val) as u16)
            }
        }
    }

    /// Renvoie la nouvelle consigne à appliquer aux roues pour le pid
    pub fn update(&mut self) -> (Command, Command) {
        // Mise à jour des QEI QEI
        self.left_qei.sample_unwrap();
        self.right_qei.sample_unwrap();

        // Mise à jour de la mémoire du PID
        let (new_left_count, new_right_count) = (self.left_qei.count(), self.right_qei.count());
        let (left_speed, right_speed) = (
            new_left_count - self.old_left_count,
            new_right_count - self.old_right_count,
        );
        self.old_left_count = new_left_count;
        self.old_right_count = new_right_count;
        // Calcul du PID
        let position_cmd =
            self.update_position_command(new_left_count, new_right_count, left_speed, right_speed);
        let (orientation_cmd_left, orientation_cmd_right) = self.update_orientation_command(
            new_left_count,
            new_right_count,
            left_speed,
            right_speed,
        );

        let left_cmd = position_cmd + orientation_cmd_left;
        let right_cmd = position_cmd + orientation_cmd_right;
        // Truncate resul
        (self.truncate(left_cmd), self.truncate(right_cmd))
    }

    pub fn print_qei_state(&self, out: &mut HStdout) {
        write!(
            out,
            "Left : {}, Right : {} \n",
            self.left_qei.count(),
            self.right_qei.count()
        )
        .unwrap();
    }
}
