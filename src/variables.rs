struct Etat {
    total_distance: isize,
    right_wheel_distance: isize,
    left_wheel_distance: isize,

    longitudinal_velocity: isize,
    angular_velocity: isize,
    right_wheel_velocity: isize,
    left_wheel_velocity: isize,

    x: isize,
    y: isize,

    rough_angle: isize,
    actual_angle: isize,
}

struct Mecanique {
    axis_length: isize,

    right_wheel_diameter: isize,
    left_wheel_diameter: isize,

    left_motor_power: isize,
    right_motor_power: isize,

    maximum_left_motor_power: isize,
    maximum_right_motor_power: isize,
    maximum_longitudinal_power: isize,
    maximum_anglar_power: isize,

    maximum_right_wheel_velocity: isize,
    maximum_left_wheel_velocity: isize,

    right_ticks: isize,
    left_ticks: isize,
}

struct Timing {
    fcy: isize,
    tcy: isize,

    fe: isize,
    te_odometry: isize,
    te_control: isize,

    pwm_frequency: isize,
    pwm_period: isize,
}

struct Control {
    required: isize,
    instruction: isize,

    maximum_speed: isize,
    required_speed: isize,
    instruction_speed: isize,
    previous_required_speed: isize,

    acceleration: isize,
    deceleration: isize,

    is_enabled: bool,
}

/*
typedef struct _CORRECTEUR
{
    float Erreur;
    float ErreurPrecedente;
    float IntegraleErreur;
    float DeriveErreur;

    float CommandeProportionelle;
    float CommandeIntegrale;
    float CommandeDerive;

    float Kp;
    float Ki;
    float Kd;

    float KpArret;
    float KiArret;
    float KdArret;

    float KpRoule;
    float KiRoule;
    float KdRoule;

    float Commande;
} CORRECTEUR;
*/

struct Movement {
    required_distance: isize,
    required_angle: isize,

    required_right_wheel_distance: isize,
    required_left_wheel_distance: isize,

    xc: isize,
    yc: isize,

    longitudinal_precision: isize,
    angular_precision: isize,

    movement_type: u8,
    forward: bool,
    stage: u8,
    done: bool,
    done_and_sent: bool,
    time_blocked: isize,

    controller_configuration: u8,
    /* movement types :
        0: //Stop
        1: //Translation de DistanceDemande
        2: //Rotation relative de AngleDemande
        3: //Rotation absolue jusqu'à AngleDemande (Avant / Arrière)
        4: //Pointe vers Xc,Yc (Avant / Arrière)
        5: //Va vers Xc, Yc (Decompose, Avant/Arriere)
        6: //VaZy Xc, Yc (Decompose, Avant/Arriere)
        7: //VaZy Xc, Yc (Avant/Arriere)
        8: //Translation à vitesse constante avec angle fixe
        9: //Rotation à vitesse constante sur place
        10: //Passer par le point Xc, Yc à vitesse constante
        11: //Pivot D
        12: //Pivot G
        13: //Position D&G
        14: //VitesseD&G
        15: //Arret urgence
        17: //VitesseL&A
    */
}
