#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterDataType {
    Int8,
    Int16,
    Int32,
    UInt8,
    UInt16,
    UInt32,
    Float32,
    String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ParameterId {
    MechanicalOffset = 0x2005,
    MeasuredPosition = 0x3016,
    MeasuredVelocity = 0x3017,
    MeasuredTorque = 0x302C,
    Mode = 0x7005,
    IqTarget = 0x7006,
    VelocityTarget = 0x700A,
    TorqueLimit = 0x700B,
    CurrentKp = 0x7010,
    CurrentKi = 0x7011,
    CurrentFilterGain = 0x7014,
    PositionTarget = 0x7016,
    VelocityLimit = 0x7017,
    CurrentLimit = 0x7018,
    MechanicalPosition = 0x7019,
    IqFiltered = 0x701A,
    MechanicalVelocity = 0x701B,
    Vbus = 0x701C,
    PositionKp = 0x701E,
    VelocityKp = 0x701F,
    VelocityKi = 0x7020,
    VelocityFilterGain = 0x7021,
    VelocityAccelerationTarget = 0x7022,
    PpVelocityMax = 0x7024,
    PpAccelerationTarget = 0x7025,
    EpscanTime = 0x7026,
    CanTimeout = 0x7028,
    ZeroState = 0x7029,
}

#[derive(Debug, Clone, Copy)]
pub struct ParameterInfo {
    pub id: u16,
    pub name: &'static str,
    pub data_type: ParameterDataType,
}

pub const ROBSTRIDE_PRODUCT_INFO_COMMIT: &str = "ba7236bc26417766fda71e75ae128c66dbd21aba";
pub const ROBSTRIDE_PRODUCT_INFO_URL: &str =
    "https://github.com/RobStride/Product_Information/commit/ba7236bc26417766fda71e75ae128c66dbd21aba";

macro_rules! param {
    ($id:expr, $name:expr, $ty:ident) => {
        ParameterInfo {
            id: $id,
            name: $name,
            data_type: $crate::registers::ParameterDataType::$ty,
        }
    };
}

#[path = "registers_00.rs"]
pub mod registers_00;
#[path = "registers_01.rs"]
pub mod registers_01;
#[path = "registers_02.rs"]
pub mod registers_02;
#[path = "registers_03.rs"]
pub mod registers_03;
#[path = "registers_04.rs"]
pub mod registers_04;
#[path = "registers_05.rs"]
pub mod registers_05;
#[path = "registers_06.rs"]
pub mod registers_06;

pub use registers_00::RS00_PARAMETER_TABLE;
pub use registers_01::RS01_PARAMETER_TABLE;
pub use registers_02::RS02_PARAMETER_TABLE;
pub use registers_03::RS03_PARAMETER_TABLE;
pub use registers_04::RS04_PARAMETER_TABLE;
pub use registers_05::RS05_PARAMETER_TABLE;
pub use registers_06::RS06_PARAMETER_TABLE;

pub static PARAMETER_TABLE: &[ParameterInfo] = &[
    param!(0x7005, "run_mode", Int8),
    param!(0x7006, "iq_ref", Float32),
    param!(0x700A, "spd_ref", Float32),
    param!(0x700B, "limit_torque", Float32),
    param!(0x7010, "cur_kp", Float32),
    param!(0x7011, "cur_ki", Float32),
    param!(0x7014, "cur_filter_gain", Float32),
    param!(0x7016, "loc_ref", Float32),
    param!(0x7017, "limit_spd", Float32),
    param!(0x7018, "limit_cur", Float32),
    param!(0x7019, "mechPos", Float32),
    param!(0x701A, "iqf", Float32),
    param!(0x701B, "mechVel", Float32),
    param!(0x701C, "VBUS", Float32),
    param!(0x701E, "loc_kp", Float32),
    param!(0x701F, "spd_kp", Float32),
    param!(0x7020, "spd_ki", Float32),
    param!(0x7021, "spd_filter_gain", Float32),
    param!(0x7022, "acc_rad", Float32),
    param!(0x7024, "vel_max", Float32),
    param!(0x7025, "acc_set", Float32),
    param!(0x7026, "EPScan_time", UInt16),
    param!(0x7028, "canTimeout", UInt32),
    param!(0x7029, "zero_sta", UInt8),
];

pub fn parameter_info(id: u16) -> Option<&'static ParameterInfo> {
    PARAMETER_TABLE.iter().find(|info| info.id == id)
}

pub fn parameter_table_for_model(model: &str) -> &'static [ParameterInfo] {
    match model.trim().to_ascii_lowercase().as_str() {
        "rs-00" | "rs00" => RS00_PARAMETER_TABLE,
        "rs-01" | "rs01" => RS01_PARAMETER_TABLE,
        "rs-02" | "rs02" => RS02_PARAMETER_TABLE,
        "rs-03" | "rs03" => RS03_PARAMETER_TABLE,
        "rs-04" | "rs04" => RS04_PARAMETER_TABLE,
        "rs-05" | "rs05" => RS05_PARAMETER_TABLE,
        "rs-06" | "rs06" => RS06_PARAMETER_TABLE,
        _ => PARAMETER_TABLE,
    }
}

pub fn parameter_info_for_model(model: &str, id: u16) -> Option<&'static ParameterInfo> {
    parameter_table_for_model(model)
        .iter()
        .find(|info| info.id == id)
        .or_else(|| PARAMETER_TABLE.iter().find(|info| info.id == id))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rs00_uses_dedicated_manual_parameter_table() {
        let encoder = parameter_info_for_model("rs-00", 0x3004).expect("rs00 encoderRaw");
        assert_eq!(encoder.name, "encoderRaw");
        assert_eq!(encoder.data_type, ParameterDataType::Int16);

        let tail = parameter_info_for_model("rs00", 0x304F).expect("rs00 H");
        assert_eq!(tail.name, "H");
        assert_eq!(tail.data_type, ParameterDataType::UInt8);
    }

    #[test]
    fn model_without_manual_table_keeps_only_common_control_parameters() {
        assert!(parameter_info_for_model("rs-99", 0x3004).is_none());
        assert_eq!(
            parameter_info_for_model("rs-99", 0x7019)
                .expect("common mechPos")
                .data_type,
            ParameterDataType::Float32
        );
    }

    #[test]
    fn rs01_uses_dedicated_manual_parameter_table() {
        let protocol = parameter_info_for_model("rs-01", 0x2020).expect("rs01 protocol_1");
        assert_eq!(protocol.name, "protocol_1");
        assert_eq!(protocol.data_type, ParameterDataType::UInt8);

        let elec = parameter_info_for_model("rs01", 0x302C).expect("rs01 ElecOffset");
        assert_eq!(elec.name, "ElecOffset");
        assert_eq!(elec.data_type, ParameterDataType::Float32);

        let theta = parameter_info_for_model("rs-01", 0x303B).expect("rs01 theta_mech_1");
        assert_eq!(theta.name, "theta_mech_1");
        assert_eq!(theta.data_type, ParameterDataType::Float32);
    }

    #[test]
    fn rs02_uses_dedicated_manual_parameter_table() {
        let zero = parameter_info_for_model("rs-02", 0x201E).expect("rs02 zero_sta");
        assert_eq!(zero.name, "zero_sta");
        assert_eq!(zero.data_type, ParameterDataType::UInt8);

        let angle = parameter_info_for_model("rs02", 0x3030).expect("rs02 motor_mech_angle");
        assert_eq!(angle.name, "motor_mech_angle");
        assert_eq!(angle.data_type, ParameterDataType::Float32);

        let status = parameter_info_for_model("rs-02", 0x3048).expect("rs02 can_status");
        assert_eq!(status.name, "can_status");
        assert_eq!(status.data_type, ParameterDataType::UInt8);
    }

    #[test]
    fn rs06_uses_dedicated_manual_parameter_table() {
        let can_id = parameter_info_for_model("rs-06", 0x2009).expect("rs06 CAN_ID");
        assert_eq!(can_id.name, "CAN_ID");
        assert_eq!(can_id.data_type, ParameterDataType::UInt8);

        let angle = parameter_info_for_model("rs06", 0x3028).expect("rs06 as_angle");
        assert_eq!(angle.name, "as_angle");
        assert_eq!(angle.data_type, ParameterDataType::Float32);

        let end = parameter_info_for_model("rs-06", 0x3048).expect("rs06 pos_cnt1");
        assert_eq!(end.name, "pos_cnt1");
        assert_eq!(end.data_type, ParameterDataType::UInt16);
    }

    #[test]
    fn rs03_uses_dedicated_manual_parameter_table() {
        let offset = parameter_info_for_model("rs-03", 0x2024).expect("rs03 position_offset");
        assert_eq!(offset.name, "position_offset");
        assert_eq!(offset.data_type, ParameterDataType::UInt8);

        let angle = parameter_info_for_model("rs03", 0x3027).expect("rs03 as_angle");
        assert_eq!(angle.name, "as_angle");
        assert_eq!(angle.data_type, ParameterDataType::Float32);

        let status = parameter_info_for_model("rs-03", 0x3041).expect("rs03 can_status");
        assert_eq!(status.name, "can_status");
        assert_eq!(status.data_type, ParameterDataType::UInt8);
    }

    #[test]
    fn rs04_uses_dedicated_manual_parameter_table() {
        let pp_vel = parameter_info_for_model("rs-04", 0x201B).expect("rs04 vel_max");
        assert_eq!(pp_vel.name, "vel_max");
        assert_eq!(pp_vel.data_type, ParameterDataType::Float32);

        let ibus = parameter_info_for_model("rs04", 0x302B).expect("rs04 ibus");
        assert_eq!(ibus.name, "ibus");
        assert_eq!(ibus.data_type, ParameterDataType::Float32);

        let coder = parameter_info_for_model("rs-04", 0x3047).expect("rs04 coder_reg");
        assert_eq!(coder.name, "coder_reg");
        assert_eq!(coder.data_type, ParameterDataType::UInt16);
    }

    #[test]
    fn rs05_uses_dedicated_manual_parameter_table() {
        let protocol = parameter_info_for_model("rs-05", 0x2022).expect("rs05 protocol_1");
        assert_eq!(protocol.name, "protocol_1");
        assert_eq!(protocol.data_type, ParameterDataType::UInt8);

        let cs_angle = parameter_info_for_model("rs05", 0x3035).expect("rs05 cs_angle");
        assert_eq!(cs_angle.name, "cs_angle");
        assert_eq!(cs_angle.data_type, ParameterDataType::Float32);

        let h = parameter_info_for_model("rs-05", 0x304E).expect("rs05 H");
        assert_eq!(h.name, "H");
        assert_eq!(h.data_type, ParameterDataType::UInt8);
    }
}
